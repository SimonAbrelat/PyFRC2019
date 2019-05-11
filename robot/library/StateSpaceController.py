import control as cnt
import numpy as np
import scipy as sp


class StateSpaceController:

    __default = object()

    def __init__(self, sys, u_min, u_max, dt):
        # System
        self.sysc = sys
        self.sysd = sys.sample(dt)
        self.dt = dt

        # Model matrices
        self.x = np.zeros((self.sysc.A.shape[0], 1))
        self.u = np.zeros((self.sysc.B.shape[1], 1))
        self.y = np.zeros((self.sysc.C.shape[0], 1))

        # Observer matrices
        self.x_hat = np.zeros((self.sysc.A.shape[0], 1))
        self.u_min = np.asarray(u_min)
        self.u_max = np.asarray(u_max)
        self.P = np.zeros(self.sysc.A.shape)
        self.kalman_gain = np.zeros(
            (self.sysc.A.shape[0], self.sysc.C.shape[0])
        )

        # Controller matrices
        self.K = np.zeros((self.sysc.B.shape[1], self.sysc.B.shape[0]))
        self.Kff = np.zeros((self.sysc.B.shape[1], self.sysc.B.shape[0]))
        self.r = np.zeros((self.sysc.A.shape[0], 1))

    def design_lqr(self, Q_elems, R_elems):
        """
        Design a discrete time linear-quadratic regulator for the system.

        Keyword arguments:
        Q -- a vector of the maximum allowed excursions of the states from
             the reference.
        R -- a vector of the maximum allowed excursions of the control
             inputs from no actuation.
        """
        Q = self.__make_cost_matrix(Q_elems)
        R = self.__make_cost_matrix(R_elems)
        self.K = self.__lqr(self.sysd, Q, R)
        self.Kff = self.__feedforward(Q, R)

    def set_reference(self, r):
        """
        Setter for the reference

        Keyword arguments:
        next_r -- next controller reference (default: current reference)
        """
        self.r = r

    def __feedforward(self, Q=None, R=None):
        """
        Computes the feedforward constant for a two-state controller.

        This will take the form u = K_ff * (r_{n+1} - A r_n), where K_ff is the
        feed-forwards constant. It is important that Kff is *only* computed off
        the goal and not the feedback terms.

        If either Q_elems or R_elems is not specified, then both are ignored.

        Keyword arguments:
        Q_elems -- a vector of the maximum allowed excursions in the state
                   tracking.
        R_elems -- a vector of the maximum allowed excursions of the control
                   inputs from no actuation.
        """
        if Q is not None and R is not None:
            # We want to find the optimal U such that we minimize the tracking
            # cost. This means that we want to minimize
            #   (B u - (r_{n+1} - A r_n))^T Q (B u - (r_{n+1} - A r_n))
            #      + u^T R u
            self.Kff = (
                np.linalg.inv(self.sysd.B.T @ Q @ self.sysd.B + R.T) @ self.sysd.B.T @ Q
            )
        else:
            # Without Q and R weighting matrices, K_ff = B^+ where B^+ is the
            # Moore-Penrose pseudoinverse of B.
            self.Kff = np.linalg.pinv(self.sysd.B)

    def update_lqr(self, next_r=__default):
        """
        Advance the controller by one timestep.

        Keyword arguments:
        next_r -- next controller reference (default: current reference)
        """
        u = self.K @ (self.r - self.x_hat)
        if next_r is not self.__default:
            uff = self.Kff @ (next_r - self.sysd.A @ self.r)
            self.r = next_r
        else:
            uff = self.Kff @ (self.r - self.sysd.A @ self.r)
        self.u = np.clip(u + uff, self.u_min, self.u_max)

    def __lqr(self, Q, R):
        """
        Solves for the optimal linear-quadratic regulator (LQR).

        For a continuous system:
            xdot = A * x + B * u
            J = int(0, inf, x.T * Q * x + u.T * R * u)
        For a discrete system:
            x(n+1) = A * x(n) + B * u(n)
            J = sum(0, inf, x.T * Q * x + u.T * R * u)

        Keyword arguments:
        A -- numpy.array(states x states), The A matrix.
        B -- numpy.array(inputs x states), The B matrix.
        Q -- numpy.array(states x states), The state cost matrix.
        R -- numpy.array(inputs x inputs), The control effort cost matrix.

        Returns:
        numpy.array(states x inputs), K
        """
        m = self.sysc.A.shape[0]

        controllability_rank = np.linalg.matrix_rank(
            cnt.ctrb(self.sysc.A, self.sysc.B)
        )
        if controllability_rank != m:
            print(
                "Warning: Controllability of %d != %d, uncontrollable state"
                % (controllability_rank, m)
            )

        P = sp.linalg.solve_discrete_are(
            a=self.sysd.A, b=self.sysd.B, q=Q, r=R
        )
        return (
            np.linalg.inv(R + self.sysd.B.T @ P @ self.sysd.B) @ self.sysd.B.T @ P @ self.sysd.A
        )

    def __make_cost_matrix(self, elems):
        """Creates a cost matrix from the given vector for use with LQR.

        The cost matrix is constructed using Bryson's rule. The inverse square
        of each element in the input is taken and placed on the cost matrix
        diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the maximum allowed
                 excursions of the states from the reference. For an R matrix,
                 its elements are the maximum allowed excursions of the control
                 inputs from no actuation.

        Returns:
        State excursion or control effort cost matrix
        """
        return np.diag(1.0 / np.square(elems))

    def design_kalman_filter(self, Q_elems, R_elems):
        """
        Design a discrete time Kalman filter for the system.

        Keyword arguments:
        Q_elems -- a vector of the standard deviations of each state from how
                   the model behaves.
        R_elems -- a vector of the standard deviations of each output
                   measurement.
        """
        self.Q = self.__make_cov_matrix(Q_elems)
        self.R = self.__make_cov_matrix(R_elems)
        self.kalman_gain, self.P_steady = self.__kalmd(
            self.sysd, Q=self.Q, R=self.R
        )

    def update_kalman_filter(self, y):
        """
        Uses the Kalman filter to predict the next output
        """
        # Corrects the filter
        self.x_hat += self.kalman_gain * (
            y - self.sysd.C @ self.x_hat - self.sysd.D @ self.u
        )
        # Predicts next time step
        self.x_hat = self.sysd.A @ self.x_hat + self.sysd.B @ self.u

    def __kalmd(sys, Q, R):
        """
        Solves for the steady state kalman gain and error covariance matrices.

        Keyword arguments:
        sys -- discrete state-space model
        Q -- process noise covariance matrix
        R -- measurement noise covariance matrix

        Returns:
        Kalman gain, error covariance matrix.
        """
        m = sys.A.shape[0]

        observability_rank = np.linalg.matrix_rank(cnt.obsv(sys.A, sys.C))
        if observability_rank != m:
            print(
                "Warning: Observability of %d != %d, unobservable state"
                % (observability_rank, m)
            )

        # Compute the steady state covariance matrix
        P_prior = sp.linalg.solve_discrete_are(a=sys.A.T, b=sys.C.T, q=Q, r=R)
        S = sys.C * P_prior * sys.C.T + R
        K = P_prior * sys.C.T * np.linalg.inv(S)
        P = (np.eye(m) - K * sys.C) * P_prior
        return K, P

    def __make_cov_matrix(self, elems):
        """Creates a covariance matrix from the given vector for use with Kalman
        filters.

        Each element is squared and placed on the covariance matrix diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the standard
                 deviations of each state from how the model behaves. For an R
                 matrix, its elements are the standard deviations for each
                 output measurement.

        Returns:
        Process noise or measurement noise covariance matrix
        """
        return np.diag(np.square(elems))