import frccontrol as frccnt
import numpy as np
from .. import StateSpaceController


class ElevatorSS:

    s_profile = False
    motion_profile = False

    def __init__(self, motor, num_motors, m, r, G, dt):
        self.dt = dt
        self.sys = frccnt.models.elevator(motor, num_motors, m, r, G)
        self.controller = StateSpaceController.StateSpaceController(
            self.sys, np.array([[-12.0]]), np.array([[12.0]]), dt
        )
        q = [0.02, 0.4]
        r = [12.0]
        self.controller.design_lqr(q, r)

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.controller.design_kalman_filter([q_pos, q_vel], [r_pos])

    def set_motion_profile_constaints(self, max_v, time_max_v):
        self.max_v = max_v
        self.time_max_v = time_max_v
        self.s_profile = False

    def set_s_curve_motion_profile_constaints(self, max_v, max_a, time_max_a):
        self.max_v = max_v
        self.max_a = max_a
        self.time_max_a = time_max_a
        self.s_profile = True

    def set_position(self, pos):
        self.reference = [[pos], [0]]
        self.motion_profile = False

    def set_reference(self, pos, vel):
        self.reference = [[pos], [vel]]

    def set_motion_profile_position(self, pos):
        _, self.mp_x, self.mp_v, _ = (
            frccnt.generate_trapezoid_profile(
                self.max_v, self.time_max_v, self.dt, pos
            )
            if self.s_profile
            else frccnt.generate_s_curve_profile(
                self.max_v, self.max_a, self.time_max_a, self.dt, pos
            )
        )
        self.motion_profile = True
        self.index = 0

    def execute(self, position, velocity):
        if self.motion_profile and (self.index < len(self.mp_x) - 1):
            self.reference = [[self.mp_x[self.index]], [self.mp_v[self.index]]]

        self.controller.update_kalman_filter([[position], [velocity]])
        self.controller.update_lqr(self.reference)
        return self.controller.u[0, 0]
