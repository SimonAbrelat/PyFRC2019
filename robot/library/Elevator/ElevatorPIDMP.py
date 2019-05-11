from ElevatorPID import ElevatorPID
import frccontrol as frccnt


class ElevatorPIDMP:
    dt = 0.02

    index = 0

    s_profile = False

    def __init__():
        pass

    def set_ceofficients(self, kp, ki, kd, kv, ka):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kv = kv
        self.ka = ka

    def set_constaints(self, max_v, time_max_v):
        self.max_v = max_v
        self.time_max_v = time_max_v
        self.s_profile = False

    def set_s_curve_constaints(self, max_v, max_a, time_max_a):
        self.max_v = max_v
        self.max_a = max_a
        self.time_max_a = time_max_a
        self.s_profile = True

    def set_position(self, pos):
        _, self.x, self.v, self.a = (
            frccnt.generate_trapezoid_profile(
                self.max_v, self.time_max_v, self.dt, pos
            )
            if self.s_profile
            else frccnt.generate_s_curve_profile(
                self.max_v, self.max_a, self.time_max_a, self.dt, pos
            )
        )
        self.index = 0
        self.error_sum = 0

    def update(self, curr):
        def index(iter, ind):
            if ind > len(iter) - 1:
                return iter[:-1]
            return iter[ind]

        x = index(self.x, self.index)
        v = index(self.v, self.index)
        a = index(self.a, self.index)
        self.index += 1

        error = x - curr
        self.error_sum += error
        output = (
            (self.kp * error)
            + (self.ki * self.error_sum)
            + (self.kd * ((error - self.error_prev) / self.dt))
            + (self.kv * v)
            + (self.ka * a)
        )
        self.error_prev = error
        return output


