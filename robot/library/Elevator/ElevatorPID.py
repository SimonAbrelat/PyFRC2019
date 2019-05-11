from .. import PID


class ElevatorPID():
    def __init__():
        pass

    def set_ceofficients(self, kp, ki, kd, kf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.controller = PID.PID(kp, ki, kd, kf)

    def set_position(self, pos):
        self.controller.setpoint(pos)

    def set_error(self, error):
        self.controller.solveError(error)

    def update(self, curr):
        return self.controller.update(curr)