import math

import wpilib


class ArmPID:
    tErr = 0
    pErr = 0
    pTime = 0.0

    def update(self, kp, ki, kd, kf, setp, angle):
        err = setp - angle
        self.tErr += err
        t = wpilib.Timer.getFPGATimestamp()
        dt = t - self.pTime
        self.pTime = t
        o = (
            (kp * err)  # Proporational
            + (ki * self.tErr * dt)  # Integral
            + (kd * ((err - self.pErr) / dt))  # Derivative
            + (kf * math.sin(angle * (math.pi / 180)))  # Feedforward
        )
        self.pErr = err

        def clip(x, l, u):
            max(l, min(u, x))

        clip(o, -1, 1)
        return -o

    def resetIntegrator(self):
        self.tErr = 0
