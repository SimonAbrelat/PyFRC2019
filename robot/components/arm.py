import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX

from magicbot import tunable, will_reset_to
from controllers.arm_pid import ArmPID
from library.Elevator.ElevatorPID import ElevatorPID


class Arm:
    """
    Single Jointed arm for that holds the hatch and cargo mechs
    """

    arm_motor = WPI_TalonSRX
    arm_limit = wpilib.DigitalInput
    reset_angle = will_reset_to(180)
    kp = tunable(1)
    ki = tunable(0)
    kd = tunable(0)
    kf = tunable(0)
    target_angle = will_reset_to(180)
    zeroed = False

    # Constants
    armRatio = 24 / 84

    def setup(self):
        self.arm_controller = ArmPID()
        self.blahhhh = ElevatorPID(self.kp, self.ki, self.kd, self.kf)
        self.arm_motor.clearStickyFaults()
        self.zeroed = False

    def set(self, angle):
        self.target_angle = angle
        self.arm_controller.resetIntegrator()

    def angle(self, ticks):
        return ticks * (360 * self.armRatio) / 4096

    def ticks(self, angle):
        return int(angle * (4096 / (360 * self.armRatio)))

    def execute(self):
        if not self.zeroed:
            self.arm_motor.set(0.2)
            if self.arm_limit.get():
                self.zeroed = True
                self.arm_motor.setSelectedSensorPosition(
                    self.ticks(self.reset_angle)
                )
            return  # exits out of the function if we aren't zeroed
        self.arm_motor.set(
            self.arm_controller.update(
                self.kp,
                self.ki,
                self.kd,
                self.kf,
                self.target_angle,
                self.angle(self.arm_motor.getSelectedSensorPosition()),
            )
        )
