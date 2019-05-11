import math

import navx
import wpilib
import wpilib.drive
from ctre.wpi_talonsrx import WPI_TalonSRX
from magicbot import tunable, will_reset_to


class Drive:
    """
    All Drive iterations go through this class
    """

    train: wpilib.drive.DifferentialDrive
    lf_motor = WPI_TalonSRX
    rf_motor = WPI_TalonSRX
    gyro: navx.AHRS

    isTank = will_reset_to(False)
    y = will_reset_to(0)
    rot = will_reset_to(0)
    left = will_reset_to(0)
    right = will_reset_to(0)

    speed_constant = tunable(1.05)
    rotational_constant = tunable(0.5)
    squared_inputs = tunable(False)

    fine_movement = will_reset_to(False)
    fine_speed_multiplier = tunable(0.5)
    fine_rotation_multiplier = tunable(0.5)

    # Robot attributes
    WHEEL_DIAMETER = 0.5  # 6 inches
    ENCODER_COUNTS_PER_REV = 360

    def setup(self):
        """
        Set input threshold.
        """
        self.train.setDeadband(0.1)

    def reset(self):
        self.lf_motor.setSelectedSensorPosition(0, 0)
        self.rf_motor.setSelectedSensorPosition(0, 0)

    def move(self, y: float, rot: float, fine_movement: bool = False):
        """
        Move robot.
        :param y: Speed of motion in the y direction. [-1..1]
        :param rot: Speed of rotation. [-1..1]
        :param fine_movement: Decrease speeds for precise motion.
        """
        self.isTank = False
        self.y = y
        self.rot = rot
        self.fine_movement = fine_movement

    def tank(self, l: float, r: float):
        """
        Tank control basically just for auto
        :param l: Left speed [-1..1]
        :param r: Right Speed [-1..1]
        """
        self.isTank = True
        self.left = l
        self.right = r

    def distance(self, motor):
        return (
            motor.getSelectedSensorPosition()
            * (math.pi * self.WHEEL_DIAMETER)
            / self.ENCODER_COUNTS_PER_REV
        )

    def leftDistance(self):
        return self.distance(self.lf_motor)

    def rightDistance(self):
        return self.distance(self.rf_motor)

    def execute(self):
        """
        Handle driving.
        """
        if self.isTank:
            self.train.tankDrive(self.left, self.right)
            return
        self.train.arcadeDrive(
            self.speed_constant
            * self.y
            * (self.fine_speed_multiplier if self.fine_movement else 1),
            self.rotational_constant
            * self.rot
            * (self.fine_rotation_multiplier if self.fine_movement else 1),
            squareInputs=self.squared_inputs,
        )
