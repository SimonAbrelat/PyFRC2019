#!/usr/bin/env python3

import math

import navx
import pathfinder as pf
import wpilib
import wpilib.drive
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre.wpi_victorspx import WPI_VictorSPX
from magicbot import MagicRobot
from robotpy_ext.autonomous import AutonomousModeSelector
from wpilib.buttons import JoystickButton

from components import drive, arm


class MyRobot(MagicRobot):

    #
    # Define components here
    #
    drive = drive.Drive
    arm = arm.Arm

    #
    # Define constants here
    #
    # Robot attributes
    WHEEL_DIAMETER = 0.5  # 6 inches
    ENCODER_COUNTS_PER_REV = 360

    # Pathfinder constants
    MAX_VELOCITY = 5  # ft/s
    MAX_ACCELERATION = 6

    def createObjects(self):
        """
        Initialize all wpilib motors & sensors
        """
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)
        self.btn_fine_movement = JoystickButton(self.joystick_right, 2)

        # Drive motor controllers
        # ID SCHEME:
        #   10^1: 1 = left, 2 = right
        #   10^0: 0 = front, 5 = rear
        self.lf_motor = WPI_TalonSRX(10)
        self.lr_motor = WPI_VictorSPX(15)
        self.rf_motor = WPI_TalonSRX(20)
        self.rr_motor = WPI_VictorSPX(25)
        self.lf_motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Absolute
        )
        self.rf_motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Absolute
        )
        # Following masters
        self.lr_motor.follow(self.lf_motor)
        self.rr_motor.follow(self.rf_motor)
        # Drive init
        self.train = wpilib.drive.DifferentialDrive(
            self.lf_motor, self.rf_motor
        )

        # Arm
        self.arm_motor = WPI_TalonSRX(0)
        self.arm_motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Absolute
        )
        self.arm_limit = wpilib.DigitalInput(4)

        # Gyro
        self.gyro = navx.AHRS.create_spi()
        self.gyro.reset()

        self.control_loop_wait_time = 0.02

    def autonomous(self):
        """
        Prepare for and start autonomous mode.
        """
        self.drive.reset()
        self.drive.squared_inputs = False
        self.drive.rotational_constant = 0.5
        # Call autonomous
        super().autonomous()

    def teleopPeriodic(self):
        """
        Place code here that does things as a result of operator
        actions
        """
        self.drive.move(
            -self.joystick_left.getY(),
            self.joystick_right.getX(),
            self.btn_fine_movement.get(),
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
