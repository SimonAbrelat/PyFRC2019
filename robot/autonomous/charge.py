import math

import pathfinder as pf
import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components import drive


class Pathing(AutonomousStateMachine):
    MODE_NAME = "Pathing"
    DEFAULT = True

    drive = drive.Drive

    # Pathfinder constants
    MAX_VELOCITY = 5  # ft/s
    MAX_ACCELERATION = 6
    DRIVE_WIDTH = 2

    def on_enable(self):
        super().on_enable()
        points = [pf.Waypoint(0, 0, 0), pf.Waypoint(13, -3, 0)]

        info, trajectory = pf.generate(
            points,
            pf.FIT_HERMITE_CUBIC,
            pf.SAMPLES_HIGH,
            dt=0.02,
            max_velocity=self.MAX_VELOCITY,
            max_acceleration=self.MAX_ACCELERATION,
            max_jerk=120.0,
        )

        # Wheelbase Width = 2 ft
        modifier = pf.modifiers.TankModifier(trajectory).modify(
            self.DRIVE_WIDTH
        )

        # Do something with the new Trajectories...
        left = modifier.getLeftTrajectory()
        right = modifier.getRightTrajectory()

        leftFollower = pf.followers.DistanceFollower(left)
        leftFollower.configurePIDVA(
            0.001,
            0.0,
            0.0,
            2.0 / (3 * self.MAX_VELOCITY),
            1.0 / (12 * self.MAX_ACCELERATION),
        )

        rightFollower = pf.followers.DistanceFollower(right)
        rightFollower.configurePIDVA(
            0.001,
            0.0,
            0.0,
            2.0 / (3 * self.MAX_VELOCITY),
            1.0 / (12 * self.MAX_ACCELERATION),
        )

        self.leftFollower = leftFollower
        self.rightFollower = rightFollower

        # This code renders the followed path on the field in simulation
        # (requires pyfrc 2018.2.0+)
        if wpilib.RobotBase.isSimulation():
            from pyfrc.sim import get_user_renderer

            renderer = get_user_renderer()
            if renderer:
                renderer.draw_pathfinder_trajectory(
                    left, color="#0000ff", offset=(-1, 0)
                )
                renderer.draw_pathfinder_trajectory(
                    modifier.source,
                    color="#00ff00",
                    show_dt=1.0,
                    dt_offset=0.0,
                )
                renderer.draw_pathfinder_trajectory(
                    right, color="#0000ff", offset=(1, 0)
                )

    @state(first=True)
    def charge(self, initial_call):
        left_speed = self.leftFollower.calculate(self.drive.leftDistance())
        right_speed = self.rightFollower.calculate(self.drive.rightDistance())

        gyro_heading = (
            -self.drive.gyro.getAngle()
        )  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(
            self.leftFollower.getHeading()
        )  # Should also be in degrees

        # This is a poor man's P controller
        angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = 1.2 * (-1.0 / 80.0) * angleDifference

        left_speed += turn
        right_speed -= turn
        print(left_speed, right_speed)

        # -1 is forward, so invert both values
        self.drive.tank(left_speed, right_speed)
