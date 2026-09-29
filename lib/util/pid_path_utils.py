import math

from wpimath import angleModulus, units
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from .controller.synchronous_pidf import SynchronousPIDF
from .math_helpers import MathHelpers

ROTATION_FEED_FORWARD = 0.01


class PidPathUtils:

    @staticmethod
    def calculate_speeds(
        current_pose: Pose2d,
        target_pose: Pose2d,
        x_controller: PIDController,
        y_controller: PIDController,
        theta_controller: PIDController,
    ) -> ChassisSpeeds:
        rotation_speed = theta_controller.calculate(
            current_pose.rotation().radians(), target_pose.rotation().radians()
        )

        if not MathHelpers.is_near(
            target_pose.rotation().degrees(), current_pose.rotation().degrees(), 1.0, -180, 180
        ):
            rotation_speed += math.copysign(
                units.rotationsToRadians(ROTATION_FEED_FORWARD), rotation_speed
            )

        return ChassisSpeeds(
            x_controller.calculate(current_pose.X(), target_pose.X()),
            y_controller.calculate(current_pose.Y(), target_pose.Y()),
            rotation_speed,
        )

    @staticmethod
    def calculate_combined(
        current_pose: Pose2d,
        target_pose: Pose2d,
        translation_controller: PIDController,
        theta_controller: PIDController,
    ) -> ChassisSpeeds:
        # We flip the sign here to produce the right output (negative error = pos output)
        # Could alternatively do (target - current).unaryMinus() but this is just cleaner

        translation_error = current_pose.translation() - target_pose.translation()

        u_error = translation_error / translation_error.norm()
        velocity = u_error * translation_controller.calculate(translation_error.norm())

        return ChassisSpeeds(
            velocity.X(),
            velocity.Y(),
            theta_controller.calculate(
                angleModulus(current_pose.rotation().radians()),
                angleModulus(target_pose.rotation().radians()),
            ),
        )

    @staticmethod
    def calculate_combined_sync(
        input: Pose2d,
        output: Pose2d,
        translation_controller: SynchronousPIDF,
        theta_controller: SynchronousPIDF,
    ) -> ChassisSpeeds:
        # We flip the sign here to produce the right output (negative error = pos output)
        # Could alternatively do (target - current).unaryMinus() but this is just cleaner

        translation_error = input.translation() - output.translation()

        u_error = translation_error / translation_error.norm()
        velocity = u_error * translation_controller.calculate(translation_error.norm())

        theta_controller.set_setpoint(angleModulus(output.rotation().radians()))

        return ChassisSpeeds(
            velocity.X(),
            velocity.Y(),
            theta_controller.calculate(angleModulus(input.rotation().radians())),
        )

    @staticmethod
    def calculate_sync(
        drive_pose: Pose2d,
        desired: Pose2d,
        x_controller: SynchronousPIDF,
        y_controller: SynchronousPIDF,
        heading_controller: SynchronousPIDF,
    ) -> ChassisSpeeds:

        return ChassisSpeeds(
            x_controller.set_setpoint_and_calculate(desired.X(), drive_pose.X()),
            y_controller.set_setpoint_and_calculate(desired.Y(), drive_pose.Y()),
            heading_controller.set_setpoint_and_calculate(
                desired.rotation().radians(), drive_pose.rotation().radians()
            ),
        )
