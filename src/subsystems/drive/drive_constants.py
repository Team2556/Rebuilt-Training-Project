import math
from collections.abc import Callable

from wpilib import SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Translation2d, Transform2d, Rotation2d, Pose2d
from wpimath import units, angleModulus

from phoenix6.swerve import requests
from phoenix6.configs import CurrentLimitsConfigs
from phoenix6.swerve.swerve_module import SwerveModule

# from choreo.trajectory import SwerveSample

from lib.util.unit_types import *
from lib.util.controller.synchronous_pidf import SynchronousPIDF
from lib.util.math_helpers import MathHelpers
from lib.util.util import Util
from lib.util.tunablenumbers.tunable_number import TunableNumber
# from lib.logging.log_util import LogUtil

from src.subsystems.drive.tuner_constants import TunerConstants

from src.controlboard.controlboard_constants import ControlboardConstants

class DriveConstants:
    MAX_SPEED = TunerConstants.speed_at_12_volts
    MAX_SPEED_FAST = MAX_SPEED * 2.0
    MAX_ACCELERATION = LinearAcceleration.of(12.0)
    MAX_ACCELERATION_FAST = LinearAcceleration.of(12.0)

    MAX_ANGULAR_RATE = AngularVelocity.of(11.0)
    MAX_ANGULAR_RATE_FAST = MAX_ANGULAR_RATE * 2.0
    MAX_ANGULAR_ACCELERATION = AngularAcceleration.of(50.0)

    MINIMUM_FEED_FOWARD_ACTIVATION = TunableNumber("Feed Foward/Activation Degrees", 0.0)
    FEED_FOWARD_LOW = TunableNumber("Feed Foward/Low", 0.0)
    FEED_FOWARD_CONSTANT = TunableNumber("Feed Foward/Constant", 0.8)
    FEED_FOWARD_SCALE_PERCENT = TunableNumber("Feed Foward/Scale Percent", 100.0)

    PIGEON_OFFSET = Translation2d(0, 0)
    SHOOTER_OFFSET = Transform2d(units.inchesToMeters(-10.432), 0, math.pi * 0.5)

    UNSTABLE_ROTATION_RATE = AngularVelocity.of(math.pi * 1.8)
    PHASE_DELAY = Millisecond.of(20.0)

    empty_swerve_request = requests.FieldCentric()
    pid_to_pose_request = (requests.FieldCentric()
        .with_deadband(MAX_SPEED * ControlboardConstants.STICK_DEADBAND / 10)
        .with_rotational_deadband(MAX_ANGULAR_RATE * ControlboardConstants.STICK_DEADBAND / 10)
        .with_forward_perspective(requests.ForwardPerspectiveValue.BLUE_ALLIANCE)
        .with_desaturate_wheel_speeds(True)
        .with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
    )
    heading_lock_request = requests.FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
    teleop_request = requests.FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
    aiming_request = requests.FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.VELOCITY)
    brake_request = requests.SwerveDriveBrake()

    DRIVE_INITIAL_ROTATION = Rotation2d(0)
    
    MAX_PITCH_STABLE_THRESHOLD = Degrees.of(5.0)
    MAX_YAW_STABLE_THRESHOLD = Degrees.per(Seconds).of(100)

    @staticmethod
    def get_deadbanded_stick(raw_value: float) -> float:
        if abs(raw_value) < ControlboardConstants.STICK_DEADBAND:
            return 0.0
        else:
            unsigned_value = (abs(raw_value) - ControlboardConstants.STICK_DEADBAND) / (
                1.0 - ControlboardConstants.STICK_DEADBAND
            )
            return unsigned_value if raw_value > 0 else -unsigned_value

    @staticmethod
    def teleop_request_updater(request: requests.FieldCentric) -> requests.FieldCentric:
        x_desired_raw = -ControlboardConstants.m_driver_controller.getLeftY()
        y_desired_raw = -ControlboardConstants.m_driver_controller.getLeftX()
        rot_desired_raw = -ControlboardConstants.m_driver_controller.getRightX()
        
        x_fancy = DriveConstants.get_deadbanded_stick(x_desired_raw)
        y_fancy = DriveConstants.get_deadbanded_stick(y_desired_raw)
        rot_fancy = DriveConstants.get_deadbanded_stick(rot_desired_raw)

        target_omega = DriveConstants.MAX_ANGULAR_RATE * rot_fancy

        target_x = DriveConstants.MAX_SPEED * x_fancy
        target_y = DriveConstants.MAX_SPEED * y_fancy
        target_magnitude = MathHelpers.hypot(target_x, target_y)

        SmartDashboard.putNumber("Sticks/Target/Omega Radians Per Second", target_omega)
        SmartDashboard.putNumber("Sticks/Target/X Meters Per Second", target_x)
        SmartDashboard.putNumber("Sticks/Target/Y Meters Per Second", target_y)
        SmartDashboard.putNumber("Sticks/Target/Translation Magnitude", target_magnitude)

        SmartDashboard.putNumber("Sticks/hypot/raw", math.hypot(x_desired_raw, y_desired_raw))

        return request.with_velocity_x(target_x).with_velocity_y(target_y).with_rotational_rate(target_omega)

    @staticmethod
    def get_pid_to_pose_request_updater(target_pose: Pose2d, translation_controller: PIDController, heading_controller: PIDController) -> Callable[[requests.FieldCentric], requests.FieldCentric]:
        def updater(request: requests.FieldCentric) -> requests.FieldCentric:
            from src.subsystems.drive.drive import Drive

            current_pose = Drive.m_instance.get_pose()
            
            translation_error = current_pose.translation() - target_pose.translation()
            translation_error_norm = translation_error.norm()
            if translation_error_norm < 1e-9:
                scaled_output = Translation2d()
            else:
                unscaled_output = translation_error / translation_error_norm
                scaled_output = unscaled_output * translation_controller.calculate(translation_error_norm)
            
            commanded_velocity_x = scaled_output.X()
            commanded_velocity_y = scaled_output.Y()
            
            heading_controller.setSetpoint(units.radiansToRotations(angleModulus(target_pose.rotation().radians())))
            commanded_rotation_rate = heading_controller.calculate(units.radiansToRotations(angleModulus(current_pose.rotation().radians())))
            
            return (request
                .with_velocity_x(commanded_velocity_x)
                .with_velocity_y(commanded_velocity_y)
                .with_rotational_rate(commanded_rotation_rate)
            )
        
        return updater
    
    @staticmethod
    def get_tele_config() -> CurrentLimitsConfigs:
        return (CurrentLimitsConfigs()
            .with_stator_current_limit(60.0)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(30.0)
            .with_supply_current_lower_limit(30.0)
            .with_supply_current_limit_enable(True)
            .with_supply_current_lower_time(Seconds.of(1.0))
        )