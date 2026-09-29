import math
from typing import Callable

from choreo.trajectory import SwerveSample
from commands2 import Command, Subsystem, cmd
from ntcore import NetworkTableInstance
from phoenix6 import swerve
from phoenix6.configs import CurrentLimitsConfigs
from wpilib import Field2d, SmartDashboard
from wpimath.filter import Debouncer, LinearFilter
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds
from wpiutil import SendableBuilder

# from lib.logging.log_util import LogUtil
# from lib.logging.logged_tracer import LoggedTracer
from lib.math.chassis_accelerations import ChassisAccelerations
from lib.math.kinematic_helpers import KinematicHelpers
from lib.util.tracker.derivative_tracker import DerivativeTracker
from lib.util.tracker.linear_filter_derivative_tracker import LinearFilterDerivativeTracker

from src.telemetry import Telemetry
from src.subsystems.drive.drive_constants import DriveConstants
from src.subsystems.drive.generated_drivetrain import GeneratedDrivetrain
from src.subsystems.drive.tuner_constants import TunerConstants

_STANDARD_GRAVITY = 9.80665


class Drive(Subsystem):
    m_instance: "Drive"

    def __init__(self) -> None:
        super().__init__()

        self.drive_request: swerve.requests.SwerveRequest = DriveConstants.teleop_request
        self.drivetrain: GeneratedDrivetrain = TunerConstants.create_drivetrain()
        self.telemetry = Telemetry(DriveConstants.MAX_SPEED)

        self.m_translation_acceleration_x_filtered = LinearFilterDerivativeTracker(
            LinearFilter.movingAverage(5), 0.00
        )
        self.m_translation_acceleration_y_filtered = LinearFilterDerivativeTracker(
            LinearFilter.movingAverage(5), 0.00
        )

        self.m_translation_acceleration_x = DerivativeTracker(0.0)
        self.m_translation_acceleration_y = DerivativeTracker(0.0)

        self.m_translation_acceleration = Translation2d()

        self.m_angular_acceleration_tracker_filtered = LinearFilterDerivativeTracker(
            LinearFilter.movingAverage(5), 0.0
        )
        self.m_angular_acceleration_tracker = DerivativeTracker(0.0)

        self.m_filtered_translation_velocity_x = 0.0
        self.m_filtered_translation_velocity_y = 0.0
        self.m_filtered_angular_velocity = 0.0

        self.m_translation_x_velocity_filter = LinearFilter.movingAverage(5)
        self.m_translation_y_velocity_filter = LinearFilter.movingAverage(5)
        self.m_angular_velocity_filter = LinearFilter.movingAverage(5)

        self.yaw_debouncer = Debouncer(0.1)
        self.pitch_debouncer = Debouncer(0.1)

        self.mechanism_publisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic("Mechanisms/Drivetrain", Pose3d)
            .publish()
        )

        self.elastic_pose = Field2d()

        self.last_read_state = self.drivetrain.get_state()
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: self.drive_request)
        )

        self.drivetrain.odometry_thread.set_thread_priority(31)

    def get_generated_drive(self) -> GeneratedDrivetrain:
        return self.drivetrain

    def get_translation_acceleration_from_pigeon(
        self, pigeon_offset: Translation2d
    ) -> Translation2d:
        omega_rads_per_second = Drive.m_instance.get_state().speeds.omega
        pigeon = self.drivetrain.pigeon2
        return Translation2d(
            pigeon.get_acceleration_x().value * _STANDARD_GRAVITY
            - (self.get_angular_acceleration() * pigeon_offset.X())
            - (omega_rads_per_second * omega_rads_per_second * pigeon_offset.X()),
            pigeon.get_acceleration_y().value * _STANDARD_GRAVITY
            - (self.get_angular_acceleration() * pigeon_offset.Y())
            - (omega_rads_per_second * omega_rads_per_second * pigeon_offset.Y()),
        )

    def get_filtered_accelerations(self) -> ChassisAccelerations:
        return ChassisAccelerations(
            self.m_translation_acceleration_x_filtered.get_tracked_derivative(),
            self.m_translation_acceleration_y_filtered.get_tracked_derivative(),
            self.m_angular_acceleration_tracker_filtered.get_tracked_derivative(),
        )

    def get_accelerations(self) -> ChassisAccelerations:
        return ChassisAccelerations(
            self.m_translation_acceleration_x.get_tracked_derivative(),
            self.m_translation_acceleration_y.get_tracked_derivative(),
            self.m_angular_acceleration_tracker.get_tracked_derivative(),
        )

    def get_filtered_chasis_speeds(self) -> ChassisSpeeds:
        return ChassisSpeeds(
            self.m_filtered_translation_velocity_x,
            self.m_filtered_translation_velocity_y,
            self.m_filtered_angular_velocity,
        )

    def get_translation_acceleration_from_pigeon_default(self) -> Translation2d:
        return self.get_translation_acceleration_from_pigeon(DriveConstants.PIGEON_OFFSET)

    def get_translation_acceleration(self) -> Translation2d:
        return self.m_translation_acceleration

    def get_filtered_translation_acceleration(self) -> Translation2d:
        return Translation2d(
            self.m_translation_acceleration_x_filtered.get_tracked_derivative(),
            self.m_translation_acceleration_y_filtered.get_tracked_derivative(),
        )

    def get_angular_acceleration(self) -> float:
        return self.m_angular_acceleration_tracker_filtered.get_tracked_derivative()

    def periodic(self) -> None:
        self.last_read_state = self.drivetrain.get_state()
        self.update_acceleration()
        self.output_telemetry()
        self.update_filtered_velocity()
        # LoggedTracer.record("Drive")

    def update_filtered_velocity(self) -> None:
        speeds = Drive.m_instance.get_state().speeds
        self.m_filtered_translation_velocity_x = self.m_translation_x_velocity_filter.calculate(
            speeds.vx
        )
        self.m_filtered_translation_velocity_y = self.m_translation_y_velocity_filter.calculate(
            speeds.vy
        )
        self.m_filtered_angular_velocity = self.m_angular_velocity_filter.calculate(
            speeds.omega
        )

    def update_acceleration(self) -> None:
        speeds = self.get_state().speeds
        self.m_translation_acceleration_x_filtered.update(speeds.vx, 0.02)
        self.m_translation_acceleration_y_filtered.update(speeds.vy, 0.02)
        self.m_angular_acceleration_tracker_filtered.update(speeds.omega, 0.02)

        self.m_translation_acceleration_x.update(speeds.vx, 0.02)
        self.m_translation_acceleration_y.update(speeds.vy, 0.02)
        self.m_angular_acceleration_tracker.update(speeds.omega, 0.02)

    def output_telemetry(self) -> None:
        self.mechanism_publisher.set(Pose3d(self.get_pose()))
        self.telemetry.telemeterize(self.last_read_state)
        SmartDashboard.putData("Drive", self)
        self.elastic_pose.setRobotPose(self.get_pose())
        # LogUtil.log("Drive/Compensated Pose", self.get_phase_delayed_pose())
        SmartDashboard.putData("Elastic Field 2D", self.elastic_pose)

    def get_phase_delayed_pose(self) -> Pose2d:
        speeds = self.get_state().speeds
        dt = DriveConstants.PHASE_DELAY
        return KinematicHelpers.second_order_approximate(
            self.get_pose(), speeds, self.get_filtered_accelerations(), dt
        )

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addStringProperty(
            "Swerve Request Type",
            lambda: type(self.drive_request).__name__,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Pitch Velocity Degrees Per Second",
            lambda: self.drivetrain.pigeon2.get_angular_velocity_y_device().value,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Pitch Degrees",
            lambda: self.drivetrain.pigeon2.get_pitch().value,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Angular Acceleration Radians Per Second Per Second",
            lambda: self.get_angular_acceleration(),
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Roll Velocity Degrees Per Second",
            lambda: self.drivetrain.pigeon2.get_angular_velocity_x_device().value,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Roll Degrees",
            lambda: self.drivetrain.pigeon2.get_roll().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Tilt",
            lambda: math.degrees(self.get_field_aligned_pigeon_angle().Y()),
            lambda v: None,
        )

        builder.addBooleanProperty(
            "Pigeon Connected",
            lambda: self.drivetrain.pigeon2.is_connected,
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Field Velocity/X Meters Per Second",
            lambda: self.get_field_relative_speeds().vx,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Field Velocity/Y Meters per Second",
            lambda: self.get_field_relative_speeds().vy,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Field Velocity/O Radians per Second",
            lambda: self.get_field_relative_speeds().omega,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Field Velocity/Direction Meters per Second",
            lambda: math.hypot(
                self.get_field_relative_speeds().vx, self.get_field_relative_speeds().vy
            ),
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Filtered Field Velocity/X Meters Per Second",
            lambda: self.m_filtered_translation_velocity_x,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Filtered Field Velocity/Y Meters per Second",
            lambda: self.m_filtered_translation_velocity_y,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Filtered Field Velocity/O Radians per Second",
            lambda: self.m_filtered_angular_velocity,
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Filtered Field Velocity/Direction Meters per Second",
            lambda: math.hypot(
                self.m_filtered_translation_velocity_x, self.m_filtered_translation_velocity_y
            ),
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Filtered Acceleration Translation/X Meters Per Second Per Second",
            lambda: self.m_translation_acceleration_x_filtered.get_tracked_derivative(),
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Filtered Acceleration Translation/Y Meters Per Second Per Second",
            lambda: self.m_translation_acceleration_y_filtered.get_tracked_derivative(),
            lambda v: None,
        )

        builder.addDoubleProperty(
            "Acceleration Translation/X Meters Per Second Per Second",
            lambda: self.m_translation_acceleration_x.get_tracked_derivative(),
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Acceleration Translation/Y Meters Per Second Per Second",
            lambda: self.m_translation_acceleration_y.get_tracked_derivative(),
            lambda v: None,
        )

        builder.addBooleanProperty("Pitch Stable", lambda: self.pitch_stable(), lambda v: None)
        builder.addDoubleProperty(
            "Translation Velocity",
            lambda: math.hypot(self.get_state().speeds.vx, self.get_state().speeds.vy),
            lambda v: None,
        )
        self.add_module_to_builder(builder, 0)
        self.add_module_to_builder(builder, 1)
        self.add_module_to_builder(builder, 2)
        self.add_module_to_builder(builder, 3)

    def add_module_to_builder(self, builder: SendableBuilder, module: int) -> None:
        builder.addDoubleProperty(
            f"ModuleStates/{module}/Drive/Volts",
            lambda: self.drivetrain.modules[module].drive_motor.get_motor_voltage().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Rotation/Volts",
            lambda: self.drivetrain.modules[module].steer_motor.get_motor_voltage().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Drive/Stator Current",
            lambda: self.drivetrain.modules[module].drive_motor.get_stator_current().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Drive/Temperature Celsius",
            lambda: self.drivetrain.modules[module].drive_motor.get_device_temp().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Rotation/Stator Current",
            lambda: self.drivetrain.modules[module].steer_motor.get_stator_current().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Drive/Supply Current",
            lambda: self.drivetrain.modules[module].drive_motor.get_supply_current().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Rotation/Supply Current",
            lambda: self.drivetrain.modules[module].steer_motor.get_supply_current().value,
            lambda v: None,
        )

        builder.addDoubleProperty(
            f"ModuleStates/{module}/Rotation/Temperature Celsius",
            lambda: self.drivetrain.modules[module].steer_motor.get_device_temp().value,
            lambda v: None,
        )

    def get_look_ahead_pose_without_heading(self, look_ahead_time: float) -> Pose2d:
        speeds = self.get_state().speeds
        return self.get_pose().exp(
            Twist2d(
                speeds.vx * look_ahead_time,
                speeds.vy * look_ahead_time,
                0.0,
            )
        )

    def get_look_ahead_pose(self, look_ahead_time: float) -> Pose2d:
        speeds = self.get_state().speeds
        return self.get_pose().exp(
            Twist2d(
                speeds.vx * look_ahead_time,
                speeds.vy * look_ahead_time,
                speeds.omega * look_ahead_time,
            )
        )

    def get_look_ahead_heading(self, look_ahead_time: float) -> Rotation2d:
        speeds = self.get_state().speeds
        return self.get_pose().exp(
            Twist2d(0.0, 0.0, speeds.omega * look_ahead_time)
        ).rotation()

    def get_state(self) -> swerve.SwerveDrivetrain.SwerveDriveState:
        return self.last_read_state

    def get_field_relative_speeds(self) -> ChassisSpeeds:
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            self.get_state().speeds, self.get_pose().rotation()
        )

    def get_pose(self) -> Pose2d:
        return self.last_read_state.pose

    def set_swerve_request(self, request: swerve.requests.SwerveRequest) -> None:
        self.drive_request = request
        self.drivetrain.set_control(request)

    def follow_swerve_request_command(
        self,
        request: swerve.requests.FieldCentric,
        updater: Callable[[swerve.requests.FieldCentric], swerve.requests.FieldCentric],
    ) -> Command:
        return self.run(lambda: self.set_swerve_request(updater(request))).handleInterrupt(
            lambda: self.set_swerve_request(swerve.requests.FieldCentric())
        )

    def brake_command(self) -> Command:
        """Points the wheels into an X so the robot is hard to push, for as long as it is held."""
        return self.run(lambda: self.set_swerve_request(DriveConstants.brake_request)).withName(
            "X Brake"
        )

    def follow_choreo_trajectory(self, sample: SwerveSample) -> None:
        self.set_swerve_request(
            DriveConstants.get_choreo_path_request_updater(sample)(DriveConstants.choreo_request)
        )

    def add_vision_update(
        self,
        pose: Pose2d,
        timestamp: float,
        std_devs: tuple[float, float, float] | None = None,
    ) -> None:
        self.get_generated_drive().add_vision_measurement(pose, timestamp, std_devs)

    def reset_pose(self, pose: Pose2d) -> None:
        self.get_generated_drive().reset_pose(pose)

    def reset_pose_cmd(self, pose: Pose2d) -> Command:
        return cmd.runOnce(lambda: self.reset_pose(pose))

    def yaw_stable(self) -> bool:
        yaw_is_stable = (
            abs(self.drivetrain.pigeon2.get_angular_velocity_z_device().value)
            < DriveConstants.MAX_YAW_STABLE_THRESHOLD
        )

        return self.yaw_debouncer.calculate(yaw_is_stable)

    def pitch_stable(self) -> bool:
        pitch_is_stable = (
            abs(self.drivetrain.pigeon2.get_pitch().value)
            < DriveConstants.MAX_PITCH_STABLE_THRESHOLD
        )

        return self.pitch_debouncer.calculate(pitch_is_stable)

    def drive_stable(self) -> bool:
        SmartDashboard.putBoolean("Yaw Stable", self.yaw_stable())
        SmartDashboard.putBoolean("Pitch Stable", self.pitch_stable())
        return self.yaw_stable() and self.pitch_stable()

    def get_drive_speeds_as_translation(self) -> Translation2d:
        speed = Drive.m_instance.get_state().speeds
        return Translation2d(speed.vx, speed.vy)

    def get_field_aligned_pigeon_angle(self) -> Rotation3d:
        robot_rotation = Drive.m_instance.get_generated_drive().pigeon2.getRotation3d()
        SmartDashboard.putString("Pigeon Rotation3d", str(robot_rotation))
        robot_yaw = Rotation3d(
            0.0,
            0.0,
            Drive.m_instance.get_pose().rotation().radians(),
        )

        return (-robot_yaw) + robot_rotation

    def config_drivetain_current(self, config: CurrentLimitsConfigs) -> None:
        self.drivetrain.get_module(0).drive_motor.configurator.apply(config)
        self.drivetrain.get_module(1).drive_motor.configurator.apply(config)
        self.drivetrain.get_module(2).drive_motor.configurator.apply(config)
        self.drivetrain.get_module(3).drive_motor.configurator.apply(config)


Drive.m_instance = Drive()
