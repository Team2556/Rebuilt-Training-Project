"""
Class that extends the Phoenix 6 SwerveDrivetrain class and implements
Subsystem so it can easily be used in command-based projects.
"""

import atexit
import math
from typing import Callable, overload

from phoenix6 import SignalLogger, swerve, units, utils
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import State, SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine

from .tuner_constants import TunerSwerveDrivetrain


class GeneratedDrivetrain(Subsystem, TunerSwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005  # 5 ms

    # Blue alliance sees forward as 0 degrees (toward red alliance wall)
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    # Red alliance sees forward as 180 degrees (toward blue alliance wall)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:   Drivetrain-wide constants for the swerve drive
        :param modules:                Constants for each specific module
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:     Drivetrain-wide constants for the swerve drive
        :param odometry_update_frequency: The frequency to run the odometry loop. If
                                          unspecified or set to 0 Hz, this is 250 Hz on
                                          CAN FD, and 100 Hz on CAN 2.0.
        :param modules:                  Constants for each specific module
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:       Drivetrain-wide constants for the swerve drive
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :param modules:                     Constants for each specific module
        """
        ...

    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        TunerSwerveDrivetrain.__init__(self, drivetrain_constants, arg0, arg1, arg2, arg3)

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        # Keep track if we've ever applied the operator perspective before or not
        self._has_applied_operator_perspective = False

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic step voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: self._log_sys_id_state(
                    "SysIdTranslation_State", state
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: self._log_sys_id_state(
                    "SysIdSteer_State", state
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.set_control(
                    self._steer_characterization.with_volts(volts)
                ),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing rotation.
        # This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        # See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second, but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # This is in radians per second, but SysId only supports "volts"
                stepVoltage=math.pi,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: self._log_sys_id_state(
                    "SysIdRotation_State", state
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._rotation_sys_id_drive(output),
                lambda log: None,
                self,
            ),
        )

        # The SysId routine to test
        self._sys_id_routine_to_apply = self._sys_id_routine_translation

        if utils.is_simulation():
            self._start_sim_thread()

    @staticmethod
    def _log_sys_id_state(name: str, state: State) -> None:
        SignalLogger.write_string(name, SysIdRoutineLog.stateEnumToString(state))

    def _rotation_sys_id_drive(self, output: units.volt) -> None:
        # output is actually radians per second, but SysId only supports "volts"
        self.set_control(self._rotation_characterization.with_rotational_rate(output))
        # also log the requested output for SysId
        SignalLogger.write_double("Rotational_Rate", output)

    def apply_request(
        self, request_supplier: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request_supplier: Function returning the request to apply
        :returns: Command to run
        """
        return self.run(lambda: self.set_control(request_supplier()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by ``self._sys_id_routine_to_apply``.

        :param direction: Direction of the SysId Quasistatic test
        :returns: Command to run
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by ``self._sys_id_routine_to_apply``.

        :param direction: Direction of the SysId Dynamic test
        :returns: Command to run
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self) -> None:
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def _start_sim_thread(self) -> None:
        self._last_sim_time = utils.get_current_time_seconds()

        # Run simulation at a faster rate so PID gains behave more reasonably
        def _sim_periodic() -> None:
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

        atexit.register(self._sim_notifier.stop)

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: units.second,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
        while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        ``set_vision_measurement_std_devs()`` or this method.

        :param vision_robot_pose:            The pose of the robot as measured by the vision camera.
        :param timestamp:                    The timestamp of the vision measurement in seconds.
        :param vision_measurement_std_devs:  Standard deviations of the vision pose measurement
                                             in the form [x, y, theta]ᵀ, with units in meters and radians.
        """
        if vision_measurement_std_devs is not None:
            super().add_vision_measurement(
                vision_robot_pose,
                utils.fpga_to_current_time(timestamp),
                vision_measurement_std_devs,
            )
        else:
            super().add_vision_measurement(
                vision_robot_pose, utils.fpga_to_current_time(timestamp)
            )
