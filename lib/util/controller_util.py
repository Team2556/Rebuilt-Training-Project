from typing import Any, Callable, TypeVar

import commands2
from commands2 import Command
from commands2.button import Trigger
from wpilib import SmartDashboard, Timer
from wpimath import units

from ..bases.motor_subsystem import MotorSubsystem
from ..io.motor_io import MotorIO, Mode, Setpoint

IO = TypeVar("IO", bound=MotorIO[Any])


class ControllerUtil:

    @staticmethod
    def create_from_mode(mode: Mode, base_units: float) -> Setpoint:
        match mode:
            case Mode.IDLE:
                return Setpoint.with_coast_setpoint()
            case Mode.VOLTAGE:
                return Setpoint.with_voltage_setpoint(base_units)
            case Mode.MOTIONMAGIC:
                return Setpoint.with_motion_magic_setpoint(base_units)
            case Mode.VELOCITY:
                return Setpoint.with_velocity_setpoint(base_units)
            case Mode.DUTY_CYCLE:
                return Setpoint.with_duty_cycle_setpoint(base_units)
            case Mode.POSITIONPID:
                return Setpoint.with_position_setpoint(base_units)

    @staticmethod
    def bind_jog(
        subsystem: MotorSubsystem[IO],
        interval_base_units: float,
        initial_value: float,
        max_value_base_units: float,
        min_value_base_units: float | None,
        setpoint_mode: Mode,
        start_setpoint: Setpoint,
        continous_delay: units.seconds,
        *triggers: Trigger,
    ) -> None:
        """
        :param subsystem: subsystem to bind jog to
        :param interval_base_units: intervals between each jog input
        :param initial_value: subsystem's start value of the jogging action
        :param max_value_base_units: max value to apply jogging action to
        :param min_value_base_units: min value to apploy jogging action to. Pass None to use
            the negative of max_value_base_units as a symmetric absolute bound.
        :param setpoint_mode: mode to apply jogging action to
        :param start_setpoint: subsystem's initial setpoint
        :param continous_delay: delay between each jog input when holding down the trigger
        :param triggers: binded triggers ([0]: jog up (REQUIRED FOR AN EFFECT TO HAPPEN), [1]: jog down, [2]: set to start)
        """
        if min_value_base_units is None:
            ensured_absolute = abs(max_value_base_units)
            max_value_base_units = ensured_absolute
            min_value_base_units = -ensured_absolute

        if min_value_base_units > max_value_base_units:
            raise ValueError("Min output can't be greater than the max output")

        subsystem.apply_setpoint(
            start_setpoint
        )  # apply the setpoint in case the subsytem currently has no setpoint

        def setpoint_offseter(offset: float) -> None:
            followed = subsystem.get_setpoint()
            if followed is start_setpoint:
                next = ControllerUtil.create_from_mode(setpoint_mode, initial_value + offset)
            else:
                if followed.mode is not setpoint_mode:
                    SmartDashboard.putNumber(
                        "Jogging/Last Unable to Jog Seconds timestamp", Timer.getFPGATimestamp()
                    )
                    return
                if followed.base_units + offset == initial_value:
                    next = start_setpoint
                else:
                    next_base_units = followed.base_units + offset
                    next = ControllerUtil.create_from_mode(
                        setpoint_mode,
                        max(min_value_base_units, min(next_base_units, max_value_base_units)),
                    )
            subsystem.apply_setpoint(next)

        def command_getter(interval: float) -> Command:
            return (
                commands2.cmd.runOnce(lambda: setpoint_offseter(interval))
                .andThen(commands2.cmd.waitSeconds(continous_delay))
                .repeatedly()
            )

        if len(triggers) >= 3:
            triggers[2].onTrue(subsystem.setpoint_command(start_setpoint))
        if len(triggers) >= 2:
            triggers[1].whileTrue(command_getter(-interval_base_units))
        if len(triggers) >= 1:
            triggers[0].whileTrue(command_getter(interval_base_units))
