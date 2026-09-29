import math
from enum import Enum, auto
from typing import Callable

from commands2 import Command, InterruptionBehavior, Subsystem, cmd
from phoenix6.swerve import requests
from wpilib import Timer
from wpimath.filter import Debouncer
from wpiutil import SendableBuilder

# from lib.bases.motor_subsystem import MotorSubsystem
# from lib.io.motor_io import Setpoint

from src.subsystems.drive.drive import Drive
from src.subsystems.drive.drive_constants import DriveConstants
from src.subsystems.superstructure.superstructure_constants import SuperstructureConstants

from src.subsystems.intake_rollers.intake_rollers import IntakeRollers


class State(Enum):
    # TODO: add more states (example: intaking, firing, all that stuff)
    IDLE = auto()
    INTAKING = auto()
    EXTAKING = auto()

class Superstructure(Subsystem):
    """
    Owns the whole-robot context and builds every command that coordinates more than one
    mechanism. Commands read the flags here (hold fire, wiggle) instead of reaching for the
    controllers themselves, so any other command can change how a shot behaves by setting a
    flag rather than by editing the shot.
    The default commands and controlboard commands are made up of commands defined in this
    superstructure.
    """

    m_instance: "Superstructure"

    State = State

    def __init__(self) -> None:
        super().__init__()
        self.setName("Superstructure")

        self._state_requests: list[list[State]] = []

        # TODO: Put flags here
        # example: self.firing = False

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        
        builder.addStringProperty("State", lambda: self.state.name, lambda v: None)
        
        # TODO: add more Sendable properties (at least all of your flags)

    @property
    def state(self) -> State:
        """The state of the most recently started command still running, or IDLE."""
        if self._state_requests:
            return self._state_requests[-1][0]
        return State.IDLE

    def get_state(self) -> State:
        return self.state

    def state_command(self, state: State) -> Command:
        """Command that overrides the current state to whatever you want when it starts, and
        sets the state back to State.IDLE or the last run state_command's target state when it finishes"""
        request: list[list[State]] = [[state]]

        def start() -> None:
            request[0] = [state]
            self._state_requests.append(request[0])

        def end() -> None:
            self._state_requests = [r for r in self._state_requests if r is not request[0]]

        return cmd.startEnd(start, end)

    # TODO: create helper functions here

    def configure_default_commands(self) -> None:
        Drive.m_instance.setDefaultCommand(
            Drive.m_instance.follow_swerve_request_command(
                DriveConstants.teleop_request, DriveConstants.teleop_request_updater
            ).withName("Teleop Drive")
        )
    
    # TODO: create commands here
    
    def intake_command(self) -> Command:
        return cmd.parallel(
            cmd.runEnd(
                IntakeRollers.m_instance.intake, 
                IntakeRollers.m_instance.stop_roller, 
                IntakeRollers.m_instance
            ),
            self.state_command(State.INTAKING)
        )

    def extake_command(self) -> Command:
        return cmd.parallel(
            cmd.runEnd(
                IntakeRollers.m_instance.extake, 
                IntakeRollers.m_instance.stop_roller, 
                IntakeRollers.m_instance
            ),
            self.state_command(State.EXTAKING)
        )

Superstructure.m_instance = Superstructure()
