import traceback

import phoenix6.unmanaged
from phoenix6 import SignalLogger
from wpilib import LiveWindow, TimedRobot, DriverStation, SmartDashboard, Timer, reportError
from wpilib.simulation import DriverStationSim

from hal import AllianceStationID

from commands2 import Command, CommandScheduler, cmd

from src.robot_constants import RobotConstants
from src.controlboard.controlboard import Controlboard

from src.subsystems.superstructure.superstructure import Superstructure
from src.subsystems.intake_rollers.intake_rollers import IntakeRollers


class RobotContainer(TimedRobot):
    disabled_counter: int
    # battery_checker: BatteryCheckerIO

    reset_pose_for_auto: bool = False

    def __init__(self, period: float = 0.02) -> None:
        super().__init__(period)

        phoenix6.unmanaged.set_phoenix_diagnostics_start_time(-1)
        SignalLogger.enable_auto_logging(False)

        self.disabled_counter = 0
        self.autonomous_command: Command | None = None

    def update_alliance(self) -> None:
        if self.isSimulation():
            RobotConstants.is_red_alliance = DriverStationSim.getAllianceStationId() in [
                AllianceStationID.kRed1,
                AllianceStationID.kRed2,
                AllianceStationID.kRed3
            ]
        else:
            driver_station_alliance = DriverStation.getAlliance()
            if driver_station_alliance is not None:
                RobotConstants.is_red_alliance = driver_station_alliance == DriverStation.Alliance.kRed
            else:
                SmartDashboard.putNumber("Last unable to set alliance", Timer.getFPGATimestamp())

    def robotInit(self) -> None:
        Controlboard.configure_bindings()

        for sendable in (
            Superstructure.m_instance,
            IntakeRollers.m_instance,
        ):
            SmartDashboard.putData(sendable)

        self.update_alliance()

    def robotPeriodic(self) -> None:
        
        try:
            CommandScheduler.getInstance().run()
        except Exception as exception:
            reportError(f"Robot loop error: {exception!r}\n{traceback.format_exc()}", False)
            SmartDashboard.putString("Error/Last Loop Error/Last Error Message", repr(exception))
            SmartDashboard.putNumber("Error/Last Loop Error/Last Error Timestamp", Timer.getFPGATimestamp())

    def disabledInit(self) -> None:
        self.disabled_counter = 0

    def disabledPeriodic(self) -> None:
        self.update_alliance()

        self.disabled_counter += 1
        if self.isReal():
            self.battery_checker.periodic()

    def disabledExit(self) -> None: pass

    def autonomousInit(self) -> None:
        self.update_alliance()

        self.autonomous_command = cmd.runOnce(lambda: print("Autonomous mode enabled")).ignoringDisable(True)
        CommandScheduler.getInstance().schedule(self.autonomous_command)

    def autonomousPeriodic(self) -> None: pass
    def autonomousExit(self) -> None: pass

    def teleopInit(self) -> None:
        self.update_alliance()
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

    def teleopPeriodic(self) -> None: pass
    def teleopExit(self) -> None: pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None: pass
    def testExit(self) -> None: pass
    def _simulationInit(self) -> None: pass
    def _simulationPeriodic(self) -> None: pass
