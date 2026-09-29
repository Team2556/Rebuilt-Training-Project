from commands2 import cmd

from src.controlboard.controlboard_constants import ControlboardConstants
# from src.subsystems.camera.camera import Cameras
from src.subsystems.drive.drive import Drive
from src.subsystems.drive.drive_constants import DriveConstants
from src.subsystems.superstructure.superstructure import Superstructure


class Controlboard:
    m_instance: "Controlboard"

    driver = ControlboardConstants.m_driver_controller
    # operator = ControlboardConstants.m_operator_controller

    @staticmethod
    def configure_bindings() -> None:
        
        Superstructure.m_instance.configure_default_commands()

        Controlboard.driver_controls()
        Controlboard.operator_controls()

    @staticmethod
    def driver_controls() -> None:
        driver = Controlboard.driver
        
        # Example of a drive control command (that resets the field-centric view)
        # This should be the only button command that doesn't use superstructure
        driver.leftStick().onTrue(
            cmd.runOnce(
                lambda: Drive.m_instance.get_generated_drive().seed_field_centric(),
                Drive.m_instance,
            ).ignoringDisable(True)
        )

        # TODO: create more driver control commands
        
        driver.leftBumper().whileTrue(
            Superstructure.m_instance.intake_command()
        )
        
        driver.rightBumper().whileTrue(
            Superstructure.m_instance.extake_command()
        )

    @staticmethod
    def operator_controls() -> None:
        # TODO: create more operator control commands (if you want a second controller, but I would recommend)
        # not having one because it's not needed and we're kinda low on controllers
        pass

Controlboard.m_instance = Controlboard()
