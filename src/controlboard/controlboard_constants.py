from commands2.button.commandxboxcontroller import CommandXboxController

from lib.util.unit_types import *

class ControlboardConstants:
    m_driver_controller = CommandXboxController(0)
    # m_operator_controller = CommandXboxController(1)
    
    INTAKE_RUMBLE_TIME = Seconds.of(0.2)
    
    # Is 0.05 in original code; increased it because my controller is like really bad
    STICK_DEADBAND = 0.15
