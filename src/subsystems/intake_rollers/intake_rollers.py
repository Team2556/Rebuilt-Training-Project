from src.subsystems.intake_rollers.intake_rollers_constants import IntakeRollerConstants
from src.ports import Ports
from src.robot_constants import RobotConstants

from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import NeutralOut, DutyCycleOut

class IntakeRollers(Subsystem):
    def __init__(self):
        super().__init__()

        self.roller_motor = TalonFX(Ports.INTAKE_ROLLER.value[0], Ports.INTAKE_ROLLER.value[1])

        self.roller_cfg = IntakeRollerConstants._CONFIG
        self.roller_motor.configurator.apply(self.roller_cfg)
        
        self.duty_cycle_control = DutyCycleOut(output=0)
        self.coast = NeutralOut()

    def initSendable(self, builder):
        super().initSendable(builder)
        
        builder.addFloatProperty("Intake Speed", self.roller_motor.get, lambda: None)
        
        builder.addFloatProperty(
            "Target Intake Speed",
            lambda: IntakeRollerConstants.TARGET_SPEED,
            lambda value: setattr(IntakeRollerConstants, "TARGET_SPEED", value)
        )

    def intake(self) -> None:
        self.roller_motor.set_control(self.duty_cycle_control.with_output(IntakeRollerConstants.TARGET_SPEED))
    
    def extake(self) -> None:
        self.roller_motor.set_control(self.duty_cycle_control.with_output(-IntakeRollerConstants.TARGET_SPEED))

    def stop_roller(self) -> None:
        self.roller_motor.set_control(self.coast)
    
    m_instance: "IntakeRollers"

IntakeRollers.m_instance = IntakeRollers()
