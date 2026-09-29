from enum import Enum

from phoenix6 import CANBus
from src.robot_constants import RobotConstants


class Ports(Enum):
    INTAKE_ROLLER = (16, RobotConstants.rio)    
    INTAKE_DEPLOY_MAIN = (17, RobotConstants.rio)
    INTAKE_DEPLOY_FOLLOWER = (18, RobotConstants.rio)
    SPINDEXER = (21, RobotConstants.rio)
    TRANSFER = (22, RobotConstants.rio)
    SHOOTER_MAIN = (27, RobotConstants.rio)
    SHOOTER_FOLLOWER = (26, RobotConstants.rio)
    SHOOTER_HOOD_CONTROL = (28, RobotConstants.rio)

    def __init__(self, id: int, bus: CANBus) -> None:
        self.id = id
        self.bus = bus
