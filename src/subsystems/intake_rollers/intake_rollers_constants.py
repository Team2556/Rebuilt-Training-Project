from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from wpilib import RobotBase

class IntakeRollerConstants:
    _CONFIG = TalonFXConfiguration()
    
    if RobotBase.isSimulation():
        _CONFIG.current_limits.stator_current_limit_enable = False
        _CONFIG.current_limits.supply_current_limit_enable = False

    TARGET_SPEED = -0.4