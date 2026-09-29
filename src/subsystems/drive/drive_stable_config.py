import math

from wpimath.units import meters_per_second, radians_per_second, seconds
from wpimath.filter import Debouncer
from wpimath.kinematics import ChassisSpeeds

from src.subsystems.drive.drive import Drive

# NOTE: This class isn't referenced so I have no idea why it exists.
# Probably because I forgot what a "debounce" is
class DriveStableConfig:
    def __init__(self, max_linear_velocity: meters_per_second, max_angular_velocity: radians_per_second, threshold_check_debounce: seconds) -> None:
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.threshold_check_debounce = threshold_check_debounce

        self.threshold_check_debouncer = Debouncer(threshold_check_debounce, Debouncer.DebounceType.kBoth)

    def is_debounce_stable(self, speeds: ChassisSpeeds) -> bool:
        return self.threshold_check_debouncer.calculate(
            math.hypot(speeds.vx, speeds.vy) <= self.max_linear_velocity
            and abs(speeds.omega) <= self.max_angular_velocity
        )
    
    def is_stable(self) -> bool:
        return self.is_debounce_stable(Drive.m_instance.get_state().speeds)
