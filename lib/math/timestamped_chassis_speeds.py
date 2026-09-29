from wpilib import Timer
from wpimath.kinematics import ChassisSpeeds


class TimestampedChassisSpeeds(ChassisSpeeds):

    def __init__(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        timestamp_seconds: float | None = None,
    ) -> None:
        super().__init__(vx, vy, omega)
        self.timestamp_seconds = (
            Timer.getFPGATimestamp() if timestamp_seconds is None else timestamp_seconds
        )

    @classmethod
    def from_speeds(
        cls, speeds: ChassisSpeeds, timestamp_seconds: float | None = None
    ) -> "TimestampedChassisSpeeds":
        if timestamp_seconds is None and isinstance(speeds, TimestampedChassisSpeeds):
            timestamp_seconds = speeds.timestamp_seconds
        return cls(speeds.vx, speeds.vy, speeds.omega, timestamp_seconds)

    @classmethod
    def from_timestamp(cls, timestamp_seconds: float) -> "TimestampedChassisSpeeds":
        return cls(0, 0, 0, timestamp_seconds)

    def timestamp_difference(self, other: "TimestampedChassisSpeeds") -> float:
        return self.timestamp_seconds - other.timestamp_seconds
