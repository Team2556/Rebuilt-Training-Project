import math

from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d


class Rotation2dObjectBuilder:

    def __init__(self) -> None:
        self._angle: units.radians = 0.0

    def with_angle(self, angle: units.radians | Translation2d) -> "Rotation2dObjectBuilder":
        self._angle = angle.angle().radians() if isinstance(angle, Translation2d) else angle
        return self

    def with_xy(self, x: units.meters, y: units.meters) -> "Rotation2dObjectBuilder":
        norm = math.hypot(x, y)

        if not math.isnan(norm) and norm > 1e-6:
            self._angle = math.atan2(x, y)
        else:
            self._angle = 0.0

        return self

    def build(self) -> Rotation2d:
        return Rotation2d(self._angle)
