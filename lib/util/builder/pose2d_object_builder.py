from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d


class Pose2dObjectBuilder:

    def __init__(
        self,
        x: units.meters = 0.0,
        y: units.meters = 0.0,
        rotation: Rotation2d | None = None,
    ) -> None:
        self._x = x
        self._y = y
        self._rotation = Rotation2d() if rotation is None else rotation

    @classmethod
    def from_pose(cls, base: Pose2d) -> "Pose2dObjectBuilder":
        return cls(base.X(), base.Y(), base.rotation())

    def with_x(self, x: units.meters) -> "Pose2dObjectBuilder":
        self._x = x
        return self

    def with_y(self, y: units.meters) -> "Pose2dObjectBuilder":
        self._y = y
        return self

    def with_rotation(self, rotation: Rotation2d | units.radians) -> "Pose2dObjectBuilder":
        self._rotation = rotation if isinstance(rotation, Rotation2d) else Rotation2d(rotation)
        return self

    def build(self) -> Pose2d:
        return Pose2d(self._x, self._y, self._rotation)
