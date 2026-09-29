from wpimath import units
from wpimath.geometry import Translation3d

from ..axis3d.translation_axis3d import TranslationAxis3d


class Translation3dObjectBuilder:

    def __init__(
        self,
        x: units.meters = 0.0,
        y: units.meters = 0.0,
        z: units.meters = 0.0,
    ) -> None:
        self._x = x
        self._y = y
        self._z = z

    @classmethod
    def from_translation(cls, base: Translation3d) -> "Translation3dObjectBuilder":
        return cls(base.X(), base.Y(), base.Z())

    def with_axis(self, axis: TranslationAxis3d, distance: units.meters) -> "Translation3dObjectBuilder":
        match axis:
            case TranslationAxis3d.X:
                self._x = distance
            case TranslationAxis3d.Y:
                self._y = distance
            case TranslationAxis3d.Z:
                self._z = distance
        return self

    def with_x(self, x: units.meters) -> "Translation3dObjectBuilder":
        self._x = x
        return self

    def with_y(self, y: units.meters) -> "Translation3dObjectBuilder":
        self._y = y
        return self

    def with_z(self, z: units.meters) -> "Translation3dObjectBuilder":
        self._z = z
        return self

    def build(self) -> Translation3d:
        return Translation3d(self._x, self._y, self._z)
