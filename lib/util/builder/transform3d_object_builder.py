from wpimath import units
from wpimath.geometry import Rotation3d, Transform3d, Translation3d

from ..axis3d.rotation_axis3d import RotationAxis3d
from ..axis3d.translation_axis3d import TranslationAxis3d


class Transform3dObjectBuilder:

    def __init__(self, distance: units.meters = 0.0, angle: units.radians = 0.0) -> None:
        self._x = distance
        self._y = distance
        self._z = distance

        self._roll = angle
        self._pitch = angle
        self._yaw = angle

    @classmethod
    def from_transform(cls, transform: Transform3d) -> "Transform3dObjectBuilder":
        builder = cls()
        builder._x = transform.X()
        builder._y = transform.Y()
        builder._z = transform.Z()
        builder._roll = transform.rotation().X()
        builder._pitch = transform.rotation().Y()
        builder._yaw = transform.rotation().Z()
        return builder

    def with_x(self, x: units.meters) -> "Transform3dObjectBuilder":
        self._x = x
        return self

    def with_y(self, y: units.meters) -> "Transform3dObjectBuilder":
        self._y = y
        return self

    def with_z(self, z: units.meters) -> "Transform3dObjectBuilder":
        self._z = z
        return self

    def with_offset(self, axis: TranslationAxis3d, distance: units.meters) -> "Transform3dObjectBuilder":
        match axis:
            case TranslationAxis3d.X:
                self._x = self._x + distance
            case TranslationAxis3d.Y:
                self._y = self._y + distance
            case TranslationAxis3d.Z:
                self._z = self._z + distance
        return self

    def with_offset_x(self, distance: units.meters) -> "Transform3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.X, distance)

    def with_offset_y(self, distance: units.meters) -> "Transform3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.Y, distance)

    def with_offset_z(self, distance: units.meters) -> "Transform3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.Z, distance)

    def with_rotation_axis(self, axis: RotationAxis3d, angle: units.radians) -> "Transform3dObjectBuilder":
        match axis:
            case RotationAxis3d.ROLL:
                self._roll = angle
            case RotationAxis3d.PITCH:
                self._pitch = angle
            case RotationAxis3d.YAW:
                self._yaw = angle
        return self

    def with_translation(self, translation: Translation3d) -> "Transform3dObjectBuilder":
        return self.with_x(translation.X()).with_y(translation.Y()).with_z(translation.Z())

    def with_roll(self, roll: units.radians) -> "Transform3dObjectBuilder":
        self._roll = roll
        return self

    def with_pitch(self, pitch: units.radians) -> "Transform3dObjectBuilder":
        self._pitch = pitch
        return self

    def with_yaw(self, yaw: units.radians) -> "Transform3dObjectBuilder":
        self._yaw = yaw
        return self

    def with_rotation(self, rotation: Rotation3d) -> "Transform3dObjectBuilder":
        return self.with_roll(rotation.X()).with_pitch(rotation.Y()).with_yaw(rotation.Z())

    def build(self) -> Transform3d:
        return Transform3d(self._x, self._y, self._z, Rotation3d(self._roll, self._pitch, self._yaw))
