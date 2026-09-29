from wpimath import units
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from ..axis3d.rotation_axis3d import RotationAxis3d
from ..axis3d.translation_axis3d import TranslationAxis3d


class Pose3dObjectBuilder:

    def __init__(self) -> None:
        self._x: units.meters = 0.0
        self._y: units.meters = 0.0
        self._z: units.meters = 0.0

        self._roll: units.radians = 0.0
        self._pitch: units.radians = 0.0
        self._yaw: units.radians = 0.0

    @classmethod
    def from_pose(cls, pose: Pose3d) -> "Pose3dObjectBuilder":
        builder = cls()
        builder._x = pose.X()
        builder._y = pose.Y()
        builder._z = pose.Z()
        builder._roll = pose.rotation().X()
        builder._pitch = pose.rotation().Y()
        builder._yaw = pose.rotation().Z()
        return builder

    def with_x(self, x: units.meters) -> "Pose3dObjectBuilder":
        self._x = x
        return self

    def with_y(self, y: units.meters) -> "Pose3dObjectBuilder":
        self._y = y
        return self

    def with_z(self, z: units.meters) -> "Pose3dObjectBuilder":
        self._z = z
        return self

    def with_translation(self, translation: Translation3d) -> "Pose3dObjectBuilder":
        self._x = translation.X()
        self._y = translation.Y()
        self._z = translation.Z()
        return self

    def with_roll(self, roll: units.radians) -> "Pose3dObjectBuilder":
        self._roll = roll
        return self

    def with_pitch(self, pitch: units.radians) -> "Pose3dObjectBuilder":
        self._pitch = pitch
        return self

    def with_yaw(self, yaw: units.radians) -> "Pose3dObjectBuilder":
        self._yaw = yaw
        return self

    def with_rotation(self, rotation: Rotation3d) -> "Pose3dObjectBuilder":
        self._roll = rotation.X()
        self._pitch = rotation.Y()
        self._yaw = rotation.Z()
        return self

    def times(self, scalar: float) -> "Pose3dObjectBuilder":
        return (
            self.with_x(self._x * scalar)
            .with_y(self._y * scalar)
            .with_z(self._z * scalar)
            .with_roll(self._roll * scalar)
            .with_pitch(self._pitch * scalar)
            .with_yaw(self._yaw * scalar)
        )

    def unary_minus(self) -> "Pose3dObjectBuilder":
        return self.times(-1.0)

    def with_offset(
        self,
        axis: TranslationAxis3d | RotationAxis3d,
        value: units.meters | units.radians,
    ) -> "Pose3dObjectBuilder":
        match axis:
            case TranslationAxis3d.X:
                self._x = self._x + value
            case TranslationAxis3d.Y:
                self._y = self._y + value
            case TranslationAxis3d.Z:
                self._z = self._z + value
            case RotationAxis3d.ROLL:
                self._roll = self._roll + value
            case RotationAxis3d.PITCH:
                self._pitch = self._pitch + value
            case RotationAxis3d.YAW:
                self._yaw = self._yaw + value
        return self

    def with_offset_x(self, distance: units.meters) -> "Pose3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.X, distance)

    def with_offset_y(self, distance: units.meters) -> "Pose3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.Y, distance)

    def with_offset_z(self, distance: units.meters) -> "Pose3dObjectBuilder":
        return self.with_offset(TranslationAxis3d.Z, distance)

    def with_offset_roll(self, angle: units.radians) -> "Pose3dObjectBuilder":
        return self.with_offset(RotationAxis3d.ROLL, angle)

    def with_offset_pitch(self, angle: units.radians) -> "Pose3dObjectBuilder":
        return self.with_offset(RotationAxis3d.PITCH, angle)

    def with_offset_yaw(self, angle: units.radians) -> "Pose3dObjectBuilder":
        return self.with_offset(RotationAxis3d.YAW, angle)

    def build(self) -> Pose3d:
        return Pose3d(self._x, self._y, self._z, Rotation3d(self._roll, self._pitch, self._yaw))
