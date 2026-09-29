from wpimath import units
from wpimath.geometry import Rotation3d

from ..axis3d.rotation_axis3d import RotationAxis3d


class Rotation3dObjectBuilder:

    def __init__(self) -> None:
        self._roll: units.radians = 0.0
        self._pitch: units.radians = 0.0
        self._yaw: units.radians = 0.0

    def with_roll(self, roll: units.radians) -> "Rotation3dObjectBuilder":
        self._roll = roll
        return self

    def with_axis(self, axis: RotationAxis3d, angle: units.radians) -> "Rotation3dObjectBuilder":
        match axis:
            case RotationAxis3d.ROLL:
                self._roll = angle
            case RotationAxis3d.PITCH:
                self._pitch = angle
            case RotationAxis3d.YAW:
                self._yaw = angle
        return self

    def with_pitch(self, pitch: units.radians) -> "Rotation3dObjectBuilder":
        self._pitch = pitch
        return self

    def with_yaw(self, yaw: units.radians) -> "Rotation3dObjectBuilder":
        self._yaw = yaw
        return self

    def build(self) -> Rotation3d:
        return Rotation3d(self._roll, self._pitch, self._yaw)
