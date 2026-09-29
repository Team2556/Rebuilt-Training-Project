from enum import Enum, auto

from .axis3d import Axis3d


class RotationAxis3d(Enum):
    ROLL = auto()
    PITCH = auto()
    YAW = auto()

    def to_axis3d(self) -> Axis3d:
        match self:
            case RotationAxis3d.ROLL:
                return Axis3d.ROLL
            case RotationAxis3d.PITCH:
                return Axis3d.PITCH
            case RotationAxis3d.YAW:
                return Axis3d.YAW
