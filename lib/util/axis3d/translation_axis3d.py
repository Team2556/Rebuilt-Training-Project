from enum import Enum, auto

from .axis3d import Axis3d


class TranslationAxis3d(Enum):
    X = auto()
    Y = auto()
    Z = auto()

    def to_axis3d(self) -> Axis3d:
        match self:
            case TranslationAxis3d.X:
                return Axis3d.X
            case TranslationAxis3d.Y:
                return Axis3d.Y
            case TranslationAxis3d.Z:
                return Axis3d.Z
