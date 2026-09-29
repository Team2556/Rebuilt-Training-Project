from enum import Enum, auto


class Axis3d(Enum):
    X = auto()
    Y = auto()
    Z = auto()
    ROLL = auto()
    PITCH = auto()
    YAW = auto()

    def is_rotation_axis(self) -> bool:
        match self:
            case Axis3d.ROLL | Axis3d.PITCH | Axis3d.YAW:
                return True
            case _:
                return False

    def is_translation_axis(self) -> bool:
        return not self.is_rotation_axis()
