from .axis3d import Axis3d
from .rotation_axis3d import RotationAxis3d
from .translation_axis3d import TranslationAxis3d


class AxisUtil:

    @staticmethod
    def get_translation_axis_from_rotation_axis(axis: RotationAxis3d) -> TranslationAxis3d:
        match axis:
            case RotationAxis3d.ROLL:
                return TranslationAxis3d.X
            case RotationAxis3d.PITCH:
                return TranslationAxis3d.Y
            case RotationAxis3d.YAW:
                return TranslationAxis3d.Z

    @staticmethod
    def try_to_get_translation(axis: Axis3d) -> TranslationAxis3d | None:
        match axis:
            case Axis3d.X:
                return TranslationAxis3d.X
            case Axis3d.Y:
                return TranslationAxis3d.Y
            case Axis3d.Z:
                return TranslationAxis3d.Z
            case _:
                return None

    @staticmethod
    def try_to_get_rotation(axis: Axis3d) -> RotationAxis3d | None:
        match axis:
            case Axis3d.ROLL:
                return RotationAxis3d.ROLL
            case Axis3d.PITCH:
                return RotationAxis3d.PITCH
            case Axis3d.YAW:
                return RotationAxis3d.YAW
            case _:
                return None
