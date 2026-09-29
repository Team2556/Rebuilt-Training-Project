import math
from typing import Any, overload

from wpimath import inputModulus, units
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from wpimath.kinematics import ChassisSpeeds

from .axis3d.translation_axis3d import TranslationAxis3d


class MathHelpers:
    K_POSE2D_ZERO = Pose2d()
    PI: units.radians = math.pi
    PI_DIV_2: units.radians = PI / 2.0

    K_ROTATION2D_ZERO = Rotation2d()
    K_ROTATION2D_PI = Rotation2d.fromDegrees(180.0)

    K_TRANSLATION2D_ZERO = Translation2d()

    K_TRANSFORM2D_ZERO = Transform2d()

    @staticmethod
    def pose2d_from_rotation(rotation: Rotation2d) -> Pose2d:
        return Pose2d(MathHelpers.K_TRANSLATION2D_ZERO, rotation)

    @staticmethod
    def pose2d_from_translation(translation: Translation2d) -> Pose2d:
        return Pose2d(translation, MathHelpers.K_ROTATION2D_ZERO)

    @staticmethod
    def transform2d_from_rotation(rotation: Rotation2d) -> Transform2d:
        return Transform2d(MathHelpers.K_TRANSLATION2D_ZERO, rotation)

    @staticmethod
    def transform2d_from_translation(translation: Translation2d) -> Transform2d:
        return Transform2d(translation, MathHelpers.K_ROTATION2D_ZERO)

    @overload
    @staticmethod
    def pose_from_transform(transform: Transform3d) -> Pose3d: ...

    @overload
    @staticmethod
    def pose_from_transform(transform: Transform2d) -> Pose2d: ...

    @staticmethod
    def pose_from_transform(transform: Transform2d | Transform3d) -> Pose2d | Pose3d:
        if isinstance(transform, Transform3d):
            return Pose3d(transform.X(), transform.Y(), transform.Z(), transform.rotation())
        return Pose2d(transform.X(), transform.Y(), transform.rotation())

    @overload
    @staticmethod
    def transform_from_pose(pose: Pose3d) -> Transform3d: ...

    @overload
    @staticmethod
    def transform_from_pose(pose: Pose2d) -> Transform2d: ...

    @staticmethod
    def transform_from_pose(pose: Pose2d | Pose3d) -> Transform2d | Transform3d:
        if isinstance(pose, Pose3d):
            return Transform3d(pose.translation(), pose.rotation())
        return Transform2d(pose.X(), pose.Y(), pose.rotation())

    @staticmethod
    def get_look_ahead(
        speeds: ChassisSpeeds, current_pose: Pose2d, look_ahead_time: units.seconds
    ) -> Pose2d:
        return current_pose.transformBy(
            Transform2d(speeds.vx, speeds.vy, Rotation2d(speeds.omega)) * look_ahead_time
        )

    @staticmethod
    def clamp(input: float, high: float, low: float) -> float:
        return max(min(input, high), low)

    @staticmethod
    def abs(value: units.radians | Rotation2d) -> units.radians | Rotation2d:
        if isinstance(value, Rotation2d):
            return Rotation2d(abs(value.radians()))
        return abs(value)

    @staticmethod
    def sin(theta: units.radians) -> float:
        return math.sin(theta)

    @staticmethod
    def cos(theta: units.radians) -> float:
        return math.cos(theta)

    @staticmethod
    def tan(theta: units.radians) -> float:
        return math.tan(theta)

    @overload
    @staticmethod
    def hypot(translation: Translation2d, /) -> float: ...

    @overload
    @staticmethod
    def hypot(speed: ChassisSpeeds, /) -> float: ...

    @overload
    @staticmethod
    def hypot(x: float, y: float, /) -> float: ...

    @staticmethod
    def hypot(*args: Any) -> float:
        match args:
            case (Translation2d() as translation,):
                return math.hypot(translation.X(), translation.Y())
            case (ChassisSpeeds() as speed,):
                return math.hypot(speed.vx, speed.vy)
            case (x, y):
                return math.hypot(x, y)
        raise TypeError("hypot expects a Translation2d, a ChassisSpeeds, or an x and y pair")

    @overload
    @staticmethod
    def angle_modulus(angle: Rotation2d) -> Rotation2d: ...

    @overload
    @staticmethod
    def angle_modulus(angle: units.radians) -> units.radians: ...

    @staticmethod
    def angle_modulus(angle: units.radians | Rotation2d) -> units.radians | Rotation2d:
        if isinstance(angle, Rotation2d):
            return Rotation2d(MathHelpers.angle_modulus(angle.radians()))
        return inputModulus(angle, -math.pi, math.pi)

    @staticmethod
    def interpolate(
        start_value: Rotation2d, end_value: Rotation2d, t: float
    ) -> Rotation2d:
        """
        Linearly interpolates between two rotations.

        :param start_value:  rotation at t = 0
        :param end_value:  rotation at t = 1
        :param t:  fraction along the interpolation, clamped to [0, 1]
        :returns: the interpolated rotation
        """
        return start_value + (end_value - start_value) * MathHelpers.clamp(t, 1.0, 0.0)

    @staticmethod
    def unary_minus(pose: Pose3d) -> Pose3d:
        return pose * -1.0

    @overload
    @staticmethod
    def add_to_translation3d(
        target: Transform3d, axis: TranslationAxis3d, offset: units.meters
    ) -> Transform3d: ...

    @overload
    @staticmethod
    def add_to_translation3d(
        target: Translation3d, axis: TranslationAxis3d, offset: units.meters
    ) -> Translation3d: ...

    @staticmethod
    def add_to_translation3d(
        target: Translation3d | Transform3d, axis: TranslationAxis3d, offset: units.meters
    ) -> Translation3d | Transform3d:
        if isinstance(target, Transform3d):
            return Transform3d(
                MathHelpers.add_to_translation3d(target.translation(), axis, offset), target.rotation()
            )
        match axis:
            case TranslationAxis3d.X:
                return target + Translation3d(offset, 0.0, 0.0)
            case TranslationAxis3d.Y:
                return target + Translation3d(0.0, offset, 0.0)
            case TranslationAxis3d.Z:
                return target + Translation3d(0.0, 0.0, offset)

    @staticmethod
    def transform3d(pose: Pose2d) -> Transform3d:
        return Transform3d(pose.X(), pose.Y(), 0.0, Rotation3d(0.0, 0.0, pose.rotation().radians()))

    @staticmethod
    def get_2d_cords_from_circle_angle(
        radius: units.meters, angle: Rotation2d | units.radians
    ) -> Translation2d:
        rotation = angle if isinstance(angle, Rotation2d) else Rotation2d(angle)
        return Translation2d(radius * rotation.cos(), radius * rotation.sin())

    @staticmethod
    def calculate_triangle_angle_from_sides(
        opposing_face: float, face1: float, face2: float
    ) -> units.radians:
        a = face1
        b = face2
        c = opposing_face
        return math.acos(((a * a) + (b * b) - (c * c)) / (2.0 * a * b))

    @staticmethod
    def clamp_to_zero(value: float) -> float:
        return value if value >= 0.0 else 0.0

    @staticmethod
    def is_near(
        expected: float,
        actual: float,
        tolerance: float,
        min: float | None = None,
        max: float | None = None,
    ) -> bool:
        if min is None or max is None:
            return abs(expected - actual) < tolerance
        error_bound = (max - min) / 2.0
        error = inputModulus(expected - actual, -error_bound, error_bound)
        return abs(error) < tolerance
