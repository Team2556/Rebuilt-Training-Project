from dataclasses import dataclass

from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d

from ..util.math_helpers import MathHelpers


@dataclass(frozen=True)
class PoseErrorTolerance:
    linear_error_tolerance: units.meters
    angular_error_tolerance: units.radians

    @classmethod
    def from_rotation(
        cls, linear_error_tolerance: units.meters, angular_error_tolerance: Rotation2d
    ) -> "PoseErrorTolerance":
        return cls(linear_error_tolerance, angular_error_tolerance.radians())

    def at_pose(self, expected: Pose2d, actual: Pose2d) -> bool:
        linear_error = expected.translation().distance(actual.translation())

        return MathHelpers.is_near(
            0, linear_error, self.linear_error_tolerance
        ) and MathHelpers.is_near(
            expected.rotation().degrees(),
            actual.rotation().degrees(),
            units.radiansToDegrees(self.angular_error_tolerance),
            -180,
            180,
        )
