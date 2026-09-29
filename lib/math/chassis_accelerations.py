import math

from wpimath.geometry import Rotation2d, Translation2d


class ChassisAccelerations:
    """
    Represents the chassis accelerations of a robot in 2D planar space
    with optional angular acceleration about the vertical axis.

    This class provides utilities for frame transformations (field <-> robot),
    as well as basic operations such as addition and scaling.
    """

    def __init__(self, ax: float = 0.0, ay: float = 0.0, ao: float = 0.0) -> None:
        """
        Constructor for full 2D linear and angular accelerations.

        :param ax: linear acceleration along X-axis
        :param ay: linear acceleration along Y-axis
        :param ao: angular acceleration around vertical axis
        """
        self.ax = ax
        """Linear acceleration along the X-axis (forward/backward)"""

        self.ay = ay
        """Linear acceleration along the Y-axis (sideways)"""

        self.alpha = ao
        """Angular acceleration around the vertical axis (rotation)"""

    def get_ax(self) -> float:
        """Getters"""
        return self.ax

    def get_ay(self) -> float:
        return self.ay

    def get_ao(self) -> float:
        return self.alpha

    def set_ax(self, ax: float) -> None:
        """Setters"""
        self.ax = ax

    def set_ay(self, ay: float) -> None:
        self.ay = ay

    def set_ao(self, ao: float) -> None:
        self.alpha = ao

    def get_norm(self) -> float:
        """
        Returns the planar magnitude (norm) of the linear acceleration.

        :returns: sqrt(ax^2 + ay^2)
        """
        return math.hypot(self.ax, self.ay)

    def to_robot_relative(self, robot_angle: Rotation2d) -> "ChassisAccelerations":
        """
        Rotates the acceleration from field-relative coordinates to robot-relative coordinates.

        :param robot_angle: the current robot heading (rotation of robot relative to field)

        :returns: a new ChassisAccelerations in robot-relative frame
        """
        rotated = Translation2d(self.ax, self.ay).rotateBy(-robot_angle)
        return ChassisAccelerations(rotated.X(), rotated.Y(), self.alpha)

    def to_field_relative(self, robot_angle: Rotation2d) -> "ChassisAccelerations":
        """
        Rotates the acceleration from robot-relative coordinates to field-relative coordinates.

        :param robot_angle: the current robot heading (rotation of robot relative to field)

        :returns: a new ChassisAccelerations in field-relative frame
        """
        rotated = Translation2d(self.ax, self.ay).rotateBy(robot_angle)
        return ChassisAccelerations(rotated.X(), rotated.Y(), self.alpha)

    def plus(self, other: "ChassisAccelerations") -> "ChassisAccelerations":
        """
        Adds another ChassisAccelerations to this one.

        :param other: the other ChassisAccelerations

        :returns: a new ChassisAccelerations representing the sum
        """
        return ChassisAccelerations(self.ax + other.ax, self.ay + other.ay, self.alpha + other.alpha)

    def times(self, scalar: float) -> "ChassisAccelerations":
        """
        Scales the accelerations by a constant factor.

        :param scalar: the factor to scale by

        :returns: a new ChassisAccelerations representing the scaled accelerations
        """
        return ChassisAccelerations(self.ax * scalar, self.ay * scalar, self.alpha * scalar)

    def __str__(self) -> str:
        """
        Returns a human-readable string representation of this acceleration.
        """
        return f"ChassisAccelerations(ax={self.ax:.3f}, ay={self.ay:.3f}, ao={self.alpha:.3f})"
