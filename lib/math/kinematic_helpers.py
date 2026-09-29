from typing import Any, overload

from wpimath import units
from wpimath.geometry import Pose2d, Twist2d
from wpimath.kinematics import ChassisSpeeds

from .chassis_accelerations import ChassisAccelerations


class KinematicHelpers:

    @overload
    @staticmethod
    def first_order_approximate(derivative: float, dt: units.seconds, /) -> float: ...

    @overload
    @staticmethod
    def first_order_approximate(
        drive_speeds: ChassisSpeeds, dt: units.seconds, /
    ) -> Twist2d: ...

    @overload
    @staticmethod
    def first_order_approximate(
        measure: float, derivative: float, dt: units.seconds, /
    ) -> float: ...

    @overload
    @staticmethod
    def first_order_approximate(
        pose: Pose2d, drive_speeds: ChassisSpeeds, dt: units.seconds, /
    ) -> Pose2d: ...

    @overload
    @staticmethod
    def first_order_approximate(
        vx: float, vy: float, vo: float, dt: units.seconds, /
    ) -> Twist2d: ...

    @staticmethod
    def first_order_approximate(*args: Any) -> float | Twist2d | Pose2d:
        """
        First-order (Euler) integration.

        Computes the new value of a quantity using its first derivative,
        assuming the derivative remains constant over the time step.

        dx = v * dt
        x_new = x + dx

        Accepted forms:

        (derivative, dt)
            :param derivative: First derivative (v), e.g., velocity
            :param dt:         Time step over which to integrate
            :returns:          Change in position after the time step

        (measure, derivative, dt)
            :param measure:    Current value (x), e.g., position
            :param derivative: First derivative (v), e.g., velocity
            :param dt:         Time step over which to integrate
            :returns: Updated value after dt (x_new)

        (vx, vy, vo, dt)
            :returns: Twist2d representing the change in pose over dt seconds

        (driveSpeeds, dt)
            Computes the change in position of a quantity using the robot's velocity,
            assuming the robot's velocity remains constant over the time step.

            :param driveSpeeds:      Robot's robot centric chasis speeds
            :param dt:               Time step over which to integrate (seconds)
            :returns:                 Twist2d representing the change in pose over dt seconds

        (pose, driveSpeeds, dt)
            Computes the change in position of a quantity using the robot's velocity,
            assuming the robot's velocity remains constant over the time step.

            :param pose:             Current pose (Meters & Radians)
            :param driveSpeeds:      Robot's robot centric chasis speeds
            :param dt:               Time step over which to integrate (seconds)
            :returns:                 Pose after the time step
        """
        match args:
            case (Pose2d() as pose, ChassisSpeeds() as drive_speeds, dt):
                return pose.exp(KinematicHelpers.first_order_approximate(drive_speeds, dt))
            case (ChassisSpeeds() as drive_speeds, dt):
                return KinematicHelpers.first_order_approximate(
                    drive_speeds.vx, drive_speeds.vy, drive_speeds.omega, dt
                )
            case (vx, vy, vo, dt):
                return Twist2d(
                    KinematicHelpers.first_order_approximate(vx, dt),
                    KinematicHelpers.first_order_approximate(vy, dt),
                    KinematicHelpers.first_order_approximate(vo, dt),
                )
            case (measure, derivative, dt):
                return measure + KinematicHelpers.first_order_approximate(derivative, dt)
            case (derivative, dt):
                return derivative * dt
        raise TypeError("unsupported argument list for first_order_approximate")

    @overload
    @staticmethod
    def second_order_approximate(
        derivative: float, second_derivative: float, dt: units.seconds, /
    ) -> float: ...

    @overload
    @staticmethod
    def second_order_approximate(
        drive_speeds: ChassisSpeeds, acceleration: ChassisAccelerations, dt: units.seconds, /
    ) -> Twist2d: ...

    @overload
    @staticmethod
    def second_order_approximate(
        measure: float, derivative: float, second_derivative: float, dt: units.seconds, /
    ) -> float: ...

    @overload
    @staticmethod
    def second_order_approximate(
        pose: Pose2d,
        drive_speeds: ChassisSpeeds,
        acceleration: ChassisAccelerations,
        dt: units.seconds,
        /,
    ) -> Pose2d: ...

    @overload
    @staticmethod
    def second_order_approximate(
        vx: float, vy: float, vo: float, ax: float, ay: float, ao: float, dt: units.seconds, /
    ) -> Twist2d: ...

    @staticmethod
    def second_order_approximate(*args: Any) -> float | Twist2d | Pose2d:
        """
        Second-order (constant acceleration) integration.

        Computes the new value of a quantity assuming constant acceleration over the
        time step.

        dx = v * dt + (1/2) * a * dt^2
        x_new = x + dx

        Accepted forms:

        (derivative, secondDerivative, dt)
            :param derivative:       First derivative (v), e.g., velocity
            :param secondDerivative: Second derivative (a), e.g., acceleration
            :param dt:               Time step over which to integrate (seconds)
            :returns:                 Change in position after the time step

        (measure, derivative, secondDerivative, dt)
            :param derivative:       First derivative (v), e.g., velocity
            :param secondDerivative: Second derivative (a), e.g., acceleration
            :param dt:               Time step over which to integrate (seconds)
            :returns: Updated value after dt (x_new)

        (vx, vy, vo, ax, ay, ao, dt)
            :returns:                  Twist2d representing the change in pose over dt seconds

        (driveSpeeds, acceleration, dt)
            :param driveSpeeds:       Robot's robot centric chasis speeds
            :param acceleration:      Robot's acceleration
            :param dt:                Time step (seconds)
            :returns:                  Twist2d representing the change in pose over dt seconds

        (pose, driveSpeeds, acceleration, dt)
            :param pose:              Current pose
            :param driveSpeeds:       Robot's robot centric chasis speeds
            :param acceleration:      Robot's acceleration
            :param dt:                Time step
            :returns:                  Twist2d representing the change in pose over dt seconds
        """
        match args:
            case (Pose2d() as pose, ChassisSpeeds() as drive_speeds, ChassisAccelerations() as acceleration, dt):
                return pose.exp(
                    KinematicHelpers.second_order_approximate(drive_speeds, acceleration, dt)
                )
            case (ChassisSpeeds() as drive_speeds, ChassisAccelerations() as acceleration, dt):
                vx = drive_speeds.vx
                vy = drive_speeds.vy
                vo = drive_speeds.omega

                ax = acceleration.ax
                ay = acceleration.ay
                ao = acceleration.alpha

                return KinematicHelpers.second_order_approximate(vx, vy, vo, ax, ay, ao, dt)
            case (vx, vy, vo, ax, ay, ao, dt):
                return Twist2d(
                    KinematicHelpers.second_order_approximate(vx, ax, dt),
                    KinematicHelpers.second_order_approximate(vy, ay, dt),
                    KinematicHelpers.second_order_approximate(vo, ao, dt),
                )
            case (measure, derivative, second_derivative, dt):
                return measure + KinematicHelpers.second_order_approximate(
                    derivative, second_derivative, dt
                )
            case (derivative, second_derivative, dt):
                return (
                    KinematicHelpers.first_order_approximate(derivative, dt)
                    + 0.5 * second_derivative * dt * dt
                )
        raise TypeError("unsupported argument list for second_order_approximate")
