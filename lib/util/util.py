import bisect
import math
from typing import Callable, Iterable, TypeVar, overload

import commands2
from commands2 import Command, CommandScheduler, Subsystem
from phoenix6.swerve import SwerveDrivetrain
from wpilib import SmartDashboard, Timer
from wpimath import angleModulus, units
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory

from .axis3d.translation_axis3d import TranslationAxis3d
from .builder.transform3d_object_builder import Transform3dObjectBuilder
from .field_layout import FieldLayout
from .math_helpers import MathHelpers

T = TypeVar("T")


class Util:
    """
    Contains basic functions that are used often.
    """

    K_EPSILON = 1e-12

    @staticmethod
    def copy_sign_or_zero(magnitude: float, sign: float) -> float:
        return math.copysign(magnitude, sign) if sign != 0 else 0.0

    @staticmethod
    def limit(v: float, min: float, max: float | None = None) -> float:
        """
        Limits the given input to the given magnitude.
        """
        if max is None:
            return Util.limit(v, -min, min)
        return max if v > max else (min if v < min else v)

    @staticmethod
    def in_range(v: float, min: float, max: float | None = None) -> bool:
        """
        Checks if the given input is within the range (min, max), both exclusive.
        """
        if max is None:
            return Util.in_range(v, -min, min)
        return v > min and v < max

    @staticmethod
    def interpolate(a: float, b: float, x: float) -> float:
        x = Util.limit(x, 0.0, 1.0)
        return a + (b - a) * x

    @staticmethod
    def join_strings(delim: str, strings: list) -> str:
        return delim.join(str(s) for s in strings)

    @staticmethod
    def epsilon_equals(a, b, epsilon=None) -> bool:
        if isinstance(a, Translation2d):
            if epsilon is None:
                return Util.epsilon_equals(a.X(), b.X()) or Util.epsilon_equals(a.Y(), b.Y())
            return Util.epsilon_equals(a.X(), b.X(), epsilon) and Util.epsilon_equals(
                a.Y(), b.Y(), epsilon
            )
        if isinstance(a, ChassisSpeeds):
            if epsilon is None:
                return (
                    Util.epsilon_equals(a.vx, b.vx)
                    and Util.epsilon_equals(a.vy, b.vy)
                    and Util.epsilon_equals(a.omega, b.omega)
                )
            return Util.epsilon_equals(a.vx, b.vx, epsilon) and Util.epsilon_equals(
                a.vy, b.vy, epsilon
            )
        if epsilon is None:
            epsilon = Util.K_EPSILON
        return (a - epsilon <= b) and (a + epsilon >= b)

    @staticmethod
    def safe_equals_check(a, b) -> bool:
        if a is None and b is None:
            return True
        if a is None or b is None:
            return False
        return a == b

    @staticmethod
    def all_close_to(list: list[float], value: float, epsilon: float) -> bool:
        result = True
        for value_in in list:
            result &= Util.epsilon_equals(value_in, value, epsilon)
        return bool(result)

    @staticmethod
    def handle_deadband(value: float, deadband: float) -> float:
        deadband = abs(deadband)
        if deadband == 1:
            return 0
        scaled_value = (value + (deadband if value < 0 else -deadband)) / (1 - deadband)
        return scaled_value if abs(value) > abs(deadband) else 0

    @overload
    @staticmethod
    def flip_red_blue(original: Pose2d) -> Pose2d: ...

    @overload
    @staticmethod
    def flip_red_blue(original: Translation3d) -> Translation3d: ...

    @staticmethod
    def flip_red_blue(original: Translation3d | Pose2d) -> Translation3d | Pose2d:
        if isinstance(original, Pose2d):
            return Pose2d(
                FieldLayout.K_FIELD_LENGTH - original.X(), original.Y(), Rotation2d()
            )
        return Translation3d(
            FieldLayout.K_FIELD_LENGTH - original.X(), original.Y(), original.Z()
        )

    @staticmethod
    def memoize_by_iteration(
        iteration: Callable[[], int], delegate: Callable[[], T]
    ) -> Callable[[], T]:
        value: list[T | None] = [None]
        last_iteration = [-1]

        def supplier() -> T:
            last = last_iteration[0]
            now = iteration()
            if last != now:
                value[0] = None
            val = value[0]
            if val is None:
                val = delegate() if value[0] is None else value[0]
                value[0] = val
                last_iteration[0] = now
            return val

        return supplier

    class DistanceAngleConverter:
        """
        Class used store translate distances in the form of angles. Used for
        elevators to interface with the IO layer which only supports angles.
        """

        def __init__(self, radius: units.meters) -> None:
            self._radius = radius

        def to_angle(self, distance: units.meters) -> units.radians:
            """
            Converts a distance measurement to an equal angle measurement based on radius
            initialized with.

            :param distance: Distance to convert to angle.
            :returns: Angle distance is equivalent to.
            """
            return distance / self._radius

        def to_distance(self, angle: units.radians) -> units.meters:
            """
            Converts an angle measurement to an equal distance measurement based on
            radius initialized with.

            :param angle: angle to convert to distance.
            :returns: Distance agle is equivalent to.
            """
            return angle * self._radius

        def to_angular_velocity(
            self, velocity: units.meters_per_second
        ) -> units.radians_per_second:
            return self.to_angle(velocity)

        def to_linear_velocity(
            self, velocity: units.radians_per_second
        ) -> units.meters_per_second:
            return self.to_distance(velocity)

        def get_drum_radius(self) -> units.meters:
            return self._radius

    class InterpolatingMeasureMap:

        def __init__(self, data: list[tuple[float, float]] | None = None) -> None:
            self._keys: list[float] = []
            self._values: list[float] = []
            if data is not None:
                for key, value in data:
                    self.put(key, value)

        def put(self, key: float, value: float) -> None:
            index = bisect.bisect_left(self._keys, key)
            if index < len(self._keys) and self._keys[index] == key:
                self._values[index] = value
            else:
                self._keys.insert(index, key)
                self._values.insert(index, value)

        def get(self, key: float) -> float | None:
            if not self._keys:
                return None

            index = bisect.bisect_left(self._keys, key)

            if index < len(self._keys) and self._keys[index] == key:
                return self._values[index]
            if index == 0:
                return self._values[0]
            if index == len(self._keys):
                return self._values[-1]

            start_key = self._keys[index - 1]
            end_key = self._keys[index]
            t = (key - start_key) / (end_key - start_key)
            start_value = self._values[index - 1]
            end_value = self._values[index]
            return start_value + (end_value - start_value) * t

        def clear(self) -> None:
            self._keys.clear()
            self._values.clear()

    @staticmethod
    def inverse_pose(pose: Pose2d) -> Pose2d:
        """
        The inverse of this transform "undoes" the effect of translating by this
        transform.

        :returns: The opposite of this transform.
        """
        rotation_inverted = Util.inverse_rotation(pose.rotation())
        return Pose2d(
            Util.inverse_translation(pose.translation()).rotateBy(rotation_inverted),
            rotation_inverted,
        )

    @staticmethod
    def inverse_rotation(angle: Rotation2d) -> Rotation2d:
        """
        The inverse of a Rotation2d "undoes" the effect of this rotation.

        :returns: The inverse of this rotation.
        """
        return Rotation2d(angle.cos(), -angle.sin())

    @staticmethod
    def inverse_translation(translation: Translation2d) -> Translation2d:
        """
        The inverse simply means a Translation2d that "undoes" this object.

        :returns: Translation by -x and -y.
        """
        return Translation2d(-translation.X(), -translation.Y())

    @staticmethod
    def direction(translation: Translation2d) -> Rotation2d:
        return Rotation2d(translation.X(), translation.Y())

    @staticmethod
    def norm(translation: Translation2d) -> float:
        return math.hypot(translation.X(), translation.Y())

    class Pose2dTimeInterpolable:

        def __init__(
            self,
            traj_with_tan: Trajectory,
            start_heading: Rotation2d,
            end_heading: Rotation2d,
        ) -> None:
            self._pose_list: list[tuple[Pose2d, units.seconds]] = []
            total_time_secpnods = traj_with_tan.totalTime()
            for state in traj_with_tan.states():
                pose_rotation = MathHelpers.interpolate(
                    start_heading, end_heading, state.t / total_time_secpnods
                )
                self._pose_list.append(
                    (Pose2d(state.pose.translation(), pose_rotation), state.t)
                )
            SmartDashboard.putNumber(
                "Auto Align Traj/Number Of Trajectory States", len(self._pose_list)
            )

        def get_time_from_pose(self, pose: Pose2d) -> units.seconds:
            prev_state = self._pose_list[0]
            next_state = self._pose_list[0]
            for i in range(len(self._pose_list) - 1):
                if i >= len(self._pose_list) - 2:
                    return self._pose_list[-1][1]
                prev_state = self._pose_list[i]
                next_state = self._pose_list[i + 2]
                if prev_state[0].translation().distance(pose.translation()) < next_state[
                    0
                ].translation().distance(pose.translation()):
                    next_state = self._pose_list[i + 1]
                    break

            distance_to_prev_pose = prev_state[0].translation().distance(pose.translation())
            distance_to_next_pose = next_state[0].translation().distance(pose.translation())
            percent_to_next_pose = (
                prev_state[0].translation().distance(next_state[0].translation())
                - distance_to_next_pose
            ) / (distance_to_prev_pose + distance_to_next_pose)

            time_delta = next_state[1] - prev_state[1]
            time_at_pose = prev_state[1] + time_delta * percent_to_next_pose
            return time_at_pose

        def get_pose_from_time(self, time: units.seconds) -> Pose2d:
            if time >= self._pose_list[-1][1]:
                return self._pose_list[-1][0]
            elif time <= self._pose_list[0][1]:
                return self._pose_list[0][0]

            prev_state = self._pose_list[0]
            next_state = self._pose_list[0]

            for i in range(1, len(self._pose_list)):
                next_state = self._pose_list[i]
                if next_state[1] >= time:
                    prev_state = self._pose_list[i - 1]
                    break

            time_delta = next_state[1] + prev_state[1]
            percent_into_delta = (time - prev_state[1]) / time_delta
            # SmartDashboard.putNumber("Auto Align Traj/Percent As Delta",
            # percentIntoDelta)
            prev_to_time_pose = (next_state[0] - prev_state[0]) * percent_into_delta
            return prev_state[0] + prev_to_time_pose

        def clear_states_before_time(self, time: units.seconds) -> None:
            while len(self._pose_list) > 1 and self._pose_list[0][1] < time:
                self._pose_list.pop(0)

    @staticmethod
    def get_circle_intersection_points(
        center1: Translation2d, radius1: float, center2: Translation2d, radius2: float
    ) -> list[Translation2d]:
        """
        Calculates the intersection points of two circles.

        :param center1: Center point of the first circle
        :param radius1: Radius of the first circle
        :param center2: Center point of the second circle
        :param radius2: Radius of the second circle
        :returns: An ArrayList containing all intersection points of the circle.
                  ArrayList may have 0, 1, or 2 values.
        """
        all_points: list[Translation2d] = []

        delta = center2 - center1
        distance = center2.distance(center1)

        if distance > (abs(radius1) + abs(radius2)):
            return all_points  # Circles do not intersect

        a = (radius1 * radius1 - radius2 * radius2 + distance * distance) / (2 * distance)
        h = math.sqrt(radius1 * radius1 - a * a)

        point0 = center1 + (delta * a / distance)

        solution1 = point0 + (Translation2d(delta.Y(), -delta.X()) * h / distance)
        solution2 = point0 + (Translation2d(-delta.Y(), delta.X()) * h / distance)

        all_points.append(solution1)

        if solution1 != solution2:
            all_points.append(solution2)

        return all_points

    @staticmethod
    def get_empty_subsystem_set() -> set[Subsystem]:
        return set()

    @staticmethod
    def add_poses(a: Pose2d, b: Pose2d) -> Pose2d:
        return Pose2d(a.translation() + b.translation(), a.rotation() + b.rotation())

    @staticmethod
    def get_lowest_delta(base: units.radians, options: Iterable[units.radians]) -> units.radians:
        lowest_rotation = math.inf
        for a in options:
            diff_rotations = abs(units.radiansToRotations(base - a))
            if lowest_rotation > diff_rotations:
                lowest_rotation = diff_rotations
        return units.rotationsToRadians(lowest_rotation)

    @staticmethod
    def smart_dash_command(message: str) -> Command:
        return commands2.DeferredCommand(
            lambda: commands2.cmd.runOnce(
                lambda: SmartDashboard.putNumber(message, Timer.getFPGATimestamp())
            ),
            *Util.get_empty_subsystem_set(),
        )

    @staticmethod
    def min(x: float, y: float) -> float:
        return x if x < y else y

    @staticmethod
    def max(x: float, y: float) -> float:
        return x if x > y else y

    class ScheduleIfWontCancelOther(Command):

        def __init__(self, command_to_schedule: Command) -> None:
            super().__init__()
            self._command = command_to_schedule

        def initialize(self) -> None:
            for requirement in self._command.getRequirements():
                if requirement.getCurrentCommand() is not None:
                    return
            CommandScheduler.getInstance().schedule(self._command)

    @staticmethod
    def calculate_needed_field_relative_hold_angle(
        drive_state: SwerveDrivetrain.SwerveDriveState,
        target_pose: Translation2d,
        time: units.seconds,
    ) -> units.radians:

        drive_pose = drive_state.pose
        drive_speeds = drive_state.speeds

        distance_translation = drive_state.pose.translation() - target_pose

        dx = distance_translation.X()
        dy = distance_translation.Y()

        t = time

        # Convert robot-relative speeds -> field-relative
        field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive_speeds, drive_pose.rotation())

        vrx = field_speeds.vx
        vry = field_speeds.vy

        # Lead velocity vector (field-relative)
        vsx = dx / t - vrx
        vsy = dy / t - vry

        # Required facing angle (field-relative)
        field_relative_desired_angle = angleModulus(math.atan2(vsy, vsx)) + math.pi

        return field_relative_desired_angle

    @staticmethod
    def calculate_distance_to_start_deccel(
        current_vel: float, stowed_accel: float, raised_accel: float, time_to_raise: float
    ) -> float:
        dist_to_raise_elev = Util.calculat_distance_to_raise_elevator(raised_accel, time_to_raise)
        vel_at_dist_to_raise_elev = raised_accel * time_to_raise
        delta_vel = current_vel - vel_at_dist_to_raise_elev
        time_to_deccel_delta = delta_vel / stowed_accel
        dist_to_deccel_delta = 0.5 * stowed_accel * time_to_deccel_delta * time_to_deccel_delta
        total_dist = dist_to_deccel_delta + dist_to_raise_elev
        return total_dist

        # return (currentVel * currentVel) / (2.0 * stowedAccel) + ((timeToRaise *
        # timeToRaise * raisedAccel * 0.5) *
        # (1 - (raisedAccel / stowedAccel)));

    @staticmethod
    def calculat_distance_to_raise_elevator(raised_accel: float, time_to_raise: float) -> float:
        return 0.5 * raised_accel * time_to_raise * time_to_raise

    @staticmethod
    def add_to_translation3d(
        *args,
    ) -> Translation3d:
        match args:
            case (Translation3d() as translation, axis, offset):
                pass
            case (axis, offset):
                translation = Translation3d()
            case _:
                raise TypeError("unsupported argument list for add_to_translation3d")
        match axis:
            case TranslationAxis3d.X:
                return translation + Translation3d(offset, 0.0, 0.0)
            case TranslationAxis3d.Y:
                return translation + Translation3d(0.0, offset, 0.0)
            case TranslationAxis3d.Z:
                return translation + Translation3d(0.0, 0.0, offset)
            case _:
                raise TypeError("unsupported axis for add_to_translation3d")

    @overload
    @staticmethod
    def convert_onshape_to_wpi(target: Transform3d) -> Transform3d: ...

    @overload
    @staticmethod
    def convert_onshape_to_wpi(target: Translation3d) -> Translation3d: ...

    @staticmethod
    def convert_onshape_to_wpi(target: Translation3d | Transform3d) -> Translation3d | Transform3d:
        if isinstance(target, Transform3d):
            return (
                Transform3dObjectBuilder.from_transform(target)
                .with_x(target.Y())
                .with_y(target.X())
                .build()
            )
        return Translation3d(target.Y(), target.X(), target.Z())

    @staticmethod
    def test_current_pose(target_pose: Pose2d, current_pose: Pose2d) -> bool:
        return (
            abs(target_pose.X() - current_pose.X()) <= 0.2
            and abs(target_pose.Y() - current_pose.Y()) <= 0.2
            and abs(target_pose.rotation().degrees() - current_pose.rotation().degrees()) <= 5.0
        )
