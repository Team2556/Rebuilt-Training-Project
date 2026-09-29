import math
from typing import Callable, overload

from robotpy_apriltag import AprilTag, AprilTagFieldLayout
from wpilib import DriverStation
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d


def _default_alliance_supplier() -> bool:
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


_alliance_supplier: Callable[[], bool] = _default_alliance_supplier


class FieldLayout:
    """
    Contains various field dimensions and useful reference points. Dimensions are
    in meters, and sets
    of corners start in the lower left moving clockwise. **All units in
    Meters**

    All translations and poses are stored with the origin at the rightmost point
    on the BLUE
    ALLIANCE wall.

    Length refers to the *x* direction (as described by wpilib)
    Width refers to the *y* direction (as described by wpilib)

    Only game-independent geometry lives here. Per-game reference points and the
    AprilTag map are supplied by the robot project through :meth:`configure`.
    """

    K_FIELD_LENGTH: units.meters = units.feetToMeters(54.0) + units.inchesToMeters(3.2)

    K_FIELD_WIDTH: units.meters = units.feetToMeters(26.0) + units.inchesToMeters(5.7)

    K_APRIL_TAG_MAP: AprilTagFieldLayout | None = None

    K_APRIL_TAG_WIDTH: units.meters = units.inchesToMeters(6.5)

    K_GROUND_ALTITUDE: units.meters = 0.0

    FIELD_CROP: units.meters = units.feetToMeters(-0.5)

    @staticmethod
    def configure(
        april_tag_map: AprilTagFieldLayout | None = None,
        field_length: units.meters | None = None,
        field_width: units.meters | None = None,
    ) -> None:
        """
        Install the per-game field description. Call once during robot start-up,
        before anything reads the tag map.

        :param april_tag_map:  layout of this game's AprilTags
        :param field_length:  override for the field's *x* extent
        :param field_width:  override for the field's *y* extent
        """
        if april_tag_map is not None:
            FieldLayout.K_APRIL_TAG_MAP = april_tag_map
        if field_length is not None:
            FieldLayout.K_FIELD_LENGTH = field_length
        if field_width is not None:
            FieldLayout.K_FIELD_WIDTH = field_width

    @staticmethod
    def set_alliance_supplier(supplier: Callable[[], bool]) -> None:
        """
        Override how the layout decides which alliance the robot is on. Defaults
        to reading the Driver Station.

        :param supplier:  returns true when the robot is on the red alliance
        """
        global _alliance_supplier
        _alliance_supplier = supplier

    @staticmethod
    def is_red_alliance() -> bool:
        return _alliance_supplier()

    @staticmethod
    def get_april_tag_map() -> AprilTagFieldLayout:
        if FieldLayout.K_APRIL_TAG_MAP is None:
            raise RuntimeError(
                "FieldLayout has no AprilTag map; call FieldLayout.configure(april_tag_map=...)"
            )
        return FieldLayout.K_APRIL_TAG_MAP

    @staticmethod
    def get_april_tag_array_from_ids(ids: list[int]) -> list[AprilTag]:
        buffer: list[AprilTag] = []
        for tag in FieldLayout.get_april_tag_map().getTags():
            for id in ids:
                if id == tag.ID:
                    buffer.append(tag)
                    break
        return buffer

    @staticmethod
    def get_april_tag_by_id(id: int) -> AprilTag | None:
        for tag in FieldLayout.get_april_tag_map().getTags():
            if tag.ID == id:
                return tag
        return None

    @staticmethod
    def get_tags_on_robot_alliance() -> list[AprilTag]:
        tags: list[AprilTag] = []
        for tag in FieldLayout.get_april_tag_map().getTags():
            if FieldLayout.is_pose_on_robot_alliance(tag.pose.toPose2d()):
                tags.append(tag)
        return tags

    @staticmethod
    def get_pose_array_from_april_tag_array(tags: list[AprilTag]) -> list[Pose3d]:
        return [tag.pose for tag in tags]

    @staticmethod
    def get_id_array_from_april_tag_array(tags: list[AprilTag]) -> list[int]:
        return [tag.ID for tag in tags]

    @staticmethod
    def is_pose_on_red_side(pose: Pose2d) -> bool:
        return not pose.X() < FieldLayout.K_FIELD_LENGTH / 2

    @staticmethod
    def is_pose_on_robot_alliance(pose: Pose2d, is_red_alliance: bool | None = None) -> bool:
        if is_red_alliance is None:
            is_red_alliance = FieldLayout.is_red_alliance()
        return FieldLayout.is_pose_on_red_side(pose) and is_red_alliance

    @staticmethod
    def is_april_tag_on_red_alliance(tag: AprilTag, is_red_alliance: bool) -> bool:
        return FieldLayout.is_pose_on_robot_alliance(tag.pose.toPose2d(), is_red_alliance)

    @staticmethod
    def is_april_tag_on_robot_alliance(tag: AprilTag) -> bool:
        return FieldLayout.is_april_tag_on_red_alliance(tag, FieldLayout.is_red_alliance())

    @staticmethod
    def is_pose_on_left_side(is_red_alliance: bool, y_coordinate: units.meters) -> bool:
        if is_red_alliance:
            return y_coordinate <= FieldLayout.K_FIELD_WIDTH / 2.0
        else:
            return y_coordinate >= FieldLayout.K_FIELD_WIDTH / 2.0

    @overload
    @staticmethod
    def handle_alliance_flip(blue: Pose2d, is_red_alliance: bool) -> Pose2d: ...

    @overload
    @staticmethod
    def handle_alliance_flip(blue: Translation3d, is_red_alliance: bool) -> Translation3d: ...

    @overload
    @staticmethod
    def handle_alliance_flip(blue: Translation2d, is_red_alliance: bool) -> Translation2d: ...

    @overload
    @staticmethod
    def handle_alliance_flip(blue: Rotation2d, is_red_alliance: bool) -> Rotation2d: ...

    @staticmethod
    def handle_alliance_flip(
        blue: Pose2d | Translation2d | Translation3d | Rotation2d, is_red_alliance: bool
    ) -> Pose2d | Translation2d | Translation3d | Rotation2d:
        if isinstance(blue, Pose2d):
            if is_red_alliance:
                blue = FieldLayout.rotate_about_center(blue, Rotation2d.fromDegrees(180))
            return blue
        if isinstance(blue, Translation3d):
            if is_red_alliance:
                blue = blue.rotateAround(
                    Translation3d(
                        FieldLayout.K_FIELD_LENGTH / 2.0, FieldLayout.K_FIELD_WIDTH / 2.0, blue.Y()
                    ),
                    Rotation3d(0.0, 0.0, math.pi),
                )
            return blue
        if isinstance(blue, Translation2d):
            if is_red_alliance:
                blue = blue.rotateAround(
                    Translation2d(
                        FieldLayout.K_FIELD_LENGTH / 2.0, FieldLayout.K_FIELD_WIDTH / 2.0
                    ),
                    Rotation2d.fromDegrees(180),
                )
            return blue
        if is_red_alliance:
            blue = blue + Rotation2d.fromDegrees(180)
        return blue

    @staticmethod
    def distance_from_alliance_wall(
        x_coordinate: units.meters, is_red_alliance: bool
    ) -> units.meters:
        if is_red_alliance:
            return FieldLayout.K_FIELD_LENGTH - x_coordinate
        return x_coordinate

    @overload
    @staticmethod
    def mirror_about_x(t: Rotation2d, x_value: units.meters | None = None) -> Rotation2d: ...

    @overload
    @staticmethod
    def mirror_about_x(t: Pose2d, x_value: units.meters) -> Pose2d: ...

    @overload
    @staticmethod
    def mirror_about_x(t: Translation2d, x_value: units.meters) -> Translation2d: ...

    @staticmethod
    def mirror_about_x(
        t: Translation2d | Rotation2d | Pose2d, x_value: units.meters | None = None
    ) -> Translation2d | Rotation2d | Pose2d:
        if isinstance(t, Rotation2d):
            return Rotation2d(-t.cos(), t.sin())
        if x_value is None:
            raise TypeError("mirror_about_x requires x_value unless mirroring a Rotation2d")
        if isinstance(t, Pose2d):
            return Pose2d(
                FieldLayout.mirror_about_x(t.translation(), x_value),
                FieldLayout.mirror_about_x(t.rotation()),
            )
        return Translation2d(x_value + (x_value - t.X()), t.Y())

    @overload
    @staticmethod
    def mirror_about_y(t: Rotation2d, y_value: units.meters | None = None) -> Rotation2d: ...

    @overload
    @staticmethod
    def mirror_about_y(t: Pose2d, y_value: units.meters) -> Pose2d: ...

    @overload
    @staticmethod
    def mirror_about_y(t: Translation2d, y_value: units.meters) -> Translation2d: ...

    @staticmethod
    def mirror_about_y(
        t: Translation2d | Rotation2d | Pose2d, y_value: units.meters | None = None
    ) -> Translation2d | Rotation2d | Pose2d:
        if isinstance(t, Rotation2d):
            return Rotation2d(t.cos(), -t.sin())
        if y_value is None:
            raise TypeError("mirror_about_y requires y_value unless mirroring a Rotation2d")
        if isinstance(t, Pose2d):
            return Pose2d(
                FieldLayout.mirror_about_y(t.translation(), y_value),
                FieldLayout.mirror_about_y(t.rotation()),
            )
        return Translation2d(t.X(), y_value + (y_value - t.Y()))

    @staticmethod
    def rotate_about_pose(start_pose: Pose2d, point: Translation2d, rotation: Rotation2d) -> Pose2d:
        return Pose2d(
            start_pose.translation().rotateAround(point, rotation),
            start_pose.rotation() + rotation,
        )

    @overload
    @staticmethod
    def flip_about_midline(target: Pose2d) -> Pose2d: ...

    @overload
    @staticmethod
    def flip_about_midline(target: Translation2d) -> Translation2d: ...

    @staticmethod
    def flip_about_midline(target: Translation2d | Pose2d) -> Translation2d | Pose2d:
        if isinstance(target, Pose2d):
            return Pose2d(FieldLayout.K_FIELD_LENGTH - target.X(), target.Y(), target.rotation())
        return Translation2d(FieldLayout.K_FIELD_LENGTH - target.X(), target.Y())

    @overload
    @staticmethod
    def flip_across_y(target: Pose2d) -> Pose2d: ...

    @overload
    @staticmethod
    def flip_across_y(target: Translation2d) -> Translation2d: ...

    @overload
    @staticmethod
    def flip_across_y(target: units.meters) -> units.meters: ...

    @staticmethod
    def flip_across_y(
        target: Translation2d | Pose2d | units.meters,
    ) -> Translation2d | Pose2d | units.meters:
        if isinstance(target, Pose2d):
            return Pose2d(target.X(), FieldLayout.K_FIELD_WIDTH - target.Y(), target.rotation())
        if isinstance(target, Translation2d):
            return Translation2d(target.X(), FieldLayout.K_FIELD_WIDTH - target.Y())
        return FieldLayout.K_FIELD_WIDTH - target

    @staticmethod
    def flip_across_x(d: units.meters) -> units.meters:
        return FieldLayout.K_FIELD_LENGTH - d

    @staticmethod
    def rotate_about_center(start_pose: Pose2d, rotation: Rotation2d) -> Pose2d:
        return FieldLayout.rotate_about_pose(
            start_pose,
            Translation2d(FieldLayout.K_FIELD_LENGTH / 2.0, FieldLayout.K_FIELD_WIDTH / 2.0),
            rotation,
        )

    @staticmethod
    def outside_field(pose: Pose2d) -> bool:
        return (
            pose.X() >= FieldLayout.K_FIELD_LENGTH - FieldLayout.FIELD_CROP
            or pose.X() <= 0.0 + FieldLayout.FIELD_CROP
            or pose.Y() >= FieldLayout.K_FIELD_WIDTH - FieldLayout.FIELD_CROP
            or pose.Y() <= 0.0 + FieldLayout.FIELD_CROP
        )

    @staticmethod
    def get_field_width_midline() -> units.meters:
        return FieldLayout.K_FIELD_WIDTH / 2.0
