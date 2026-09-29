from robotpy_apriltag import AprilTag
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Pose2d, Translation2d

from ...logging.log_util import LogUtil
from ..field_layout import FieldLayout


class VisionEstimate:

    def __init__(
        self,
        pose: Pose2d,
        timestamp: units.seconds | None = None,
        tags: list[AprilTag] | None = None,
        tag_ids: list[int] | None = None,
    ) -> None:
        self._pose = pose
        self._timestamp = Timer.getFPGATimestamp() if timestamp is None else timestamp
        if tags is None:
            tags = FieldLayout.get_april_tag_array_from_ids([] if tag_ids is None else tag_ids)
        self._tags = tags
        self._average_distance: units.meters | None = None

    def get_pose(self) -> Pose2d:
        return self._pose

    def get_timestamp(self) -> units.seconds:
        return self._timestamp

    def get_tags(self) -> list[AprilTag]:
        return self._tags

    def with_average_distance(self, average_distance: units.meters) -> "VisionEstimate":
        self._average_distance = average_distance
        return self

    def get_average_distance(self, reference: Translation2d | None = None) -> units.meters:
        """
        :param reference:  point the tags are measured from, normally the
            robot's current translation; ignored once a distance has been set
            with `with_average_distance`
        :returns: the mean distance from `reference` to the tags in this estimate
        """
        if self._average_distance is not None:
            return self._average_distance
        average: units.meters = 0.0
        if len(self._tags) > 0 and reference is not None:
            for tag in self._tags:
                average += tag.pose.translation().toTranslation2d().distance(reference)
            average = average / len(self._tags)
        return average

    def log(self, key: str) -> None:
        LogUtil.log(key + "/Pose", self._pose)
        SmartDashboard.putNumber(key + "/Timestamp Seconds", self._timestamp)
