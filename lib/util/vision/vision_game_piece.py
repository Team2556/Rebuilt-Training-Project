from wpilib import Timer
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d


class VisionGamePiece:

    def __init__(self, pose: Pose3d | Pose2d, time_stamp: units.seconds | None = None) -> None:
        self._pose = Pose3d(pose) if isinstance(pose, Pose2d) else pose
        self._time_stamp = Timer.getFPGATimestamp() if time_stamp is None else time_stamp

    def get_pose3d(self) -> Pose3d:
        return self._pose

    def get_pose2d(self) -> Pose2d:
        return self._pose.toPose2d()

    def get_time_stamp(self) -> units.seconds:
        return self._time_stamp
