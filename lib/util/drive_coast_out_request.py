from typing import Any

from phoenix6 import StatusCode
from phoenix6.controls import CoastOut, StaticBrake
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import SwerveRequest
from phoenix6.swerve.swerve_drivetrain import SwerveControlParameters


class DriveCoastOutRequest(SwerveRequest):

    def __init__(self) -> None:
        self._drive_request = CoastOut()
        """Local reference to a coast request for the drive motors"""
        self._steer_request = StaticBrake()
        """Local reference to a static brake request for the steer motors"""

    def apply(
        self,
        parameters: SwerveControlParameters,
        modules_to_apply: list[SwerveModule[Any, Any, Any]],
    ) -> StatusCode:
        for module in modules_to_apply:
            module.apply(self._drive_request, self._steer_request)
        return StatusCode.OK
