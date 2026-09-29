from dataclasses import dataclass

from .camera_pipeline_type import CameraPipelineType


@dataclass(frozen=True)
class CameraPipeline:
    type: CameraPipelineType
    name: str
    index: int

    @staticmethod
    def get_default() -> "CameraPipeline":
        return CameraPipeline(CameraPipelineType.DISABLED, "DEFAULT", -1)
