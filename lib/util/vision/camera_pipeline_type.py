from enum import Enum, auto


class CameraPipelineType(Enum):
    DISABLED = auto()
    APRIL_TAG = auto()
    OBJECT_DETECTION = auto()
    OTHER = auto()
