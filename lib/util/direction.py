from enum import Enum, auto


class Direction(Enum):
    FORWARD = auto()
    REVERSE = auto()
    LEFT = auto()
    RIGHT = auto()

    def is_reversed(self) -> bool:
        return self is Direction.REVERSE

    def is_foward(self) -> bool:
        return self is Direction.FORWARD
