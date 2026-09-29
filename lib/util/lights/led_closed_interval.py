from enum import Enum, auto

from ...math.closed_interval import ClosedInterval


class LEDClosedIntervalCollisionBehavior(Enum):
    SELF_CANCEL = auto()
    INCOMING_CANCEL = auto()


class LEDClosedInterval(ClosedInterval):

    LEDClosedIntervalCollisionBehavior = LEDClosedIntervalCollisionBehavior

    def __init__(
        self,
        start: int,
        end: int,
        collision_behavior: LEDClosedIntervalCollisionBehavior = (
            LEDClosedIntervalCollisionBehavior.SELF_CANCEL
        ),
    ) -> None:
        super().__init__(start, end)
        self._collision_behavior = collision_behavior

    def with_collision_behavior(
        self, collision_behavior: LEDClosedIntervalCollisionBehavior
    ) -> "LEDClosedInterval":
        self._collision_behavior = collision_behavior
        return self

    def get_collision_behavior(self) -> LEDClosedIntervalCollisionBehavior:
        return self._collision_behavior

    def decide_collision(self, incoming: "LEDClosedInterval") -> "LEDClosedInterval":
        match self._collision_behavior:
            case LEDClosedIntervalCollisionBehavior.SELF_CANCEL:
                return incoming
            case LEDClosedIntervalCollisionBehavior.INCOMING_CANCEL:
                return self
