from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from ..led_closed_interval import LEDClosedInterval, LEDClosedIntervalCollisionBehavior

if TYPE_CHECKING:
    from ....io.lights.lights_io import LightsIO


class LightsState(ABC):

    def __init__(self, interval: LEDClosedInterval) -> None:
        self.interval = interval

    def with_collision_behavior(
        self, collision_behavior: LEDClosedIntervalCollisionBehavior
    ) -> "LightsState":
        self.interval.with_collision_behavior(collision_behavior)
        return self

    def get_interval(self) -> LEDClosedInterval:
        return self.interval

    def handle_collision(self, colliding: "LightsState") -> "LightsState":
        if self == colliding:
            return self
        if self.get_interval().collides(colliding.get_interval()):
            match self.get_interval().get_collision_behavior():
                case LEDClosedIntervalCollisionBehavior.SELF_CANCEL:
                    return colliding
                case LEDClosedIntervalCollisionBehavior.INCOMING_CANCEL:
                    return self
        return self

    @abstractmethod
    def apply(self, io: "LightsIO") -> None: ...
