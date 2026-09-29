from typing import TYPE_CHECKING

from ..led_closed_interval import LEDClosedInterval
from ..rgb_color import RGBColor
from .lights_state import LightsState

if TYPE_CHECKING:
    from ....io.lights.lights_io import LightsIO


class SolidLightsState(LightsState):

    def __init__(self, color: RGBColor, segment: LEDClosedInterval) -> None:
        super().__init__(segment)
        self._color = color

    def apply(self, io: "LightsIO") -> None:
        io.set_led(self._color, self.interval)

    def __str__(self) -> str:
        return "Solid: " + str(self._color)
