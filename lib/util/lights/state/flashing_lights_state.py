from typing import TYPE_CHECKING

from wpimath import units

from ...stopwatch import Stopwatch
from ..led_closed_interval import LEDClosedInterval
from ..rgb_color import RGBColor
from .lights_state import LightsState

if TYPE_CHECKING:
    from ....io.lights.lights_io import LightsIO


class FlashingLightsState(LightsState):

    def __init__(
        self, segment: LEDClosedInterval, interval: units.seconds, *colors: RGBColor
    ) -> None:
        super().__init__(segment)
        self._time_interval = interval
        self.stop_watch = Stopwatch()
        self.colors = list(colors)
        self.current_index = 0

    def desire_change(self) -> bool:
        self.stop_watch.start_if_not_running()
        return self.stop_watch.get_time() >= self._time_interval

    def get_rgb_color(self) -> RGBColor:
        if self.desire_change():
            self.current_index += 1
            if self.current_index >= len(self.colors):
                self.current_index = 0
            self.stop_watch.reset_and_start()
        return self.colors[self.current_index]

    def apply(self, io: "LightsIO") -> None:
        io.set_led(self.get_rgb_color(), self.interval)

    def __str__(self) -> str:
        return "Flashing: " + str(self.colors[self.current_index])
