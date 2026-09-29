from typing import Callable

from wpimath import units

from ..led_closed_interval import LEDClosedInterval
from ..rgb_color import RGBColor
from .flashing_lights_state import FlashingLightsState


class DynamicFlashingLightsState(FlashingLightsState):

    def __init__(
        self,
        interval: LEDClosedInterval,
        time_interval_getter: Callable[[], units.seconds],
        colors_getter: Callable[[], list[RGBColor]],
    ) -> None:
        super().__init__(interval, 0.0, RGBColor.NONE)
        self._time_interval_getter = time_interval_getter
        self._colors_getter = colors_getter

        self._last_length = 0

    def desire_change(self) -> bool:
        self.stop_watch.start_if_not_running()
        return self.stop_watch.get_time() >= self._time_interval_getter()

    def get_rgb_color(self) -> RGBColor:
        current_color = self._colors_getter()
        if self.desire_change():
            if self._last_length != len(current_color):
                self.current_index = 0
                self._last_length = len(current_color)
            self.current_index += 1
            if self.current_index >= len(current_color):
                self.current_index = 0
            self.stop_watch.reset_and_start()
        return current_color[self.current_index]
