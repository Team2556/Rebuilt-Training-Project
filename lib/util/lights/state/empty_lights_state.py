from ..led_closed_interval import LEDClosedInterval
from ..rgb_color import RGBColor
from .solid_lights_state import SolidLightsState


class EmptyLightsState(SolidLightsState):

    def __init__(self, segment: LEDClosedInterval) -> None:
        super().__init__(RGBColor.NONE, segment)
