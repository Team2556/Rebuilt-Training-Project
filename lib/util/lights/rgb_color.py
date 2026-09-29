from typing import ClassVar

from phoenix6.signals import RGBWColor
from wpilib import Color


class RGBColor:

    LIME: ClassVar["RGBColor"]
    NONE: ClassVar["RGBColor"]
    RED: ClassVar["RGBColor"]
    ORANGE: ClassVar["RGBColor"]
    YELLOW: ClassVar["RGBColor"]
    GREEN: ClassVar["RGBColor"]
    BLUE: ClassVar["RGBColor"]
    AQUA: ClassVar["RGBColor"]
    PURPLE: ClassVar["RGBColor"]

    def __init__(self, red: int, green: int, blue: int) -> None:
        self.r = red
        self.g = green
        self.b = blue

    @staticmethod
    def from_wpi_color(color: Color) -> "RGBColor":
        return RGBColor(int(color.red * 255), int(color.green * 255), int(color.blue * 255))

    def __str__(self) -> str:
        return "R: " + str(self.r) + ", G:" + str(self.g) + ", B:" + str(self.b)

    def to_rgbw_color(self) -> RGBWColor:
        return RGBWColor(self.r, self.g, self.b, 0)

    def __eq__(self, o: object) -> bool:
        if isinstance(o, RGBColor):
            return o is self or (self.r == o.r and self.b == o.b and self.g == o.g)
        return False

    def __hash__(self) -> int:
        return hash((self.r, self.g, self.b))


RGBColor.LIME = RGBColor(102, 255, 0)
RGBColor.NONE = RGBColor(0, 0, 0)
RGBColor.RED = RGBColor(255, 0, 0)
RGBColor.ORANGE = RGBColor(255, 185, 0)
RGBColor.YELLOW = RGBColor(255, 255, 0)
RGBColor.GREEN = RGBColor(0, 255, 0)
RGBColor.BLUE = RGBColor(0, 0, 255)
RGBColor.AQUA = RGBColor(0, 255, 255)
RGBColor.PURPLE = RGBColor(255, 0, 255)
