from enum import Enum


class ForwardDirection(Enum):
    POSITIVE = 1
    NEGATIVE = -1

    @property
    def invert_multiplier(self) -> float:
        return float(self.value)

    def get_invert_multiplier(self) -> float:
        return self.invert_multiplier

    def apply(self, scalar: float) -> float:
        return scalar * self.invert_multiplier
