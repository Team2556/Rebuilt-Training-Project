import math

from wpilib import Timer
from wpimath import units


class Stopwatch:

    def __init__(self) -> None:
        self._start_time = math.inf

    def start(self) -> None:
        self._start_time = Timer.getFPGATimestamp()

    def start_if_not_running(self) -> None:
        if math.isinf(self._start_time):
            self.start()

    def get_time(self) -> units.seconds:
        if math.isinf(self._start_time):
            return 0.0
        return Timer.getFPGATimestamp() - self._start_time

    def get_time_as_double(self) -> float:
        if math.isinf(self._start_time):
            return 0.0
        return Timer.getFPGATimestamp() - self._start_time

    def reset(self) -> None:
        self._start_time = math.inf

    def reset_and_start(self) -> None:
        self._start_time = math.inf
        self.start()
