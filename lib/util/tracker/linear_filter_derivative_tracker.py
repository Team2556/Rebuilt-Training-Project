from wpimath.filter import LinearFilter

from .derivative_tracker import DerivativeTracker


class LinearFilterDerivativeTracker(DerivativeTracker):

    def __init__(self, filter: LinearFilter, initial: float) -> None:
        super().__init__(initial)
        self._filter = filter

    def update(self, measure: float, dt: float) -> None:
        measure = self._filter.calculate(measure)
        super().update(measure, dt)
