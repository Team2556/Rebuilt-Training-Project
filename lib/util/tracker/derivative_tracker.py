class DerivativeTracker:

    def __init__(self, initial: float) -> None:
        self._tracked_measurement = initial
        self._tracked_derivative = 0.0

    def update(self, measure: float, dt: float) -> None:
        self._tracked_derivative = (measure - self._tracked_measurement) / dt
        self._tracked_measurement = measure

    def get_tracked_derivative(self) -> float:
        return self._tracked_derivative
