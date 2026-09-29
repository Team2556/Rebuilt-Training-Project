class DelayedBoolean:
    """
    An iterative boolean latch that delays the transition from false to true.
    """

    def __init__(self, timestamp: float, delay: float) -> None:
        self._transition_timestamp = timestamp
        self._last_value = False
        self._delay = delay

    def update(self, timestamp: float, value: bool) -> bool:
        result = False

        if value and not self._last_value:
            self._transition_timestamp = timestamp

        # If we are still true and we have transitioned.
        if value and (timestamp - self._transition_timestamp > self._delay):
            result = True

        self._last_value = value
        return result
