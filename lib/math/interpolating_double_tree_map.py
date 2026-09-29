import bisect
import math


class InterpolatingDoubleTreeMap:
    """Python port of WPILib's InterpolatingDoubleTreeMap.

    Keys are kept sorted, and values between them are linearly interpolated.
    Queries outside the key range are clamped to the nearest entry's value, so
    :meth:`get` alone cannot tell you whether a lookup was extrapolated. Use
    :meth:`is_within_range` when the difference matters.

    Interpolation is a plain bisect and lerp rather than ``numpy.interp``: these
    tables are a handful of points and are sampled several times per 20 ms loop,
    where rebuilding numpy arrays on every call costs more than the lookup.
    """

    def __init__(self) -> None:
        self._keys: list[float] = []
        self._values: list[float] = []

    @staticmethod
    def of_entries(*entries: tuple[float, float]) -> "InterpolatingDoubleTreeMap":
        tree_map = InterpolatingDoubleTreeMap()
        for key, value in entries:
            tree_map.put(key, value)
        return tree_map

    def put(self, key: float, value: float) -> None:
        index = bisect.bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            self._values[index] = value
            return
        self._keys.insert(index, key)
        self._values.insert(index, value)

    def get(self, key: float) -> float | None:
        keys = self._keys
        if not keys:
            return None
        if math.isnan(key):
            return math.nan

        values = self._values
        if key <= keys[0]:
            return values[0]
        if key >= keys[-1]:
            return values[-1]

        index = bisect.bisect_left(keys, key)
        if keys[index] == key:
            return values[index]

        low_key, high_key = keys[index - 1], keys[index]
        low_value, high_value = values[index - 1], values[index]
        return low_value + (high_value - low_value) * (key - low_key) / (high_key - low_key)

    def min_key(self) -> float | None:
        return self._keys[0] if self._keys else None

    def max_key(self) -> float | None:
        return self._keys[-1] if self._keys else None

    def is_within_range(self, key: float) -> bool:
        """
        Whether ``key`` falls inside the sampled range, so :meth:`get` interpolates
        rather than clamping to an end point.

        A table with a single entry has ``min_key == max_key``, so only that exact key
        is in range. That is deliberate: a one-point table carries no distance
        information and every other lookup off it is a constant.
        """
        return bool(self._keys) and self._keys[0] <= key <= self._keys[-1]

    def clear(self) -> None:
        self._keys.clear()
        self._values.clear()
