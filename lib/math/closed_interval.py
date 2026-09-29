from ..util.math_helpers import MathHelpers


class ClosedInterval:

    def __init__(self, start: int, end: int) -> None:
        self.start = start
        self.end = end

    def get_start(self) -> int:
        return self.start

    def get_end(self) -> int:
        return self.end

    def get_length(self) -> int:
        return self.end - self.start

    def get_index(self, index: int) -> int:
        value = self.start + index
        if value > self.end:
            raise IndexError("Overflowing Closed Interval")
        return value

    def get_from_index_range(
        self, index_start: "int | ClosedInterval", index_end: int | None = None
    ) -> "ClosedInterval":
        if isinstance(index_start, ClosedInterval):
            return self.get_from_index_range(index_start.get_start(), index_start.get_end())
        if index_end is None:
            return self.get_from_index_range(0, index_start)
        start = int(MathHelpers.clamp(index_start + self.start, self.end, self.start))
        end = int(MathHelpers.clamp(index_end + self.start, self.end, self.start))
        return ClosedInterval(start, end)

    def collides(self, other: "ClosedInterval") -> bool:
        return max(self.start, other.start) <= min(self.end, other.end)

    def __eq__(self, o: object) -> bool:
        if isinstance(o, ClosedInterval):
            return o.start == self.start and o.end == self.end
        return False

    def __hash__(self) -> int:
        return hash((self.start, self.end))
