from typing import Generic, TypeVar

T = TypeVar("T")


class SnakeArrayAdvancer(Generic[T]):
    """
    used for advancing arrays in an snake or linked list fashion
    """

    def __init__(self, *buffer: T) -> None:
        """
        :param buffer: array buffer
        """
        self._buffer = list(buffer)
        self._current_index = 0

    def is_empty(self) -> bool:
        return len(self._buffer) == 0

    def zero(self) -> None:
        """
        sets `current_index` to 0
        """
        self._current_index = 0

    def max(self) -> None:
        if len(self._buffer) > 0:
            self._current_index = len(self._buffer) - 1
        else:
            self._current_index = 0

    def advance(self, want_wrap: bool) -> T | None:
        """
        :param want_wrap: if `current_index` will return to 0 once `current_index` is greater than the buffer size
        :returns: the advanced item or nothing
        """
        if len(self._buffer) == 0:
            return None
        next_ = self.safe_current()
        self._current_index += 1
        if self._current_index == self.get_buffer_length() or next_ is None:
            if want_wrap:
                self.zero()
        return next_

    def deadvance(self, want_wrap: bool) -> T | None:
        if len(self._buffer) == 0:
            return None
        current = self.safe_current()
        self._current_index -= 1
        if self._current_index == -1 or current is None:
            if want_wrap:
                self.max()
        return current

    def get_current_index(self) -> int:
        return self._current_index

    def current(self) -> T:
        """
        :returns: `current_index` index of `buffer`
        """
        return self._buffer[self._current_index]

    def safe_current(self) -> T | None:
        """
        returns the `current()` in an safe manner (handles for errors by returning `None`)

        :returns: the current element or nothing
        """
        try:
            if self._current_index >= len(self._buffer):
                return None
            if len(self._buffer) == 0:
                return None
            if self._current_index < 0:
                return None
            return self.current()
        except Exception:
            return None

    def get_buffer_length(self) -> int:
        return len(self._buffer)
