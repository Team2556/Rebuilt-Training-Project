from typing import Callable, TypeVar

T = TypeVar("T")


class ArrayUtil:

    @staticmethod
    def invert_source(buffer: list[T]) -> list[T]:
        if buffer is None or len(buffer) == 0:
            return buffer

        real_length = len(buffer) - 1

        for i in range(len(buffer) // 2):
            low_index = i
            high_index = real_length - i

            low = buffer[low_index]
            high = buffer[high_index]

            buffer[low_index] = high
            buffer[high_index] = low

        return buffer

    @staticmethod
    def run_function_and_skip_null(buffer: list[T], function: Callable[[int, T], None]) -> int:
        if buffer is None:
            return 0
        iterations = 0
        for i in range(len(buffer)):
            element = buffer[i]
            try:
                if element is not None:
                    function(i, element)
                    iterations += 1
            except Exception:
                pass
        return iterations

    @staticmethod
    def foreach_without_null(buffer: list[T], function: Callable[[T], None]) -> int:
        if buffer is None:
            return 0
        iterations = 0
        for element in buffer:
            try:
                if element is not None:
                    function(element)
                    iterations += 1
            except Exception:
                pass
        return iterations

    @staticmethod
    def append_array_to_list(source: list[T], destination: list[T]) -> int:
        iterations = 0
        if source is None:
            raise ValueError("source must not be null in cpyArrayToList")
        if destination is None:
            raise ValueError("destination must not be null in cpyArrayToList")

        for element in source:
            destination.append(element)
            iterations += 1
        return iterations

    @staticmethod
    def cpy_list_to_array(source: list[T], destination: list[T]) -> int:
        if len(source) == 0:
            return 0

        if len(destination) < len(source):
            return -1

        iterations = 0

        for i in range(len(source)):
            destination[i] = source[i]
            iterations += 1
        return iterations
