from typing import Protocol


class Loopable(Protocol):

    def loop(self) -> None: ...
