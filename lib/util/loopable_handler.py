from .loopable import Loopable


class LoopableHandler:

    def __init__(self, want_blacklist_self: bool = True) -> None:
        self._loops: list[Loopable] = []
        self._blacklisted_loops: list[Loopable] = []
        if want_blacklist_self:
            self.blacklist(self)

    def blacklist(self, loopable: Loopable) -> None:
        self._blacklisted_loops.append(loopable)

    def unblacklist(self, loopable: Loopable) -> None:
        self._blacklisted_loops.remove(loopable)

    def unblacklist_self(self) -> None:
        self.unblacklist(self)

    def is_blacklisted(self, loopable: Loopable) -> bool:
        if not self._blacklisted_loops:
            return False
        if len(self._blacklisted_loops) == 1:
            return self._blacklisted_loops[0] == loopable
        else:
            return loopable in self._blacklisted_loops

    def add_loop(self, loopable: Loopable) -> bool:
        if loopable is None:
            raise ValueError("loopable must not be null in addLoop")
        if self.is_blacklisted(loopable):
            return False
        else:
            self._loops.append(loopable)
            return True

    def loop(self) -> None:
        for loopable in self._loops:
            loopable.loop()
