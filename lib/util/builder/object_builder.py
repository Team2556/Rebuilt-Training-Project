from typing import Protocol, TypeVar

T = TypeVar("T", covariant=True)


class ObjectBuilder(Protocol[T]):

    def build(self) -> T: ...
