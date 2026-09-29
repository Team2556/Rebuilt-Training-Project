from typing import Protocol, TypeVar

from .axis3d import Axis3d

Source = TypeVar("Source", covariant=True)


class Axis3dConvertable(Protocol[Source]):

    def to_axis3d(self) -> Axis3d: ...
