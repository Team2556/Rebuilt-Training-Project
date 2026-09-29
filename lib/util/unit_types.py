import math
from dataclasses import dataclass

from wpimath import units


@dataclass(frozen=True)
class AngleUnit:
    _name: str
    _symbol: str
    radians_per_unit: float

    def name(self) -> str:
        return self._name

    def symbol(self) -> str:
        return self._symbol

    def of(self, magnitude: float) -> units.radians:
        return magnitude * self.radians_per_unit

    def convert(self, base: units.radians) -> float:
        return base / self.radians_per_unit

    def per(self, time: "TimeUnit") -> "AngularVelocityUnit":
        return AngularVelocityUnit(self, time)


@dataclass(frozen=True)
class DistanceUnit:
    _name: str
    _symbol: str
    meters_per_unit: float

    def name(self) -> str:
        return self._name

    def symbol(self) -> str:
        return self._symbol

    def of(self, magnitude: float) -> units.meters:
        return magnitude * self.meters_per_unit

    def convert(self, base: units.meters) -> float:
        return base / self.meters_per_unit

    def per(self, time: "TimeUnit") -> "LinearVelocityUnit":
        return LinearVelocityUnit(self, time)


@dataclass(frozen=True)
class TimeUnit:
    _name: str
    _symbol: str
    seconds_per_unit: float

    def name(self) -> str:
        return self._name

    def symbol(self) -> str:
        return self._symbol

    def of(self, magnitude: float) -> units.seconds:
        return magnitude * self.seconds_per_unit

    def convert(self, base: units.seconds) -> float:
        return base / self.seconds_per_unit


@dataclass(frozen=True)
class VoltageUnit:
    _name: str
    _symbol: str
    volts_per_unit: float

    def name(self) -> str:
        return self._name

    def symbol(self) -> str:
        return self._symbol

    def of(self, magnitude: float) -> units.volts:
        return magnitude * self.volts_per_unit

    def convert(self, base: units.volts) -> float:
        return base / self.volts_per_unit


@dataclass(frozen=True)
class AngularVelocityUnit:
    angle: AngleUnit
    time: TimeUnit

    def name(self) -> str:
        return f"{self.angle.name()} per {self.time.name()}"

    def of(self, magnitude: float) -> units.radians_per_second:
        return magnitude * self.angle.radians_per_unit / self.time.seconds_per_unit

    def convert(self, base: units.radians_per_second) -> float:
        return base / (self.angle.radians_per_unit / self.time.seconds_per_unit)

    def per(self, time: TimeUnit) -> "AngularAccelerationUnit":
        return AngularAccelerationUnit(self, time)


@dataclass(frozen=True)
class AngularAccelerationUnit:
    velocity: AngularVelocityUnit
    time: TimeUnit

    def name(self) -> str:
        return f"{self.velocity.name()} per {self.time.name()}"
    
    def of(self, magnitude: float) -> units.radians_per_second_squared:
        return  self.velocity.of(magnitude) / self.time.seconds_per_unit

    def convert(self, base: float) -> float:
        return base / (
            self.velocity.angle.radians_per_unit
            / self.velocity.time.seconds_per_unit
            / self.time.seconds_per_unit
        )


@dataclass(frozen=True)
class LinearVelocityUnit:
    distance: DistanceUnit
    time: TimeUnit

    def name(self) -> str:
        return f"{self.distance.name()} per {self.time.name()}"

    def of(self, magnitude: float) -> units.meters_per_second:
        return magnitude * self.distance.meters_per_unit / self.time.seconds_per_unit

    def convert(self, base: units.meters_per_second) -> float:
        return base / (self.distance.meters_per_unit / self.time.seconds_per_unit)

    def per(self, time: TimeUnit) -> "LinearAccelerationUnit":
        return LinearAccelerationUnit(self, time)


@dataclass(frozen=True)
class LinearAccelerationUnit:
    velocity: LinearVelocityUnit
    time: TimeUnit

    def of(self, magnitude: float) -> units.meters_per_second_squared:
        return  self.velocity.of(magnitude) / self.time.seconds_per_unit

    def name(self) -> str:
        return f"{self.velocity.name()} per {self.time.name()}"

    def convert(self, base: float) -> float:
        return base / (
            self.velocity.distance.meters_per_unit
            / self.velocity.time.seconds_per_unit
            / self.time.seconds_per_unit
        )


Radians = AngleUnit("Radians", "rad", 1.0)
Rotations = AngleUnit("Rotations", "rot", math.tau)
Degrees = AngleUnit("Degrees", "deg", math.pi / 180.0)

Meters = DistanceUnit("Meters", "m", 1.0)
Centimeters = DistanceUnit("Centimeters", "cm", 1e-2)
Inches = DistanceUnit("Inches", "in", 0.0254)
Feet = DistanceUnit("Feet", "ft", 0.3048)

Volts = VoltageUnit("Volts", "V", 1.0)
Millivolts = VoltageUnit("Millivolts", "mV", 1e-3)

Second = TimeUnit("Second", "s", 1.0)
Seconds = Second
Minute = TimeUnit("Minute", "min", 60.0)
Minutes = Minute
Millisecond = TimeUnit("Millisecond", "ms", 1e-3)

RPM = Rotations.per(Minute)

MetersPerSecond = Meters.per(Second)
MetersPerSecondSquared = MetersPerSecond.per(Seconds)
FeetPerSecond = Feet.per(Second)

LinearVelocity = MetersPerSecond
LinearAcceleration = MetersPerSecondSquared

RadiansPerSecond = Radians.per(Seconds)
RadiansPerSecondSquared = RadiansPerSecond.per(Seconds)

AngularVelocity = RadiansPerSecond
AngularAcceleration = RadiansPerSecondSquared