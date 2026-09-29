from wpimath import units
from wpiutil import Sendable, SendableBuilder

from .unit_types import TimeUnit, VoltageUnit


class BatteryCheckerInputs(Sendable):

    def __init__(
        self,
        voltage_unit: VoltageUnit,
        time_unit: TimeUnit,
        current_voltage: units.volts = 0.0,
        warning_threshold: units.volts = 0.0,
        emergency_threshold: units.volts = 0.0,
        at_warning_threshold_time: units.seconds = 0.0,
        at_emergency_threshold_time: units.seconds = 0.0,
    ) -> None:
        super().__init__()
        self.update(
            current_voltage,
            warning_threshold,
            emergency_threshold,
            at_warning_threshold_time,
            at_emergency_threshold_time,
        )
        self.voltage_unit = voltage_unit
        self.time_unit = time_unit

    def update(
        self,
        current_voltage: units.volts,
        warning_threshold: units.volts,
        emergency_threshold: units.volts,
        at_warning_threshold_time: units.seconds,
        at_emergency_threshold_time: units.seconds,
    ) -> "BatteryCheckerInputs":
        self.current_voltage = current_voltage
        self.warning_threshold = warning_threshold
        self.emergency_threshold = emergency_threshold
        self.at_warning_threshold_time = at_warning_threshold_time
        self.at_emergency_threshold_time = at_emergency_threshold_time
        return self

    def reset(self) -> "BatteryCheckerInputs":
        return self.update(0.0, 0.0, 0.0, 0.0, 0.0)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty(
            "Supplied Voltage " + self.voltage_unit.name(),
            lambda: self.voltage_unit.convert(self.current_voltage),
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Elapsed/Warning " + self.time_unit.name(),
            lambda: self.time_unit.convert(self.at_warning_threshold_time),
            lambda v: None,
        )
        builder.addDoubleProperty(
            "Elapsed/Emergency " + self.time_unit.name(),
            lambda: self.time_unit.convert(self.at_emergency_threshold_time),
            lambda v: None,
        )
