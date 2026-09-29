from wpilib import SmartDashboard

TABLE_KEY = "TunableNumbers"


class TunableNumber:
    """
    Class creating a mutable object which can be mutated dynamically through SmartDashboard
    """

    def __init__(self, smart_dashboard_key: str, default_value: float) -> None:
        """
        Constructs a new Tunable number

        :param smart_dashboard_key: key used in smartDashboard
        :param default_value:
        """
        self._key = (
            TABLE_KEY + "/" + smart_dashboard_key
        )  # Constructs a new TunableNumber with smartDashboardKey
        self._default_value = 0.0  # default value
        self.publish(
            default_value
        )  # Assigns the default value parameter as the default value and publishes the value into
        # SmartDashboard
        # smartDashboard
        self._current_value = default_value  # current value in SmartDashboard
        self._last_current_value = self._current_value  # last checked current value

    def get_default(self) -> float:
        """
        Gets the default value
        """
        return self._default_value

    def publish(self, value_to_set: float) -> None:
        """
        Sets the default value
        Smartdashboard puts that value into network tables with the object's key

        :param value_to_set:
        """
        self._default_value = value_to_set
        SmartDashboard.putNumber(self._key, SmartDashboard.getNumber(self._key, value_to_set))

    def get_as_double(self) -> float:
        """
        Gets the current value on smartdashboard
        """
        return SmartDashboard.getNumber(self._key, 0.0)

    def has_changed(self) -> bool:
        """
        Checks if the current value in smartdashboard has changed
        Returns true is the current value is different from the last current value
        """
        self._current_value = self.get_as_double()
        if self._current_value != self._last_current_value:
            self._last_current_value = self._current_value
            return True
        return False
