import math

from wpilib import Timer

from ..util import Util


class SynchronousPIDF:

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0, kf: float = 0.0) -> None:
        """
        Allocate a PID object with the given constants for P, I, D

        :param kp: the proportional coefficient
        :param ki: the integral coefficient
        :param kd: the derivative coefficient
        :param kf: the feed forward gain coefficient
        """
        self._p = 0.0  # factor for "proportional" control
        self._i = 0.0  # factor for "integral" control
        self._d = 0.0  # factor for "derivative" control
        self._f = 0.0  # factor for feed forward gain
        self._maximum_output = 1.0  # |maximum output|
        self._minimum_output = -1.0  # |minimum output|
        self._maximum_input = 0.0  # maximum input - limit setpoint to this
        self._minimum_input = 0.0  # minimum input - limit setpoint to this
        self._continuous = False  # do the endpoints wrap around? eg. absolute encoder
        self._prev_error = 0.0  # the prior sensor input (used to compute velocity)
        self._total_error = 0.0  # the sum of the errors for use in the integral calc
        self._setpoint = 0.0
        self._error = 0.0
        self._result = 0.0
        self._last_input = math.nan
        self._deadband = (
            0.0  # If the absolute error is less than deadband then treat error for the proportional term as 0
        )
        self._last_timestamp = Timer.getFPGATimestamp()
        self._tolerance = 0.0

        self.set_pidf(kp, ki, kd, kf)

    def calculate(self, input: float, dt: float | None = None) -> float:
        """
        Read the input, calculate the output accordingly, and write to the output.
        This should be called at a constant rate by the user (ex. in a timed thread)

        :param input: the input
        :param dt:    time passed since previous call to calculate
        """
        if dt is None:
            timestamp = Timer.getFPGATimestamp()
            dt = timestamp - self._last_timestamp
            self._last_timestamp = timestamp

        if dt < 1e-6:
            dt = 1e-6

        self._last_input = input
        self._error = self._setpoint - input
        if self._continuous:
            if abs(self._error) > (self._maximum_input - self._minimum_input) / 2:
                if self._error > 0:
                    self._error = self._error - self._maximum_input + self._minimum_input
                else:
                    self._error = self._error + self._maximum_input - self._minimum_input

        if Util.in_range(self._error * self._p, self._minimum_output, self._maximum_output):
            self._total_error += self._error * dt
        else:
            self._total_error = 0

        # Don't blow away m_error so as to not break derivative
        proportional_error = 0 if abs(self._error) < self._deadband else self._error

        self._result = (
            self._p * proportional_error
            + self._i * self._total_error
            + self._d * (self._error - self._prev_error) / dt
            + self._f * self._setpoint
        )
        self._prev_error = self._error

        return Util.limit(self._result, self._minimum_output, self._maximum_output)

    def set_pid(self, p: float, i: float, d: float) -> None:
        """
        Set the PID controller gain parameters. Set the proportional, integral, and
        differential coefficients.

        :param p: Proportional coefficient
        :param i: Integral coefficient
        :param d: Differential coefficient
        """
        self._p = p
        self._i = i
        self._d = d

    def set_p(self, p: float) -> None:
        self._p = p

    def set_i(self, i: float) -> None:
        self._i = i

    def set_d(self, d: float) -> None:
        self._d = d

    def set_f(self, f: float) -> None:
        self._f = f

    def set_pidf(self, p: float, i: float, d: float, f: float) -> None:
        self._p = p
        self._i = i
        self._d = d
        self._f = f

    def get_p(self) -> float:
        return self._p

    def get_i(self) -> float:
        return self._i

    def get_d(self) -> float:
        return self._d

    def get_f(self) -> float:
        return self._f

    def get(self) -> float:
        """
        :returns: the latest calculated output
        """
        return self._result

    def set_continuous(self, continuous: bool = True) -> None:
        """
        Set the PID controller to consider the input to be continuous, Rather then
        using the max and min in as constraints, it considers them to be the same
        point and automatically calculates the shortest route to the setpoint.

        :param continuous: Set to true turns on continuous, false turns off continuous
        """
        self._continuous = continuous

    def set_deadband(self, deadband: float) -> None:
        self._deadband = deadband

    def set_input_range(self, minimum_input: float, maximum_input: float) -> None:
        """
        Sets the maximum and minimum values expected from the input.

        :param minimum_input: the minimum value expected from the input
        :param maximum_input: the maximum value expected from the output
        """
        if minimum_input > maximum_input:
            raise ValueError("Lower bound is greater than upper bound")
        self._minimum_input = minimum_input
        self._maximum_input = maximum_input
        self.set_setpoint(self._setpoint)

    def set_output_range(self, minimum_output: float, maximum_output: float) -> None:
        """
        Sets the minimum and maximum values to write.

        :param minimum_output: the minimum value to write to the output
        :param maximum_output: the maximum value to write to the output
        """
        if minimum_output > maximum_output:
            raise ValueError("Lower bound is greater than upper bound")
        self._minimum_output = minimum_output
        self._maximum_output = maximum_output

    def set_max_absolute_output(self, max_absolute_output: float) -> None:
        self.set_output_range(-max_absolute_output, max_absolute_output)

    def set_setpoint(self, setpoint: float) -> None:
        """
        Set the setpoint for the PID controller

        :param setpoint: the desired setpoint
        """
        if self._maximum_input > self._minimum_input:
            if setpoint > self._maximum_input:
                self._setpoint = self._maximum_input
            elif setpoint < self._minimum_input:
                self._setpoint = self._minimum_input
            else:
                self._setpoint = setpoint
        else:
            self._setpoint = setpoint

    def set_setpoint_and_calculate(self, setpoint: float, input: float) -> float:
        self.set_setpoint(setpoint)
        return self.calculate(input)

    def set_tolerance(self, tolerance: float) -> None:
        self._tolerance = tolerance

    def get_setpoint(self) -> float:
        """
        Returns the current setpoint of the PID controller

        :returns: the current setpoint
        """
        return self._setpoint

    def get_error(self) -> float:
        """
        Returns the current difference of the input from the setpoint

        :returns: the current error
        """
        return self._error

    def on_target(self, tolerance: float | None = None) -> bool:
        """
        Return true if the error is within the tolerance

        :returns: true if the error is less than the tolerance
        """
        if tolerance is None:
            tolerance = self._tolerance
        return not math.isnan(self._last_input) and abs(self._last_input - self._setpoint) < tolerance

    def reset(self) -> None:
        """
        Reset all internal terms.
        """
        self._last_input = math.nan
        self._prev_error = 0
        self._total_error = 0
        self._result = 0
        self._setpoint = 0

    def reset_integrator(self) -> None:
        self._total_error = 0

    def get_state(self) -> str:
        l_state = ""

        l_state += "Kp: " + str(self._p) + "\n"
        l_state += "Ki: " + str(self._i) + "\n"
        l_state += "Kd: " + str(self._d) + "\n"

        return l_state

    def get_type(self) -> str:
        return "PIDController"
