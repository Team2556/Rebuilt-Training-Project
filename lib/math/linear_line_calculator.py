import math


class LinearLineCalculator:

    def __init__(self, slope: float, intercept: float) -> None:
        self._slope = slope
        self._intercept = intercept

    def get_slope(self) -> float:
        return self._slope

    def get_intercept(self) -> float:
        return self._intercept

    def calculate(self, input: float) -> float:
        if math.isnan(input):
            return math.nan
        return self._slope * input + self._intercept

    @staticmethod
    def best_fit(data: list[tuple[float, float]]) -> "LinearLineCalculator":
        n = len(data)
        sum_x = 0.0
        sum_y = 0.0
        sum_xy = 0.0
        sum_x2 = 0.0

        for x, y in data:
            sum_x += x
            sum_y += y
            sum_xy += x * y
            sum_x2 += x * x

        mean_x = sum_x / n
        mean_y = sum_y / n

        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
        intercept = mean_y - slope * mean_x

        return LinearLineCalculator(slope, intercept)
