from phoenix6.controls import (
    MotionMagicExpoVoltage,
    MotionMagicVelocityVoltage,
    PositionVoltage,
)


class ControlRequestUtil:

    @staticmethod
    def get_slot(request: object) -> int:
        try:
            if isinstance(
                request, (MotionMagicVelocityVoltage, MotionMagicExpoVoltage, PositionVoltage)
            ):
                return request.slot
        except Exception:
            pass

        # if none or error return -1
        return -1
