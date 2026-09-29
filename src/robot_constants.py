from phoenix6 import CANBus

# from lib.util.field_layout import FieldLayout

# from src.field_constants import CURRENT_FIELD_TYPE, FieldType


class RobotConstants:
    # current_field_type: FieldType = CURRENT_FIELD_TYPE
    is_red_alliance: bool = False

    rio: CANBus = CANBus("rio")


# FieldLayout.set_alliance_supplier(lambda: RobotConstants.is_red_alliance)
