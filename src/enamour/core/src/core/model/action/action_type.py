from enum import Enum


class ActionType(Enum):
    STABILIZATION_ACTION = "STABILIZATION_ACTION"
    MOVEMENT_ACTION = "MOVEMENT_ACTION"
    NO_OP = "NO_OP"
    DISPLAY_ACTION = "DISPLAY_ACTION"
    SOUND_ACTION = "SOUND_ACTION"
    LAMBDA_ACTION = "LAMBDA_ACTION"
