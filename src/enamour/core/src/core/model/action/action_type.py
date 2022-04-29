from enum import Enum


class ActionType(Enum):
    BODY_MOVEMENT_ACTION = "BODY_MOVEMENT_ACTION"
    NO_OP = "NO_OP"

    # TODO: Add here

    def is_movement_action(self):
        return self == ActionType.BODY_MOVEMENT_ACTION
