from enum import Enum


class ExecutionMethod(Enum):
    EXCLUSIVE = "EXCLUSIVE"
    """The action can only be executed alone. No other action should be executed at the same time frame."""

    MULTIPLE = "MULTIPLE"
    """The action can be executed simultaneously to multiple other actions."""

    NO_SAME_TYPE = "NO_SAME_TYPE"
    """ The action is exclusively execution for the same action type.
        No other action with the same type can be executed"""
