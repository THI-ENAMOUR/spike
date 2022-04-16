from src.core.model.action.action import Action
from src.core.model.action.selection_type import SelectDuration
from src.util.action_duration import ActionDuration


def check_completion_before(action: Action, time: ActionDuration) -> bool:
    return isinstance(action.selection_type, SelectDuration) and action.selection_type.end_time < time


def check_completion_after(action: Action) -> bool:
    return not isinstance(action.selection_type, SelectDuration)
