import abc

from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.high_level.generic.high_level_action import HighLevelAction
from src.core.model.action.selection_type import SelectionType, SelectAlways
from src.util.action_duration import ActionDuration


class ActionGroup(HighLevelAction, metaclass=abc.ABCMeta):
    """Used to recursively wrap actions within each other."""

    def __init__(self, selection_type: SelectionType = SelectAlways()):
        super().__init__(selection_type)

    @abc.abstractmethod
    def get_next_actions(self, time: ActionDuration) -> (ActionExecutionList, ActionDuration):
        raise NotImplementedError
