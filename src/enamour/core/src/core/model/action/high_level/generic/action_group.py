import abc

from src.core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from src.core.model.action.high_level.generic.high_level_action import HighLevelAction
from src.core.model.action.timing_option import TimingOption
from src.util.action_duration import ActionDuration


class ActionGroup(HighLevelAction, metaclass=abc.ABCMeta):
    """Used to recursively wrap actions within each other."""

    def __init__(self, timing_option: TimingOption = TimingOption.Always()):
        super().__init__(timing_option)

    @abc.abstractmethod
    def get_next_actions(self, time: ActionDuration) -> (ActionExecutionList, ActionDuration):
        raise NotImplementedError
