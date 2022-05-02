import abc
from typing import Dict
from uuid import UUID

from core.model.action.action import Action, ActionList
from core.model.action.atomic.generic.action_execution_list import ActionExecutionList
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption
from core.model.common.action_duration import ActionDuration


class ActionGroup(Action, metaclass=abc.ABCMeta):
    """Used to recursively wrap actions within each other. The actions are sorted by starting time."""

    def __init__(
        self,
        actions: ActionList,
        start_time_ms: int,
        timing_option: TimingOption = None,
        execution_method: ExecutionMethod = ExecutionMethod.SOLO,
    ):
        if timing_option is None:
            timing_option = TimingOption.StartTime(start_time=ActionDuration(ms=start_time_ms))

        super(ActionGroup, self).__init__(timing_option, execution_method)

        self.time = ActionDuration(ns=0)
        self.actions = actions
        # IMPORTANT: Sort the action list, so we can make assumptions based on the sorting of the start time
        self.sort()

    def sort(self):
        TimingOption.sort(self.actions)

    @abc.abstractmethod
    def update_parent_time_of_executed_actions(self, delta_time: ActionDuration):
        raise NotImplementedError

    @abc.abstractmethod
    def get_next_actions(self) -> (ActionExecutionList, Dict[UUID, "ActionGroup"]):
        raise NotImplementedError
