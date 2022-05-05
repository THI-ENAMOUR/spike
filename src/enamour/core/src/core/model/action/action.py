import abc
import uuid
from typing import List, Optional, TYPE_CHECKING

from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption
from core.model.common.time_stamp import TimeStamp
from error.illegal_state_error import IllegalStateError

if TYPE_CHECKING:
    from core.model.action.group.action_group import ActionGroup


class Action(metaclass=abc.ABCMeta):
    """Most generic, abstract action. Containing information all actions share among themselves."""

    def __init__(
        self, timing_option: TimingOption, execution_method: ExecutionMethod, parent: Optional["ActionGroup"] = None
    ):
        self.id = uuid.uuid4()
        self.completed = False
        self.timing_option = timing_option
        self.execution_method = execution_method
        self.parent = parent

    def complete(self):
        self.completed = True

    def in_time_frame(self, time_stamp: TimeStamp) -> bool:
        """Returns true if the time stamp is within the timing option of the action is"""
        return self.timing_option.in_time_frame(self, time_stamp)

    def get_parent_time(self) -> TimeStamp:
        if self.parent is not None:
            return self.parent.time
        raise IllegalStateError("Could not get parent time, since parent is none.")

    def __str__(self):
        return (
            f"{self.__class__.__name__}(id: {self.id}, "
            f"timing: {self.timing_option}, "
            f"execution: {self.execution_method.value})"
        )


ActionList = List[Action]
