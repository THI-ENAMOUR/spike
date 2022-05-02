import abc
import uuid
from typing import List, Optional, TYPE_CHECKING

from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption
from core.model.common.action_duration import ActionDuration
from error.illegal_state_error import IllegalStateError

if TYPE_CHECKING:
    from core.model.action.group.action_group import ActionGroup


class Action(metaclass=abc.ABCMeta):
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

    def in_time_frame(self, time: ActionDuration):
        return self.timing_option.in_time_frame(self, time)

    def get_parent_time(self) -> ActionDuration:
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
