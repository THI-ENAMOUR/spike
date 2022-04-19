import abc
import uuid
from typing import List

from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.timing_option import TimingOption
from src.util.action_duration import ActionDuration


class Action(metaclass=abc.ABCMeta):
    def __init__(self, timing_option: TimingOption, execution_method: ExecutionMethod):
        self.id = uuid.uuid4()
        self.completed = False
        self.timing_option = timing_option
        self.execution_method = execution_method

    def complete(self):
        self.completed = True

    def is_selected(self, time: ActionDuration):
        return self.timing_option.is_selected_action(self, time)

    def __str__(self):
        return (
            f"{self.__class__.__name__}(id: {self.id}, "
            f"timing: {self.timing_option}, "
            f"execution: {self.execution_method.value})"
        )


ActionList = List[Action]
