import abc
import uuid
from typing import List

from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.selection_type import SelectionType
from src.util.action_duration import ActionDuration


class Action(metaclass=abc.ABCMeta):
    def __init__(self, selection_type: SelectionType, execution_method: ExecutionMethod):
        self.id = uuid.uuid4()
        self.completed = False
        self.selection_type = selection_type
        self.execution_method = execution_method

    def complete(self):
        self.completed = True

    def is_selected(self, time: ActionDuration):
        return self.selection_type.is_selected_action(self, time)


ActionList = List[Action]
