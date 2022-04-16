import abc

from src.core.model.action.action import Action
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.selection_type import SelectionType


class HighLevelAction(Action, metaclass=abc.ABCMeta):
    def __init__(self, selection_type: SelectionType, execution_method: ExecutionMethod = ExecutionMethod.SOLO):
        super(HighLevelAction, self).__init__(selection_type=selection_type, execution_method=execution_method)
