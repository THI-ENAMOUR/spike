import abc

from src.core.model.action.action import Action
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.timing_option import TimingOption


class HighLevelAction(Action, metaclass=abc.ABCMeta):
    def __init__(self, timing_option: TimingOption, execution_method: ExecutionMethod = ExecutionMethod.SOLO):
        super(HighLevelAction, self).__init__(timing_option=timing_option, execution_method=execution_method)
