from src.controller import controller_provider
from src.controller.controller import Controller
from src.core.model.action.action_type import ActionType
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.timing_option import TimingOption


class NoOpAction(AtomicAction):
    def __init__(self, timing_option: TimingOption = TimingOption.Always()):
        super(NoOpAction, self).__init__(
            action_type=ActionType.NO_OP, timing_option=timing_option, execution_method=ExecutionMethod.MULTIPLE
        )

    def get_controller(self) -> Controller:
        return controller_provider.no_op_controller
