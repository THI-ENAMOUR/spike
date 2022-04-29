from src.controller import controller_provider
from src.controller.controller import Controller
from src.core.model.action.action_type import ActionType
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.timing_option import TimingOption
from src.util.action_duration import ActionDuration


class NoOpAction(AtomicAction):
    def __init__(
        self, start_time_ms=0, end_time_ms=None, start_time: ActionDuration = None, timing_option: TimingOption = None
    ):

        if start_time is None:
            start_time = (ActionDuration(ms=start_time_ms),)

        if timing_option is None:
            if end_time_ms is None:
                timing_option = TimingOption.StartTime(start_time)
            else:
                timing_option = TimingOption.Duration(start_time, ActionDuration(ms=end_time_ms))

        super(NoOpAction, self).__init__(
            action_type=ActionType.NO_OP, timing_option=timing_option, execution_method=ExecutionMethod.MULTIPLE
        )

    def get_controller(self) -> Controller:
        return controller_provider.no_op_controller
