from typing import TYPE_CHECKING

from core.controller.controller import Controller
from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption, StartTime, Duration

if TYPE_CHECKING:
    from core.model.action.group.action_group import ActionGroup


class NoOpAction(AtomicAction):
    def __init__(
        self,
        start_time_ms=0,
        end_time_ms=None,
        timing_option: TimingOption = None,
        parent: "ActionGroup" = None,
    ):

        if timing_option is None:
            if end_time_ms is None:
                timing_option = StartTime(start_ms=start_time_ms)
            else:
                timing_option = Duration(start_ms=start_time_ms, end_ms=end_time_ms)

        super(NoOpAction, self).__init__(
            action_type=ActionType.NO_OP_ACTION, timing_option=timing_option, execution_method=ExecutionMethod.MULTIPLE
        )

        self.parent = parent

    def get_controller(self) -> Controller:
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.no_op_controller
