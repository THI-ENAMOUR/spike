from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime, Duration


class NoOpAction(AtomicAction):
    def __init__(self, start_ms=0, end_ms=None, timing_option=None, parent=None):

        if timing_option is None:
            if end_ms is None:
                timing_option = StartTime(start_ms=start_ms)
            else:
                timing_option = Duration(start_ms=start_ms, end_ms=end_ms)

        super(NoOpAction, self).__init__(
            action_type=ActionType.NO_OP_ACTION, timing_option=timing_option, execution_method=ExecutionMethod.MULTIPLE
        )

        self.parent = parent

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.no_op_controller
