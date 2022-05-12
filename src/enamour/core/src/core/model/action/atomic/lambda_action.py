from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime


class LambdaAction(AtomicAction):
    def __init__(self, start_ms, function, timing_option=None):
        timing_option = timing_option if timing_option is not None else StartTime(start_ms=start_ms)
        super(LambdaAction, self).__init__(
            action_type=ActionType.LAMBDA_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.MULTIPLE,
        )
        self.function = function

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.lambda_controller
