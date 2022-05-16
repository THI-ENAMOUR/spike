from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime


class DisplayAction(AtomicAction):
    def __init__(self, start_ms, display_name, timing_option=None):
        timing_option = timing_option if timing_option is not None else StartTime(start_ms=start_ms)
        super(DisplayAction, self).__init__(
            action_type=ActionType.DISPLAY_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.MULTIPLE,
        )
        self.name = display_name

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.display_controller
