from core.controller.controller import Controller
from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption, StartTime


class NavigationAction(AtomicAction):
    def __init__(self, start_ms: int, timing_option: TimingOption = None):
        timing_option = timing_option if timing_option is not None else StartTime(start_ms=start_ms)
        super(NavigationAction, self).__init__(
            action_type=ActionType.BODY_MOVEMENT_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.NO_SAME_TYPE,
        )

    def get_controller(self) -> Controller:
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.navigation_controller
