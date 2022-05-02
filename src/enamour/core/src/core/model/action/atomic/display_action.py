from core.controller.controller import Controller
from core.model.action.timing_option import TimingOption, StartTime
from core.model.common.action_duration import ActionDuration
from src.core.model.action.action_type import ActionType
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.execution_method import ExecutionMethod


class DisplayAction(AtomicAction):
    def __init__(self, start_ms: int, timing_option: TimingOption = None):
        start_time = ActionDuration(ms=start_ms)
        timing_option = timing_option if timing_option is not None else StartTime(start_time=start_time)
        super(DisplayAction, self).__init__(
            action_type=ActionType.DISPLAY_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.MULTIPLE,
        )

    def get_controller(self) -> Controller:
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.display_controller
