from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime, Duration


class SoundAction(AtomicAction):
    def __init__(self, start_ms, name, timing_option=None, end_ms=None):
        if timing_option is None:
            if end_ms is not None:
                timing_option = Duration(start_ms=start_ms, end_ms=end_ms)
            else:
                timing_option = StartTime(start_ms=start_ms)

        super(SoundAction, self).__init__(
            action_type=ActionType.SOUND_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.MULTIPLE,
        )
        self.name = name

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.sound_controller
