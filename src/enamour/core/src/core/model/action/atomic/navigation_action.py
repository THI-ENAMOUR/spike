from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import StartTime, Duration


class NavigationAction(AtomicAction):
    def __init__(self, start_ms, end_ms=None, x=None, y=None, yaw=None, body_height=None, timing_option=None):

        if timing_option is None:
            if end_ms is None:
                timing_option = StartTime(start_ms=start_ms)
            else:
                timing_option = Duration(start_ms=start_ms, end_ms=end_ms)
                self.stopping_time = Duration(start_ms=end_ms - 100, end_ms=end_ms) # needed to stop the robot before exiting the action

        # Zeile darunter ist einfach mal auskommentiert, falls man die wieder brauchen wuerde (ich denke mal, dass wir eien Duration brauchen werden, wenn wir cmd_vel verwenden)
        # timing_option = timing_option if timing_option is not None else StartTime(start_ms=start_ms)
        super(NavigationAction, self).__init__(
            action_type=ActionType.BODY_MOVEMENT_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.NO_SAME_TYPE,
        )

        self.x = x
        self.y = y
        self.yaw = yaw

        if x is None:
            self.x = 0

        if y is None:
            self.y = 0

        if yaw is None:
            self.yaw = 0

        if body_height is None:
            self.body_height = 0

    def get_controller(self):
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.navigation_controller
