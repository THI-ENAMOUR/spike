from core.controller.controller import Controller

from core.model.action.action_type import ActionType
from core.model.action.atomic.atomic_action import AtomicAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.timing_option import TimingOption, StartTime, Duration
from core.model.common.vector3 import Vector3


class PoseAction(AtomicAction):
    def __init__(
        self,
        start_ms: int,
        end_ms: int = None,
        timing_option: TimingOption = None,
        x=0,
        y=0,
        z=0,
        ax=0,
        ay=0,
        az=0,
    ):

        if timing_option is None:
            if end_ms is None:
                timing_option = StartTime(start_ms=start_ms)
            else:
                timing_option = Duration(start_ms=start_ms, end_ms=end_ms)

        super().__init__(
            action_type=ActionType.BODY_MOVEMENT_ACTION,
            timing_option=timing_option,
            execution_method=ExecutionMethod.NO_SAME_TYPE,
        )

        self.linear = Vector3(x, y, z)
        self.angular = Vector3(ax, ay, az)

    def get_controller(self) -> Controller:
        # Local import to break cyclic import chain
        from core.controller.controller_organizer import ControllerOrganizer

        return ControllerOrganizer.pose_controller
