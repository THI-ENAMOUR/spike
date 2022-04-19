from src.controller import controller_provider
from src.controller.controller import Controller
from src.core.model.action.action_type import ActionType
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.core.model.action.execution_method import ExecutionMethod
from src.core.model.action.timing_option import TimingOption
from src.core.model.common.vector3 import Vector3


class PoseAction(AtomicAction):
    def __init__(self, x=0, y=0, z=0, ax=0, ay=0, az=0, start_ms=0, end_ms=3000):
        super().__init__(
            action_type=ActionType.BODY_MOVEMENT_ACTION,
            timing_option=TimingOption.Duration.from_ms(start_ms, end_ms),
            execution_method=ExecutionMethod.NO_SAME_TYPE,
        )

        self.linear = Vector3(x, y, z)
        self.angular = Vector3(ax, ay, az)

    def get_controller(self) -> Controller:
        return controller_provider.movement_controller
