from core.model.action.atomic.pose_action import PoseAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.group.action_group import ActionGroup


class SitAction(ActionGroup):
    def __init__(self, execution_method: ExecutionMethod = ExecutionMethod.EXCLUSIVE):
        super().__init__(
            actions=[PoseAction(start_ms=0, end_ms=3000), PoseAction(start_ms=3000, end_ms=6000)],
            execution_method=execution_method,
        )
