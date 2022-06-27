from core.model.action.atomic.pose_action import PoseAction
from core.model.action.execution_method import ExecutionMethod
from core.model.action.group.action_group import ActionGroup


class SitAction(ActionGroup):
    def __init__(self, start_ms, execution_method=ExecutionMethod.EXCLUSIVE):
        super(SitAction, self).__init__(
            start_ms=start_ms,
            actions=[PoseAction(start_ms=0, end_ms=800, pitch=-1.8, body_height=-1.0),
                     PoseAction(start_ms=6000, end_ms=8000, pitch=0.0, body_height=0.0)],
            execution_method=execution_method,
        )
