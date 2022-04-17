from src.controller.controller import Controller
from src.core.model.action.atomic.pose_action import PoseAction
from src.util.action_duration import ActionDuration


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""

    def execute_action(self, action: PoseAction, parent_duration: ActionDuration):
        if not isinstance(action, PoseAction):
            raise ValueError

        self.execute_pose_action(action)

    def execute_pose_action(self, action: PoseAction):
        pass
