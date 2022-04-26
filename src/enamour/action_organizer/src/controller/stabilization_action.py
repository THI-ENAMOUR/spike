from src.controller.controller import Controller
from src.core.model.action.atomic.stabilization_action import StabilizationAction
from src.util.action_duration import ActionDuration


class SoundController(Controller):
    """Controller for stabilizing the robot."""

    def execute_action(self, action: StabilizationAction, parent_duration: ActionDuration):
        pass
