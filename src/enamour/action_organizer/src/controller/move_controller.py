from src.controller.controller import Controller
from enamour.action_organizer.src.core.model.action.atomic.move_action import MoveAction
from src.util.action_duration import ActionDuration


class MoveController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action: MoveAction, parent_duration: ActionDuration):
        pass
