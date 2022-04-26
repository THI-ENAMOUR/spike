from src.controller.controller import Controller
from src.core.model.action.atomic.display_action import DisplayAction
from src.util.action_duration import ActionDuration


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    def execute_action(self, action: DisplayAction, parent_duration: ActionDuration):
        pass
