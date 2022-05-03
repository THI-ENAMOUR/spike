from core.controller.controller import Controller
from core.model.action.atomic.display_action import DisplayAction


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    def execute_action(self, action: DisplayAction):
        raise NotImplementedError
