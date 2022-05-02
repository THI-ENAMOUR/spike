from core.controller.controller import Controller
from core.model.action.atomic.navigation_action import NavigationAction


class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action: NavigationAction):
        raise NotImplementedError
