from core.controller.controller import Controller


class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action):
        raise NotImplementedError
