from core.controller.controller import Controller


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    def execute_action(self, action):
        raise NotImplementedError
