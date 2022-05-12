from core.controller.controller import Controller


class NoOpController(Controller):
    """Controller for NoOpActions. Does nothing."""

    def execute_action(self, action):
        pass
