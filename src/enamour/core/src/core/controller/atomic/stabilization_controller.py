from core.controller.controller import Controller


class StabilizationController(Controller):
    """Controller for stabilization of the robot."""

    def execute_action(self, action):
        raise NotImplementedError
