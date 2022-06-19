from core.controller.controller import Controller


class HeadController(Controller):
    """Controller for stabilization of the robot."""

    def execute_action(self, action):
        raise NotImplementedError
