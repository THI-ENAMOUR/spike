from core.controller.controller import Controller
from core.model.action.atomic.stabilization_action import StabilizationAction


class StabilizationController(Controller):
    """Controller for stabilization of the robot."""

    def execute_action(self, action: StabilizationAction):
        raise NotImplementedError
