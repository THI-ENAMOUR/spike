from core.controller.controller import Controller
from src.core.model.action.atomic.lambda_action import LambdaAction


class LambdaController(Controller):
    """Controller to execute a provided function by a LambdaAction."""

    def execute_action(self, action: LambdaAction):
        action.function()
