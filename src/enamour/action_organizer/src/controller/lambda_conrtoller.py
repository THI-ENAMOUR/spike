from src.controller.controller import Controller
from src.core.model.action.atomic.lambda_action import LambdaAction
from src.util.action_duration import ActionDuration


class LambdaController(Controller):
    """Controller for LambdaAction."""

    def execute_action(self, action: LambdaAction, parent_duration: ActionDuration):
        pass
