from core.controller.controller import Controller


class LambdaController(Controller):
    """Controller to execute a provided function by a LambdaAction."""

    def execute_action(self, action):
        action.function()
