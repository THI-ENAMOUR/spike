from core.controller.controller import Controller
from core.model.action.atomic.atomic_action import AtomicAction


class NoOpController(Controller):
    """Controller for NoOpActions. Does nothing."""

    def execute_action(self, action: AtomicAction):
        pass
