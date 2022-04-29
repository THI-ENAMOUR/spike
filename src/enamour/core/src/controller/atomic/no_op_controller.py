from src.controller.controller import Controller
from src.core.model.action.atomic.generic.atomic_action import AtomicAction


class NoOpController(Controller):
    """Controller for NoOpActions. Does nothing."""

    def execute_action(self, action: AtomicAction):
        pass
