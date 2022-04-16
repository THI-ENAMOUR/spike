from src.controller.controller import Controller
from src.core.model.action.atomic.generic.atomic_action import AtomicAction
from src.util.action_duration import ActionDuration


class NoOpController(Controller):
    """Controller for NoOpActions. Does nothing."""

    def execute_action(self, action: AtomicAction, parent_duration: ActionDuration):
        pass
