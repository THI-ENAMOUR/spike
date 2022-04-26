from src.controller.controller import Controller
from src.core.model.action.atomic.sound_action import SoundAction
from src.util.action_duration import ActionDuration


class SoundController(Controller):
    """Controller for sound output."""

    def execute_action(self, action: SoundAction, parent_duration: ActionDuration):
        pass
