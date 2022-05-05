from core.controller.controller import Controller
from core.model.action.atomic.sound_action import SoundAction


class SoundController(Controller):
    """Controller for sound output."""

    def execute_action(self, action: SoundAction):
        raise NotImplementedError
