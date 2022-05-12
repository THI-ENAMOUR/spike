from core.controller.controller import Controller


class SoundController(Controller):
    """Controller for sound output."""

    def execute_action(self, action):
        raise NotImplementedError
