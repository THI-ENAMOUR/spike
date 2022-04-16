from src.core.model.action.atomic.pose_action import PoseAction
from src.core.model.action.high_level.default_action_group import DefaultActionGroup
from src.util.action_duration import ActionDuration


class SitAction(DefaultActionGroup):
    def __init__(self):
        super().__init__(actions=[PoseAction(0, 0, 0, 0, 0, 0)])

    def is_selected(self, time: ActionDuration) -> bool:
        pass
