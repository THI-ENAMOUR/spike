from api.model.api_action import ApiAction
from core.model.action.group.action_group import ActionGroup


class ApiActionRequest:
    """Represents the structure of a new action request."""

    def __init__(self, clear_action_queue=False, actions=None):
        if actions is None:
            actions = []

        self.clear_action_queue = clear_action_queue
        self.actions = actions

    def to_action_group(self):
        # TODO: Create mapping
        from core.model.action.group.sit_action import SitAction

        return ActionGroup(actions=[SitAction()], start_ms=0)

    @staticmethod
    def from_json(data):
        # TODO: Implement correctly and in a nice, readable way including error handling

        actions = []
        for action in data["actions"]:
            actions.append(ApiAction.from_json(action))

        return ApiActionRequest(clear_action_queue=data["clearActionQueue"], actions=actions)
