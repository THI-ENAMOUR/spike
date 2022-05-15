import uuid

from api.model.api_action import ApiAction
from core.model.action.group.action_group import ActionGroup
from util.json_mapper import get_default, to_UUID


class ApiActionRequest:
    """Represents the structure of a new action request."""

    def __init__(self, id=None, clear_action_queue=False, actions=None):
        if actions is None:
            actions = []

        self.id = id if id is not None else uuid.uuid4()
        self.clear_action_queue = clear_action_queue
        self.actions = actions

    def to_action_group(self):
        actions = []
        for action in self.actions:
            actions.append(action.to_action_group())
        return ActionGroup(id=self.id, actions=actions, start_ms=0)

    @staticmethod
    def from_json(data):
        # TODO: Implement correctly and in a nice, readable way including error handling
        id = get_default(data, "id", uuid.uuid4())
        id = to_UUID(id)
        clear_action_queue = get_default(data, "clear_action_queue", False)
        actions_json = get_default(data, "actions", [])

        actions = []
        for action in actions_json:
            actions.append(ApiAction.from_json(action))

        return ApiActionRequest(id=id, clear_action_queue=clear_action_queue, actions=actions)
