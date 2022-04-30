from typing import TYPE_CHECKING

from src.api.model.api_action import ApiAction

from src.core.model.action.group.action_group import ActionGroup

if TYPE_CHECKING:
    from src.core.model.action.group.sit_action import SitAction


class ApiActionRequest:
    """Represents the structure of a new action request."""

    def __init__(self, clear_action_queue: bool = False, actions: "list[ApiAction]" = None):
        if actions is None:
            actions = []

        self.clear_action_queue = clear_action_queue
        self.actions = actions

    def to_action_group(self) -> ActionGroup:
        # TODO: Create mapping
        return ActionGroup(actions=[SitAction()], start_time_ms=0)

    @staticmethod
    def from_json(data: dict):
        # TODO: Create json deserialization

        actions: "list[ApiAction]" = []
        for action in data["actions"]:
            actions.append(ApiAction.from_json(action))

        return ApiActionRequest(clear_action_queue=data["clearActionQueue"], actions=actions)
