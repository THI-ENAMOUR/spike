from src.api.model.api_action import ApiAction
from src.core.model.action.high_level.generic.action_group import ActionGroup


class ActionFactory:
    @staticmethod
    def to_actions(actions: "list[ApiAction]") -> ActionGroup:
        """Creates an action group from a list of api actions while validating the content."""
        # TODO: Implement
        return ActionGroup()
