from src.api.model.api_action import ApiAction


# TODO: Generate the api model from the openapi.yaml? Tried it, it works, but we prop want to edit them, therefore
# TODO: auto-generation does not make sense. Find another solution? Or just live with the manual editing labor.


class ApiActionRequest:
    """Represents the structure of a new action request."""

    def __init__(self, clear_action_queue: bool = False, actions: "list[ApiAction]" = None):
        if actions is None:
            actions = []

        self.clear_action_queue = clear_action_queue
        self.actions = actions

    # TODO: Error Handling, especially as info for client
    @staticmethod
    def from_json(data: dict):
        actions: "list[ApiAction]" = []
        for action in data["actions"]:
            actions.append(ApiAction.from_json(action))

        return ApiActionRequest(clear_action_queue=data["clearActionQueue"], actions=actions)
