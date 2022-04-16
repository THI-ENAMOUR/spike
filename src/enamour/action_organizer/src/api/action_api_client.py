import json

from src.api.action_factory import ActionFactory
from src.api.model.api_action_request import ApiActionRequest
from src.core.action_queue import ActionQueue


# TODO: Adjust class once we know how the api looks like
class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    def __init__(self, action_queue: ActionQueue, action_factory: ActionFactory):
        self.action_queue = action_queue
        self.action_factory = action_factory

    # TODO: Remove json parameter later, when we receive the data from the outside (topic etc.)
    # TODO: Return errors to client, once we know which technology is used for communication.
    def receive_action_and_add_to_queue(self, json_string: str):
        request = self.receive_action_request(json_string)
        actions = self.action_factory.to_actions(request.actions)
        self.action_queue.push(actions)

    @staticmethod
    def receive_action_request(json_string: str) -> ApiActionRequest:
        """Receives an action request in json format and deserializes it"""

        # TODO: Replace the parsing of json with a library. Tried it, but the libraries don't support the type safety
        # TODO: we would like to have. -> Search more.
        data = json.loads(json_string)
        return ApiActionRequest.from_json(data)
