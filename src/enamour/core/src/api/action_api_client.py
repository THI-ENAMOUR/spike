import json

from std_msgs.msg import String

from api.model.api_action_request import ApiActionRequest
from core.action_queue import ActionQueue
from core.event_bus import event_bus
from error.handler.action_api_error_handler import ActionApiErrorHandler
from util.logger import Logger


class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    __logger = Logger(__name__)

    def __init__(
        self, action_queue: ActionQueue, action_api_error_handler: ActionApiErrorHandler = ActionApiErrorHandler()
    ):
        self.action_queue = action_queue
        self.action_api_error_handler = action_api_error_handler
        self.running = False

    def start(self):
        self.running = True
        while event_bus.is_running() and self.running:
            event_bus.subscribe("action", String, self.receive_action)
            event_bus.spin()

    def receive_action(self, action):
        try:
            self.__logger.info("Received a new action request")
            action_request_json = json.loads(action.data)
            action_request = ApiActionRequest.from_json(action_request_json)
            action_group = action_request.to_action_group()
            self.action_queue.push(action_group)
        except (BaseException,) as error:
            self.action_api_error_handler.handle(error)
