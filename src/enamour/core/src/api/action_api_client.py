import json

from std_msgs.msg import String

from api.model.api_action_request import ApiActionRequest
from core.action_queue import ActionQueue
from core.event_bus import event_bus
from util.logger import Logger


class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    __logger = Logger(__name__)

    def __init__(self, action_queue: ActionQueue):
        self.action_queue = action_queue
        self.running = False

    def start(self):
        self.running = True
        while event_bus.is_running() and self.running:
            event_bus.subscribe("action", String, self.receive_action)
            event_bus.spin()

    def receive_action(self, action):
        self.__logger.info("Received a new action request")
        action_request_json = json.loads(action.data)
        action_request = ApiActionRequest.from_json(action_request_json)
        action_group = action_request.to_action_group()
        self.action_queue.push(action_group)
