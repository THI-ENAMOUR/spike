import json

import rospy
from std_msgs.msg import String

from api.model.api_action_request import ApiActionRequest
from error.handler.action_api_error_handler import ActionApiErrorHandler
from util.logger import Logger


class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    __logger = Logger(__name__)

    def __init__(self, action_queue, action_api_error_handler=ActionApiErrorHandler()):
        self.action_queue = action_queue
        self.action_api_error_handler = action_api_error_handler
        self.running = False

    def start(self):
        self.running = True
        rospy.Subscriber("action", String, self.receive_action)
        while not rospy.is_shutdown() and self.running:
            # Build our own ros spin command, in order to shut down the server if self.running is false
            rospy.rostime.wallsleep(0.5)

    def receive_action(self, action):
        try:
            self.__logger.info("Received a new action request")
            action_request_json = json.loads(action.data)
            action_request = ApiActionRequest.from_json(action_request_json)
            self.apply_action_request(action_request)
        except (BaseException,) as error:
            self.action_api_error_handler.handle(error)

    def apply_action_request(self, action_request):
        action_group = action_request.to_action_group()

        if action_request.clear_action_queue:
            self.action_queue.pop_all_actions()

        self.action_queue.push(action_group)
