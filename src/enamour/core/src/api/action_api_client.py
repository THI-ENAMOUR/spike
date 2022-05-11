import json
from core.model.state import State

import rospy
from std_msgs.msg import String

from api.model.api_action_request import ApiActionRequest
from core.action_queue import ActionQueue
from error.handler.action_api_error_handler import ActionApiErrorHandler
from util.logger import Logger


class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    publisher = rospy.Publisher('/robot_state', String, queue_size=10)

    __logger = Logger(__name__)

    def __init__(
        self, action_queue: ActionQueue, action_api_error_handler: ActionApiErrorHandler = ActionApiErrorHandler()
    ):
        self.action_queue = action_queue
        self.action_api_error_handler = action_api_error_handler
        self.running = False

    def start(self):
        self.running = True
        rospy.Subscriber("action", String, self.receive_action)
        while not rospy.is_shutdown() and self.running:
            # Build our own ros spin command, in order to shut down the server if self.running is false
            self.send_robot_state()
            rospy.rostime.wallsleep(0.5)

    def receive_action(self, action):
        try:
            self.__logger.info("Received a new action request")
            action_request_json = json.loads(action.data)
            action_request = ApiActionRequest.from_json(action_request_json)
            action_group = action_request.to_action_group()
            self.action_queue.push(action_group)
        except (BaseException,) as error:
            self.action_api_error_handler.handle(error)
    
    def send_robot_state(self):
        State.update(self.action_queue);
        json = State.to_json();
        print(json)
        ActionApiClient.publisher.publish(json)