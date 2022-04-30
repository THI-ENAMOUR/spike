import json

import rospy

from src.api.model.api_action_request import ApiActionRequest
from src.core.action_queue import ActionQueue


# TODO: Adjust class once we know how the api looks like
class ActionApiClient:
    """Creates a communication channel for exchanging actions and the current state"""

    def __init__(self, action_queue: ActionQueue):
        self.action_queue = action_queue

    def start(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("action", self.receive_action)
            rospy.spin()

    def receive_action(self, action):
        action_request_json = json.loads(action)
        action_request = ApiActionRequest.from_json(action_request_json)
        action_group = action_request.to_action_group()
        self.action_queue.push(action_group)

