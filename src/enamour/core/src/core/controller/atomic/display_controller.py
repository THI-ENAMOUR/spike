import rospy
from core.controller.controller import Controller
from core.model.action.atomic.display_action import DisplayAction
from error.illegal_state_error import IllegalStateError
from std_msgs.msg import String
from util.json_mapper import to_json
from util.logger import Logger


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    __logger = Logger(__name__)
    publisher = rospy.Publisher("/display", String, queue_size=10)

    def execute_action(self, action):
        if not isinstance(action, DisplayAction):
            raise IllegalStateError("This controller does not support the action " + str(action))
        DisplayController.__logger.debug("executing display action " + str(action))

        display_json = to_json(action.data)

        DisplayController.publisher.publish(display_json)
        action.complete()
