import rospy
from std_msgs.msg import String

from core.controller.controller import Controller
from core.model.action.atomic.display_action import DisplayAction
from error.illegal_state_error import IllegalStateError
from util.logger import Logger


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    __logger = Logger(__name__)
    publisher = rospy.Publisher("/display", String, queue_size=10)

    def execute_action(self, action):
        if not isinstance(action, DisplayAction):
            raise IllegalStateError("This controller does not support the action " + str(action))
        DisplayController.__logger.debug("executing display action " + str(action))
        DisplayController.publisher.publish(action.name)
        action.complete()
