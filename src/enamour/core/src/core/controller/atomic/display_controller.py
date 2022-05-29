import rospy
from std_msgs.msg import String

from core.controller.controller import Controller
from core.model.action.atomic.display_action import DisplayAction
from core.model.action.timing_option import Duration
from error.illegal_state_error import IllegalStateError
from util.json_mapper import to_json
from util.logger import Logger


class DisplayActionMessage(object):
    def __init__(self, name, duration=None):
        self.name = name
        self.duration = duration


class DisplayController(Controller):
    """Controller for Actions regarding the display."""

    __logger = Logger(__name__)
    publisher = rospy.Publisher("/display", String, queue_size=10)

    def execute_action(self, action):
        if not isinstance(action, DisplayAction):
            raise IllegalStateError("This controller does not support the action " + str(action))
        DisplayController.__logger.debug("executing display action " + str(action))

        duration = None
        if isinstance(action.timing_option, Duration):
            duration = action.timing_option.duration_in_ms()

        display_json = to_json(DisplayActionMessage(name=action.name, duration=duration))

        DisplayController.publisher.publish(display_json)
        action.complete()
