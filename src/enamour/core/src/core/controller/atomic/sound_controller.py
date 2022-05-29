import rospy
from std_msgs.msg import String

from core.controller.controller import Controller
from core.model.action.atomic.sound_action import SoundAction
from core.model.action.timing_option import Duration
from error.illegal_state_error import IllegalStateError
from util.json_mapper import to_json
from util.logger import Logger


class SoundActionMessage(object):
    def __init__(self, name, duration=None):
        self.name = name
        self.duration = duration


class SoundController(Controller):
    """Controller for sound output."""

    __logger = Logger(__name__)
    publisher = rospy.Publisher("/sound", String, queue_size=10)

    def execute_action(self, action):
        if not isinstance(action, SoundAction):
            raise IllegalStateError("This controller does not support the action " + str(action))
        SoundController.__logger.debug("executing display action " + str(action))

        duration = None
        if isinstance(action.timing_option, Duration):
            duration = action.timing_option.duration_in_ms()

        sound_json = to_json(SoundActionMessage(name=action.name, duration=duration))

        SoundController.publisher.publish(sound_json)
        action.complete()
