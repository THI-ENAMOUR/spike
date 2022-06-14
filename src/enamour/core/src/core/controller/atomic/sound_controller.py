import rospy
from core.controller.controller import Controller
from core.model.action.atomic.sound_action import SoundAction
from error.illegal_state_error import IllegalStateError
from std_msgs.msg import String
from util.json_mapper import to_json
from util.logger import Logger


class SoundController(Controller):
    """Controller for sound output."""

    __logger = Logger(__name__)
    publisher = rospy.Publisher("/sound", String, queue_size=10)

    def execute_action(self, action):
        if not isinstance(action, SoundAction):
            raise IllegalStateError("This controller does not support the action " + str(action))
        SoundController.__logger.debug("executing display action " + str(action))

        sound_json = to_json(action.data)

        SoundController.publisher.publish(sound_json)
        action.complete()
