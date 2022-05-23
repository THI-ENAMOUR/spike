from core.controller.controller import Controller
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.logger import Logger
import rospy
from geometry_msgs.msg import Twist


class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action):
        
        if not isinstance(action, NavigationAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        logger = Logger("cmd_vel")
        velocity_publisher = rospy.Publisher('cmd_vel', log_level=rospy.DEBUG)
        rate = rospy.Rate(1000)

        # For cmd_vel messages have to be of type Twist, Twist will have the attributes of the provided action
        vel_msg = Twist()
        vel_msg.linear.x = action.linear.x
        vel_msg.linear.y = action.linear.y
        vel_msg.linear.z = action.linear.z
        vel_msg.angular.x = action.angular.x
        vel_msg.angular.y = action.angular.y
        vel_msg.angular.z = action.angular.z

        if action.timing_option == StartTime:
            # Publishes messages in the topic after the start time was hit
            while not rospy.is_shutdown() and action.timing_option.in_time_frame():
                velocity_publisher.publish(vel_msg)
                logger.info(vel_msg)
                rate.sleep()

        elif action.timing_option == Duration:
            # Publishes messages in the topic while in the given duration
            while not rospy.is_shutdown and action.timing_option.in_time_frame():
                velocity_publisher.publish(vel_msg)
                logger.info(vel_msg)
                rate.sleep()

        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )
