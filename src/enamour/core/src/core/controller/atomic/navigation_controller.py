from core.controller.controller import Controller
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.logger import Logger
import rospy
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighCmd

class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action):
        
        if not isinstance(action, NavigationAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        logger = Logger("cmd_vel")
        velocity_publisher = rospy.Publisher('/high_command', HighCmd, queue_size=10)
        rate = rospy.Rate(1000)

        # For cmd_vel messages have to be of type Twist, Twist will have the attributes of the provided action
        vel_msg = Twist()
        vel_msg.linear.x = action.linear.x
        vel_msg.linear.y = action.linear.y
        vel_msg.linear.z = action.linear.z
        vel_msg.angular.x = action.angular.x
        vel_msg.angular.y = action.angular.y
        vel_msg.angular.z = action.angular.z
        highCmd = self.velCmdToHighCmd(vel_msg)

        if action.timing_option == StartTime:
            # Publishes messages in the topic after the start time was hit
            while not rospy.is_shutdown() and action.timing_option.in_time_frame():
                velocity_publisher.publish(highCmd)
                logger.info(vel_msg)
                logger.info(highCmd)
                rate.sleep()

        elif action.timing_option == Duration:
            # Publishes messages in the topic while in the given duration
            #while not rospy.is_shutdown() and action.timing_option.in_time_frame():
             #   velocity_publisher.publish(highCmd)
             #   logger.info(vel_msg)
             #   logger.info(vel_msg)
             #   rate.sleep()
            if not rospy.is_shutdown() and action.in_time_frame(action.get_parent_time()):
                velocity_publisher.publish(highCmd)
                logger.info(vel_msg)
                logger.info(highCmd)
            
            else:
                action.complete()

        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )
        
    # Convert velocity to proportional HighCmd value
    def propToHighCMD(self, max_back_right_speed, max_forward_left_speed, value):
        newValue = 0.0
        if (value < 0):
            newValue = value / max_back_right_speed
            if (newValue < -1):
                # Set max backward or right speed
                newValue = -1.0
        else:
            newValue =  value / max_forward_left_speed
            if (newValue > 1):
                # Set max forward or left speed
                newValue = 1.0

        return newValue
      
    # Convert velocity command (Twist message) to Unitree HighCMD
    def velCmdToHighCmd(self, twist):

        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional 
        # value of -0.7 m/s (max backward speed) and 1 m/s (max forward speed)
        forwardSpeed = self.propToHighCMD(0.7, 1.0, twist.linear.x) 


        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional 
        # value of -0.4 m/s (max rightward speed) and 0.4 m/s (max leftward speed)
        sideSpeed = self.propToHighCMD(0.4, 0.4, twist.linear.y)
        
        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional 
        # value of -120 deg/s / -2.0944 rad/s (max rightward speed) and 
        # 120 deg/s / 2.0944 rad/s (max leftward speed)
        rotateSpeed = self.propToHighCMD(2.0944, 2.0944, twist.angular.z)

        highCmd = HighCmd()
        highCmd.levelFlag = 0x00;   # HIGHLEVEL 
        highCmd.mode = 2            # Walking mode
        highCmd.forwardSpeed = forwardSpeed
        highCmd.sideSpeed = sideSpeed
        highCmd.rotateSpeed = rotateSpeed

        return highCmd
