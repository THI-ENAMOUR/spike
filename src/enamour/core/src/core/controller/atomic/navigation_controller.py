from core.controller.controller import Controller
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.logger import Logger
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # for move_base
import actionlib  # for move_base
from tf.transformations import quaternion_from_euler
from unitree_legged_msgs.msg import HighCmd
import threading


class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def execute_action(self, action):

        if not isinstance(action, NavigationAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        self.vel_msg = Twist()

        logger = Logger("cmd_vel")
        velocity_publisher = rospy.Publisher("/high_command", HighCmd, queue_size=10)
        rate = rospy.Rate(500)

        if action.timing_option == StartTime:
            # print("Start time loop")
            # Publishes messages in the topic after the start time was hit
            rospy.Subscriber("/cmd_vel", Twist, self.callback, callback_args=self.vel_msg)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_footprint"  # We are only gonna use navigation based on base_link (so the navigation coordinates will be according to the robots current position)
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = action.x
            goal.target_pose.pose.position.y = action.y

            q_rot = quaternion_from_euler(0, 0, action.yaw)  # transforms degree to euler and then to quaternions
            goal.target_pose.pose.orientation.x = q_rot[0]  # uses qx-quaternion
            goal.target_pose.pose.orientation.y = q_rot[1]  # uses qy-quaternion
            goal.target_pose.pose.orientation.z = q_rot[2]  # uses qz-quaternion
            goal.target_pose.pose.orientation.w = q_rot[3]  # uses qw-quaternion

            move_base_thread = threading.Thread(target=thread_function, args=(goal,))
            move_base_thread.start()

            while (
                not rospy.is_shutdown()
                and action.in_time_frame(action.get_parent_time())
                and move_base_thread.is_alive()
            ):
                print(self.vel_msg)
                highCmd = self.velCmdToHighCmd(self.vel_msg)
                velocity_publisher.publish(highCmd)
                logger.info(self.vel_msg)
                logger.info(highCmd)
                rate.sleep()

        elif action.timing_option == Duration:
            # Publishes messages in the topic while in the given duration
            # while not rospy.is_shutdown() and action.timing_option.in_time_frame():
            #   velocity_publisher.publish(highCmd)
            #   logger.info(vel_msg)
            #   logger.info(vel_msg)
            #   rate.sleep()

            vel_msg = Twist()
            vel_msg.linear.x = action.x
            vel_msg.linear.y = action.y
            vel_msg.angular.z = action.yaw
            highCmd = self.velCmdToHighCmd(vel_msg)

            if not rospy.is_shutdown() and action.in_time_frame(action.get_parent_time()):
                velocity_publisher.publish(highCmd)
                logger.info(vel_msg)
                logger.info(highCmd)

            print(action.timing_option.end_time)


            if not rospy.is_shutdown() and action.get_parent_time() >= action.stopping_time.start_time:
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.z = 0
                highCmd = self.velCmdToHighCmd(vel_msg)
                velocity_publisher.publish(highCmd)
                logger.info(vel_msg)
                logger.info(highCmd)
                
            

        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )

    # Convert velocity to proportional HighCmd value
    def propToHighCMD(self, max_back_right_speed, max_forward_left_speed, value):
        newValue = 0.0
        if value < 0:
            newValue = value / max_back_right_speed
            if newValue < -1:
                # Set max backward or right speed
                newValue = -1.0
        else:
            newValue = value / max_forward_left_speed
            if newValue > 1:
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
        # HIGHLEVEL
        highCmd.levelFlag = 0x00
        # Standing mode
        highCmd.mode = 2
        highCmd.forwardSpeed = forwardSpeed
        highCmd.sideSpeed = sideSpeed
        highCmd.rotateSpeed = rotateSpeed

        return highCmd

    def callback(self, msg, vel_msg):
        vel_msg.linear.x = msg.linear.x
        vel_msg.linear.y = msg.linear.y
        vel_msg.angular.z = msg.angular.z


def thread_function(goal):
    print("Sending command to action server")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    wait = client.wait_for_result()
    print("Action server received results")
    if not wait:
        rospy.logerr("Action server not available")
        rospy.signal_shutdown("Action server not available")
    else:
        return client.get_result()
