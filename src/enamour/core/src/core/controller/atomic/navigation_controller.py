import threading

import actionlib  # for move_base
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # for move_base
from unitree_legged_msgs.msg import HighCmd

from core.controller.controller import Controller
from core.model.action.atomic.navigation_action import NavigationAction
from core.model.action.timing_option import Duration, StartTime
from core.model.common.time_stamp import TimeStamp
from error.illegal_state_error import IllegalStateError
from util.degree_converter import quaternion_from_euler
from util.logger import Logger


class NavigationController(Controller):
    """Controller for making the robot walk to the provided position."""

    def __init__(self):
        self.logger = Logger("NavigationController")
        self.vel_msg = Twist()
        self.error = None
        self.rate = rospy.Rate(500)
        self.velocity_publisher = rospy.Publisher("/high_command", HighCmd, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)

    def execute_action(self, action):
        self.error = None

        if not isinstance(action, NavigationAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        try:
            if action.timing_option == StartTime:

                goal = MoveBaseGoal()
                # We are only gonna use navigation based on base_link
                # (so the navigation coordinates will be according to the robots current position)
                goal.target_pose.header.frame_id = "base_footprint"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = action.x
                goal.target_pose.pose.position.y = action.y

                q_rot = quaternion_from_euler(0, 0, action.yaw)  # transforms degree to euler and then to quaternions
                goal.target_pose.pose.orientation.x = q_rot[0]  # uses qx-quaternion
                goal.target_pose.pose.orientation.y = q_rot[1]  # uses qy-quaternion
                goal.target_pose.pose.orientation.z = q_rot[2]  # uses qz-quaternion
                goal.target_pose.pose.orientation.w = q_rot[3]  # uses qw-quaternion

                move_base_thread = threading.Thread(target=self.thread_function, args=(goal,))
                move_base_thread.start()

                while (
                    not rospy.is_shutdown()
                    and action.in_time_frame(action.get_parent_time())
                    and move_base_thread.is_alive()
                    and self.error is None
                ):
                    high_cmd = NavigationController.velCmdToHighCmd(self.vel_msg)
                    self.velocity_publisher.publish(high_cmd)
                    self.logger.debug("Publishing high command: " + str(high_cmd))
                    self.rate.sleep()

                if self.error is not None:
                    raise self.error

            elif action.timing_option == Duration:
                # Publishes messages in the topic while in the given duration

                if not rospy.is_shutdown() and action.get_parent_time() >= action.stopping_time.start_time:
                    # Publish stopping command
                    self.stop_cmd()

                elif not rospy.is_shutdown() and action.in_time_frame(action.get_parent_time()):
                    vel_msg = Twist()
                    vel_msg.linear.x = action.x
                    vel_msg.linear.y = action.y
                    vel_msg.angular.z = action.yaw
                    high_cmd = NavigationController.velCmdToHighCmd(vel_msg)
                    self.velocity_publisher.publish(high_cmd)
                    self.logger.debug("Publishing high command: " + str(high_cmd))

            else:
                raise NotImplementedError(
                    "Timing option {timing} for action {id} not implemented".format(
                        timing=action.timing_option, id=action.id
                    )
                )

        except BaseException as e:
            self.stop_cmd()
            raise e

    def stop_cmd(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        high_cmd = NavigationController.velCmdToHighCmd(vel_msg)
        self.velocity_publisher.publish(high_cmd)
        self.logger.debug("Publishing high command: " + str(high_cmd))

    @staticmethod
    def velCmdToHighCmd(twist):
        """Convert velocity command (Twist message) to Unitree HighCMD"""

        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional
        # value of -0.7 m/s (max backward speed) and 1 m/s (max forward speed)
        forward_speed = NavigationController.prop_to_high_cmd(0.7, 1.0, twist.linear.x)

        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional
        # value of -0.4 m/s (max rightward speed) and 0.4 m/s (max leftward speed)
        side_speed = NavigationController.prop_to_high_cmd(0.4, 0.4, twist.linear.y)

        # HighCmd value speed between -1 and 1. Value corresponds to a linear proportional
        # value of -120 deg/s / -2.0944 rad/s (max rightward speed) and
        # 120 deg/s / 2.0944 rad/s (max leftward speed)
        rotate_speed = NavigationController.prop_to_high_cmd(2.0944, 2.0944, twist.angular.z)

        high_cmd = HighCmd()
        high_cmd.levelFlag = 0x00  # high level flag
        high_cmd.mode = 2  # Standing mode
        high_cmd.forwardSpeed = forward_speed
        high_cmd.sideSpeed = side_speed
        high_cmd.rotateSpeed = rotate_speed

        return high_cmd

    @staticmethod
    def prop_to_high_cmd(max_back_right_speed, max_forward_left_speed, value):
        """Convert velocity to proportional HighCmd value"""
        if value < 0:
            new_value = value / max_back_right_speed
            if new_value < -1:
                # Set max backward or right speed
                new_value = -1.0
            return new_value
        else:
            new_value = value / max_forward_left_speed
            if new_value > 1:
                # Set max forward or left speed
                new_value = 1.0
            return new_value

    def callback(self, twist_msg):
        self.vel_msg.linear.x = twist_msg.linear.x
        self.vel_msg.linear.y = twist_msg.linear.y
        self.vel_msg.angular.z = twist_msg.angular.z

    def thread_function(self, goal):
        self.logger.info("send goal to action server")
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        started = client.wait_for_server(timeout=TimeStamp(secs=3))
        if not started:
            self.action_server_not_available()

        client.send_goal(goal)
        wait = client.wait_for_result()

        if not wait:
            self.action_server_not_available()
        else:
            return client.get_result()

    def action_server_not_available(self):
        self.logger.error("action server not available")
        error = IllegalStateError("action server not available")
        self.error = error
        raise error
