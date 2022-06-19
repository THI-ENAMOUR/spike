import rospy
from geometry_msgs.msg import Pose
from unitree_legged_msgs.msg import HighCmd, HighState

from core.controller.controller import Controller
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.config import Config
from util.degree_converter import quaternion_from_euler, euler_from_quaternion


class PoseAggregate:
    def __init__(self, roll, pitch, yaw, body_height):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.body_height = body_height


current_pose = PoseAggregate(0, 0, 0, 0)


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""

    def __init__(self):
        self.high_cmd_publisher = rospy.Publisher("high_command", HighCmd, queue_size=10)
        self.body_pose_publisher = rospy.Publisher("body_pose", Pose, queue_size=10)
        self.highStateSub = rospy.Subscriber("high_state", HighState, self.update_high_state)

    time_prev = -1.0

    t_roll = 0.0
    t_pitch = 0.0
    t_yaw = 0.0
    t_height = 0.0

    m_roll = 0.0
    m_pitch = 0.0
    m_yaw = 0.0
    m_height = 0.0

    times_executed = 0

    def execute_action(self, action):
        if not isinstance(action, PoseAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        goal_pose = PoseController.define_goal_pose(action)

        if action.timing_option == StartTime:
            self.interpolate_start_time(goal_pose=goal_pose)

        elif action.timing_option == Duration:

            if action.get_parent_time() < action.timing_option.end_time or not PoseController.is_in_goal_state(
                goal_pose=goal_pose
            ):

                self.interpolate_duration(action=action, goal_pose=goal_pose)

            else:
                # If interpolation is complete, also complete the action
                action.complete()

        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )

        self.times_executed = self.times_executed + 1

    @staticmethod
    def define_goal_pose(action):
        global current_pose
        a_goal_roll = action.roll
        a_goal_pitch = action.pitch
        a_goal_yaw = action.yaw
        a_goal_body_height = action.body_height

        if a_goal_roll is None:
            a_goal_roll = current_pose.roll
        if a_goal_yaw is None:
            a_goal_yaw = current_pose.yaw
        if a_goal_pitch is None:
            a_goal_pitch = current_pose.pitch
        if a_goal_body_height is None:
            a_goal_body_height = current_pose.body_height

        return PoseAggregate(roll=a_goal_roll, pitch=a_goal_pitch, yaw=a_goal_yaw, body_height=a_goal_body_height)

    def interpolate_start_time(self, goal_pose):
        if Config.hardware_connected:
            # 3. publish highCmd
            high_cmd = HighCmd(
                roll=goal_pose.roll, yaw=goal_pose.yaw, pitch=goal_pose.pitch, bodyHeight=goal_pose.body_height
            )
            self.high_cmd_publisher.publish(high_cmd)
        else:
            pose_cmd = Pose()
            quaternion = quaternion_from_euler(goal_pose.roll, goal_pose.pitch, goal_pose.yaw)
            pose_cmd.orientation.x = quaternion[0]
            pose_cmd.orientation.y = quaternion[1]
            pose_cmd.orientation.z = quaternion[2]
            pose_cmd.orientation.w = quaternion[3]
            pose_cmd.position.z = goal_pose.body_height

            self.body_pose_publisher.publish(pose_cmd)

    def interpolate_duration(self, action, goal_pose):
        global current_pose
        if self.time_prev == -1:
            # 1st iteration -> set up the first linear equations, do not publish

            self.m_pitch = (goal_pose.pitch - current_pose.pitch) / (
                action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
            )
            self.t_pitch = current_pose.pitch - (self.m_pitch * action.get_parent_time().to_ms())

            self.m_roll = (goal_pose.roll - current_pose.roll) / (
                action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
            )
            self.t_roll = current_pose.roll - (self.m_roll * action.get_parent_time()).to_ms()

            self.m_yaw = (goal_pose.yaw - current_pose.yaw) / (
                action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
            )
            self.t_yaw = current_pose.yaw - (self.m_yaw * action.get_parent_time().to_ms())

            self.m_height = (goal_pose.body_height - current_pose.body_height) / (
                action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
            )
            self.t_height = current_pose.body_height - (self.m_height * action.get_parent_time().to_ms())
        else:
            # 1. correct slope and y-intercept from previous tick with a_measured_*

            self.m_pitch = (goal_pose.pitch - current_pose.pitch) / (
                action.timing_option.end_time.to_ms() - self.time_prev
            )
            self.t_pitch = current_pose.pitch - (self.m_pitch * self.time_prev)

            self.m_roll = (goal_pose.roll - current_pose.roll) / (
                action.timing_option.end_time.to_ms() - self.time_prev
            )
            self.t_roll = current_pose.roll - (self.m_roll * self.time_prev)

            self.m_yaw = (goal_pose.yaw - current_pose.yaw) / (action.timing_option.end_time.to_ms() - self.time_prev)
            self.t_yaw = current_pose.yaw - (self.m_yaw * self.time_prev)

            self.m_height = (goal_pose.body_height - current_pose.body_height) / (
                action.timing_option.end_time.to_ms() - self.time_prev
            )
            self.t_height = current_pose.body_height - (self.m_height * self.time_prev)

            # 2. calculate angle of current time with corrected linear equation

            publish_pitch = self.m_pitch * action.get_parent_time().to_ms() + self.t_pitch
            publish_roll = self.m_roll * action.get_parent_time().to_ms() + self.t_roll
            publish_yaw = self.m_yaw * action.get_parent_time().to_ms() + self.t_yaw
            publish_height = self.m_height * action.get_parent_time().to_ms() + self.t_height

            if Config.hardware_connected:
                # 3. publish highCmd
                high_cmd = HighCmd(roll=publish_roll, yaw=publish_yaw, pitch=publish_pitch, bodyHeight=publish_height)
                self.high_cmd_publisher.publish(high_cmd)
            else:
                pose_cmd = Pose()
                quaternion = quaternion_from_euler(publish_roll, publish_pitch, publish_yaw)
                pose_cmd.orientation.x = quaternion[0]
                pose_cmd.orientation.y = quaternion[1]
                pose_cmd.orientation.z = quaternion[2]
                pose_cmd.orientation.w = quaternion[3]
                pose_cmd.position.z = publish_height
                print("received pose_cmd  " + str(pose_cmd))

                self.body_pose_publisher.publish(pose_cmd)

            # 3.1 TODO Temporary since we do not have the highStates yet
            current_pose.pitch = publish_pitch
            current_pose.roll = publish_roll
            current_pose.yaw = publish_yaw
            current_pose.body_height = publish_height

        # 4. update previous tick
        self.time_prev = action.get_parent_time().to_ms()

    def update_high_state(self, high_state):
        global current_pose

        w = high_state.imu.quaternion[0]
        x = high_state.imu.quaternion[1]
        y = high_state.imu.quaternion[2]
        z = high_state.imu.quaternion[3]

        quaternion = (x, y, z, w)
        euler = euler_from_quaternion(quaternion)

        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        print("roll received: " + str(roll))
        print("pitch received: " + str(pitch))
        print("yaw received: " + str(yaw))

        current_pose.roll = roll
        current_pose.pitch = pitch
        current_pose.yaw = yaw

    @staticmethod
    def is_in_goal_state(goal_pose):
        global current_pose
        return (
            PoseController.estimate_equals(goal_pose.roll, current_pose.roll, 0.05)
            and PoseController.estimate_equals(goal_pose.pitch, current_pose.pitch, 0.05)
            and PoseController.estimate_equals(goal_pose.yaw, current_pose.yaw, 0.05)
            and PoseController.estimate_equals(goal_pose.body_height, current_pose.body_height, 0.05)
        )

    @staticmethod
    def estimate_equals(a1, a2, max_difference):
        return abs(a1 - a2) <= max_difference
