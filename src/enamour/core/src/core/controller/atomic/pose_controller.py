import rospy
from geometry_msgs.msg import Pose
from unitree_legged_msgs.msg import HighCmd, HighState

from core.controller.controller import Controller
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.config import Config
from util.degree_converter import quaternion_from_euler, euler_from_quaternion

a_measured_roll = 0.0
a_measured_pitch = 0.0
a_measured_yaw = 0.0


class PoseController(Controller):
    def __init__(self):
        self.high_cmd_publisher = rospy.Publisher("high_command", HighCmd, queue_size=10)
        self.body_pose_publisher = rospy.Publisher("body_pose", Pose, queue_size=10)
        self.highStateSub = rospy.Subscriber("high_state", HighState, self.updateHighState)

    """Controller for moving the robot body in the provided position."""

    # note: 'a_' := angle
    #       't_' := y-intercept
    #       'm_' := slope

    # temporary without using highState

    time_prev = -1.0

    t_roll = 0.0
    t_pitch = 0.0
    t_yaw = 0.0

    m_roll = 0.0
    m_pitch = 0.0
    m_yaw = 0.0

    times_executed = 0

    def updateHighState(self, highState):
        global a_measured_roll, a_measured_pitch, a_measured_yaw
        w = highState.imu.quaternion[0]
        x = highState.imu.quaternion[1]
        y = highState.imu.quaternion[2]
        z = highState.imu.quaternion[3]
        quaternionen = (x, y, z, w)
        euler = euler_from_quaternion(quaternionen)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print("roll received: " + str(roll))
        print("pitch received: " + str(pitch))
        print("yaw received: " + str(yaw))
        a_measured_roll = roll
        a_measured_pitch = pitch
        a_measured_yaw = yaw

    def execute_action(self, action):
        global a_measured_roll, a_measured_pitch, a_measured_yaw
        if not isinstance(action, PoseAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        if action.timing_option == StartTime:
            # TODO Jump instantly to position by publishing desired position
            pass
        elif action.timing_option == Duration:

            a_goal_roll = action.roll  # fuellen der Variable mit der Ziel Kordinate
            a_goal_pitch = action.yaw  # fuellen der Variable mit der Ziel Kordinate
            a_goal_yaw = action.pitch  # fuellen der Variable mit der Ziel Kordinate

            # a_measured_roll = self.getCurrentHighState()['roll']
            # a_measured_pitch = self.getCurrentHighState()['pitch']
            # a_measured_yaw = self.getCurrentHighState()['yaw']

            print("received roll" + str(a_measured_roll))

            if not (
                action.get_parent_time() > action.timing_option.end_time
                or a_measured_roll >= a_goal_roll
                and a_measured_yaw >= a_goal_yaw
                and a_measured_pitch >= a_goal_pitch
            ):
                self.times_executed = self.times_executed + 1
                if self.time_prev == -1:
                    # 1st iteration -> set up the first linear equations, do not publish
                    self.m_pitch = (a_goal_pitch - a_measured_pitch) / (
                        action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
                    )
                    self.t_pitch = a_measured_pitch - (self.m_pitch * action.get_parent_time().to_ms())

                    self.m_roll = (a_goal_roll - a_measured_roll) / (
                        action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
                    )
                    self.t_roll = a_measured_roll - (self.m_roll * action.get_parent_time()).to_ms()

                    self.m_yaw = (a_goal_yaw - a_measured_yaw) / (
                        action.timing_option.end_time.to_ms() - action.get_parent_time().to_ms()
                    )
                    self.t_yaw = a_measured_yaw - (self.m_yaw * action.get_parent_time().to_ms())
                else:
                    # 1. correct slope and y-intercept from previous tick with a_measured_*
                    self.m_pitch = (a_goal_pitch - a_measured_pitch) / (
                        action.timing_option.end_time.to_ms() - self.time_prev
                    )
                    self.t_pitch = a_measured_pitch - (self.m_pitch * self.time_prev)

                    self.m_roll = (a_goal_roll - a_measured_roll) / (
                        action.timing_option.end_time.to_ms() - self.time_prev
                    )
                    self.t_roll = a_measured_roll - (self.m_roll * self.time_prev)

                    self.m_yaw = (a_goal_yaw - a_measured_yaw) / (
                        action.timing_option.end_time.to_ms() - self.time_prev
                    )
                    self.t_yaw = a_measured_yaw - (self.m_yaw * self.time_prev)

                    # 2. calculate angle of current time with corrected linear equation
                    publish_pitch = self.m_pitch * action.get_parent_time().to_ms() + self.t_pitch
                    publish_roll = self.m_roll * action.get_parent_time().to_ms() + self.t_roll
                    publish_yaw = self.m_yaw * action.get_parent_time().to_ms() + self.t_yaw
                    if Config.hardware_connected:
                        # 3. publish highCmd
                        high_cmd = HighCmd(roll=publish_roll, yaw=publish_yaw, pitch=publish_pitch)
                        self.high_cmd_publisher.publish(high_cmd)
                    else:
                        pose_cmd = Pose()
                        quaternion = quaternion_from_euler(publish_roll, publish_pitch, publish_yaw)
                        pose_cmd.orientation.x = quaternion[0]
                        pose_cmd.orientation.y = quaternion[1]
                        pose_cmd.orientation.z = quaternion[2]
                        pose_cmd.orientation.w = quaternion[3]

                        self.body_pose_publisher.publish(pose_cmd)

                    # 3.1 TODO Temporary since we do not have the highStates yet
                    # self.a_measured_pitch = publish_pitch
                    # self.a_measured_roll = publish_roll
                    # self.a_measured_yaw = publish_yaw

                # 4. update previous tick
                self.time_prev = action.get_parent_time().to_ms()
            else:
                action.complete()  # Wenn abbruch Bedingung von der weil Schleife eintritt action.completed()

        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )
