import rospy
from core.controller.controller import Controller
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from unitree_legged_msgs.msg import HighCmd


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""

    # note: 'a_' := angle
    #       't_' := y-intercept
    #       'm_' := slope

    # temporary without using highState

    time_prev = -1

    a_last_roll = 0
    a_last_pitch = 0
    a_last_yaw = 0

    t_roll = 0
    t_pitch = 0
    t_yaw = 0

    m_roll = 0
    m_pitch = 0
    m_yaw = 0

    def getCurrentHighState(self):
        # TODO
        return {'roll': self.a_last_row, 'pitch': self.a_last_pitch, 'yaw': self.a_last_yaw}

    def execute_action(self, action):

        if not isinstance(action, PoseAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        if action.timing_option == StartTime:
            # TODO Jump instantly to position by publishing desired position
            pass
        elif action.timing_option == Duration:

            a_goal_roll = action.linear.ax  # fuellen der Variable mit der Ziel Kordinate
            a_goal_pitch = action.linear.az  # fuellen der Variable mit der Ziel Kordinate
            a_goal_yaw = action.linear.ay  # fuellen der Variable mit der Ziel Kordinate

            a_measured_roll = self.getCurrentHighState()['roll']
            a_measured_pitch = self.getCurrentHighState()['pitch']
            a_measured_yaw = self.getCurrentHighState()['yaw']

            if not (action.get_parent_time() >= action.timing_option.end_time or a_measured_roll >= a_goal_roll and
                    a_measured_yaw >= a_goal_yaw and a_measured_pitch >= a_goal_pitch):
                if self.time_prev == -1:
                    # 1st iteration -> set up the first linear equations, do not publish
                    self.m_pitch = (a_goal_pitch - a_measured_pitch) /\
                                   (action.timing_option.end_time - action.get_parent_time())
                    self.t_pitch = a_measured_pitch - (self.m_pitch * action.get_parent_time())

                    self.m_roll = (a_goal_roll - a_measured_roll) /\
                                  (action.timing_option.end_time - action.get_parent_time())
                    self.t_roll = a_measured_roll - (self.m_roll * action.get_parent_time())

                    self.m_yaw = (a_goal_yaw - a_measured_yaw) /\
                                 (action.timing_option.end_time - action.get_parent_time())
                    self.t_yaw = a_measured_yaw - (self.m_yaw * action.get_parent_time())
                else:
                    # 1. correct slope and y-intercept from previous tick with a_measured_*
                    self.m_pitch = (a_goal_pitch - a_measured_pitch) / (action.timing_option.end_time - self.time_prev)
                    self.t_pitch = a_measured_pitch - (self.m_pitch * self.time_prev)

                    self.m_roll = (a_goal_roll - a_measured_roll) / (action.timing_option.end_time - self.time_prev)
                    self.t_roll = a_measured_roll - (self.m_roll * self.time_prev)

                    self.m_yaw = (a_goal_yaw - a_measured_yaw) / (action.timing_option.end_time - self.time_prev)
                    self.t_yaw = a_measured_yaw - (self.m_yaw * self.time_prev)

                    # 2. calculate angle of current time with corrected linear equation
                    publish_pitch = self.m_pitch * action.get_parent_time() + self.t_pitch
                    publish_roll = self.m_roll * action.get_parent_time() + self.t_roll
                    publish_yaw = self.m_yaw * action.get_parent_time() + self.t_yaw

                    # 3. publish highCmd
                    high_cmd = HighCmd()  # erstellen der Nachricht
                    high_cmd_publisher = rospy.Publisher('high_command', HighCmd,
                                                         queue_size=10)  # publisher für HighCmd Nachricht
                    high_cmd.roll = publish_roll  # befuellen der Nachricht mit Werten
                    high_cmd.pitch = publish_pitch  # befuellen der Nachricht mit Werten
                    high_cmd.yaw = publish_yaw  # befuellen der Nachricht mit Werten
                    high_cmd_publisher.publish(high_cmd)  # publishen der Nachricht

                    # 3.1 TODO Temporary since we do not have the highStates yet
                    self.a_last_pitch = publish_pitch
                    self.a_last_roll = publish_roll
                    self.a_last_yaw = publish_yaw

                # 4. update previous tick
                self.time_prev = action.get_parent_time()
            else:
                action.completed()  # Wenn abbruch Bedingung von der weil Schleife eintritt action.completed()
            pass
        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )