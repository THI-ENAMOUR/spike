import rospy
from core.controller.controller import Controller
from core.model.action.atomic.pose_action import PoseAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from unitree_legged_msgs.msg import HighCmd


class PoseController(Controller):
    """Controller for moving the robot body in the provided position."""
    a_measure_roll = 0                                                       #globale Variable
    a_measure_pitch = 0                                                      #globale Variable
    a_measure_yaw = 0                                                        #globale Variable
    t_ak_roll = 0                                                          #globale Variable
    t_ak_pitch = 0                                                           #globale Variable
    t_ak_yaw = 0
    def execute_action(self, action):

        if not isinstance(action, PoseAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        if action.timing_option == StartTime:
            # TODO Jump instantly to position by publishing desired position
            pass
        elif action.timing_option == Duration:

            a_mea_roll = action.linear.x              # Aufruf der measurement fuer ROLL
            a_mea_pitch = action.linear.z            # Aufruf der measurement fuer PITCH
            a_mea_yaw = action.linear.y                 # Aufruf der measurement fuer YAW

            a_goal_roll = action.linear.ax                   # fuellen der Variable mit der Ziel Kordinate
            a_goal_pitch = action.linear.az                  # fuellen der Variable mit der Ziel Kordinate
            a_goal_yaw = action.linear.ay                    # fuellen der Variable mit der Ziel Kordinate

            t_ak_roll = self.t_ak_roll                      # Aufruf der y Achsenschnittstelle  für ROLL
            t_ak_pitch = self.t_ak_pitch                    # Aufruf der y Achsenschnittstelle  für PITCH
            t_ak_yaw = self.t_ak_yaw                        # Aufruf der y Achsenschnittstelle  für YAW

            t_ak_roll = action.linear.x                     #befuellen des y Achsenabschnittes mit ihrem Wert
            t_ak_pitch = action.linear.z                    #befuellen des y Achsenabschnittes mit ihrem Wert
            t_ak_yaw = action.linear.y                      #befuellen des y Achsenabschnittes mit ihrem Wert

            m_acc_roll = a_mea_roll                         # füllen der zu benutzenden Variablen
            m_acc_pitch = a_mea_pitch                       # füllen der zu benutzenden Variablen
            m_acc_yaw = a_mea_yaw                           # füllen der zu benutzenden Variablen

            publish_roll = 0                                # Initialisierung für die erste Runde
            publish_pitch = 0                               # Initialisierung für die erste Runde
            publish_yaw = 0                                 # Initialisierung für die erste Runde



        while not (action.get_parent_time() >= action.timing_option.end_time | a_mea_roll >= a_goal_roll | a_mea_yaw >= a_goal_yaw | a_mea_pitch >= a_goal_pitch):   # wenn gewuenschter Wert oder Zeit ereicht ist

            m_acc_roll = a_mea_roll                                                                                 # Updaten für die Naechsten Runden                                                                                    beenden der while schleife
            m_acc_pitch = a_mea_pitch                                                                               # Updaten für die Naechsten Runden
            m_acc_yaw = a_mea_yaw                                                                                   # Updaten für die Naechsten Runden

            m_acc_roll = (a_goal_roll - a_mea_roll)/(action.timing_option.end_time - action.get_parent_time())      # Steigung der Aktuelen ROLL Graphen
            publish_roll = m_acc_roll * action.timing_option + t_ak_roll                                            # Speicherung der Berechneten ROLL Werte(y wert)

            m_acc_pitch = (a_goal_pitch - a_mea_pitch)/(action.timing_option.end_time - action.get_parent_time())    # Steigung der Aktuelen PITCH Graphen
            publish_pitch = m_acc_pitch * action.timing_option + t_ak_pitch                                          # Speicherung der Berechneten PITCH Werte (y Wert)

            m_acc_yaw = (a_goal_yaw - a_mea_yaw)/(action.timing_option.end_time - action.get_parent_time())         # Steigung der Aktuelen YAW Graphen
            publish_yaw = m_acc_yaw * action.timing_option + t_ak_yaw                                               # Speicherung der Berechneten YAW Werte(y Wert)

            high_cmd = HighCmd()                                                                                    # erstellen der Nachricht
            high_cmd_publisher = rospy.Publisher('high_command', HighCmd, queue_size=10)                            # publisher für HighCmd Nachricht
            high_cmd.roll = publish_roll                                                                            # befuellen der Nachricht mit Werten
            high_cmd.pitch = publish_pitch                                                                          # befuellen der Nachricht mit Werten
            high_cmd.yaw = publish_yaw                                                                              # befuellen der Nachricht mit Werten
            high_cmd_publisher.publish(high_cmd)                                                                    # publishen der Nachricht

            t_ak_roll = publish_roll - (m_acc_roll * action.timing_option)                                          # updaten der t Variable durch die Messung der echten Messwerte für ROLL
            t_ak_pitch = publish_pitch - (m_acc_pitch * action.timing_option)                                       # updaten der t Variable durch die Messung der echten Messwerte für PITCH
            t_ak_yaw = publish_yaw - (m_acc_yaw * action.timing_option)                                             # updaten der t Variable durch die Messung der echten Messwerte für YAW

        action.completed()                                                                                          # Wenn abbruch Bedingung von der weil Schleife eintritt action.completed()



           # # TODO Interpolate the pose based on the parent_duration, current location (TODO), and desired end position
            pass
        else:
            raise NotImplementedError(
                "Timing option {timing} for action {id} not implemented".format(
                    timing=action.timing_option, id=action.id
                )
            )
