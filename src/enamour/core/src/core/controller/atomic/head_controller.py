import socket
import select

import threading

import rospy

from core.controller.controller import Controller
from core.model.action.atomic.head_action import HeadAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.degree_converter import quaternion_from_euler
from util.logger import Logger

from core.controller.controller import Controller


class HeadController(Controller):
    """Controller for the head of the robot."""

    def execute_action(self, action):
        logger = Logger("head_controller")

        if action.get_parent_time() > action.timing_option.start_ms:

            logger.info("received " + str(action.roll) + "," + str(action.pitch) + "," + str(action.yaw))

            roll = action.roll
            pitch = action.pitch
            yaw = action.yaw

            tcp_string = "angles:" + str(roll) + "," + str(pitch) + "," + str(yaw) + ","

            if action.timing_option == StartTime:
                logger.info("head action by StartTime")
                tcp_string = tcp_string + str(0) + "\0"
            elif action.timing_option == Duration:
                logger.info("head action by Duration")
                tcp_string = tcp_string + str(action.timing_option.end_ms - action.timing_option.start_ms) + "\0"
            else:
                raise NotImplementedError(
                    "Timing option {timing} for action {id} not implemented".format(
                        timing=action.timing_option, id=action.id
                    )
                )

            data = sendTCP(tcp_string)

            if data == "OK":
                logger.info("received OK from server")
            elif data == "error.busy":
                logger.warn("received busy error from server")
            elif data == "error.range":
                logger.warn("received range error from server")
            elif data == "error.unexpected":
                logger.warn("received unexpected error from server")

            action.complete()

    def sendTCP(string):
        logger.info("sending: |" + string + "|")

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("127.0.0.1", 12345))
        sock.setblocking(0)

        sock.sendall(string)

        data = sock.recv(1024)

        sock.close()

        return data