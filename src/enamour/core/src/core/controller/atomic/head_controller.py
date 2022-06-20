import select
import socket

from core.controller.controller import Controller
from core.model.action.atomic.head_action import HeadAction
from core.model.action.timing_option import Duration, StartTime
from error.illegal_state_error import IllegalStateError
from util.logger import Logger

logger = Logger(__name__)


class HeadController(Controller):
    """Controller for the head of the robot."""

    def execute_action(self, action):
        if not isinstance(action, HeadAction):
            raise IllegalStateError("This controller does not support the action " + str(action))

        if action.in_time_frame(action.get_parent_time()):

            logger.info("received " + str(action.roll) + "," + str(action.pitch) + "," + str(action.yaw))

            roll = action.roll
            pitch = action.pitch
            yaw = action.yaw

            tcp_string = "angles:" + str(int(roll)) + "," + str(int(pitch)) + "," + str(int(yaw)) + ","

            if action.timing_option == StartTime:
                logger.info("head action by StartTime")
                tcp_string = tcp_string + str(0) + "\0"
            elif action.timing_option == Duration:
                logger.info("head action by Duration")
                tcp_string = tcp_string + str(action.timing_option.duration_in_ms()) + "\0"
            else:
                raise NotImplementedError(
                    "Timing option {timing} for action {id} not implemented".format(
                        timing=action.timing_option, id=action.id
                    )
                )

            data = self.sendTCP(tcp_string)

            if data == "OK":
                logger.info("received OK from server")
            elif data == "error.busy":
                logger.warning("received busy error from server")
            elif data == "error.range":
                logger.warning("received range error from server")
            elif data == "error.unexpected":
                logger.warning("received unexpected error from server")
            else:
                logger.warning("unexpected error")

            action.complete()

    def sendTCP(self, string):
        logger.info("sending: |" + string + "|")

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("127.0.0.1", 12345))
        sock.setblocking(False)

        sock.sendall(string)

        ready = select.select([sock], [], [], 2)
        data = None

        if ready[0]:
            data = sock.recv(1024)

        sock.close()

        return data
