#!/usr/bin/env python2.7
import os
import sys
from distutils.util import strtobool

import rospy

from util.config import Config

# The path to the base directory of the project. Used for creating the log file.
PROJECT_BASE_PATH = os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))


def argv_to_dict(argv):
    argv_dict = dict(map(lambda s: s.split("="), argv[1:]))
    for key, value in argv_dict.items():
        argv_dict[key] = parse_argv_value(value)
    return argv_dict


def parse_argv_value(string):
    try:
        return bool(strtobool(string))
    except ValueError:
        return string


if __name__ == "__main__":
    rospy.init_node("enamour_core")
    passed_arguments = argv_to_dict(sys.argv)
    Config.init_config(passed_arguments, PROJECT_BASE_PATH)

    from core.action_queue import ActionQueue

    from core.model.action.atomic.pose_action import PoseAction
    from core.model.action.group.action_group import ActionGroup

    __queue = ActionQueue()
    # __queue.push(ActionGroup(actions=[HeadAction(1000, roll=None, pitch=0.3, yaw=None)]))
    # __queue.push(ActionGroup(actions=[HeadAction(1000, roll=None, pitch=0.3, yaw=None)]))
    __queue.push(
        ActionGroup(
            actions=[
                PoseAction(start_ms=0, end_ms=800, roll=0, pitch=0.1, yaw=0, body_height=-0.05),
                PoseAction(start_ms=1600, end_ms=2400, roll=0, pitch=None, yaw=0.2, body_height=None),
                PoseAction(start_ms=2400, end_ms=3200, roll=0, pitch=None, yaw=-0.2, body_height=None),
                PoseAction(start_ms=3200, end_ms=4000, roll=0, pitch=None, yaw=0.2, body_height=None),
                PoseAction(start_ms=4000, end_ms=4800, roll=0, pitch=None, yaw=-0.2, body_height=None),
                PoseAction(start_ms=4800, end_ms=5600, roll=0, pitch=0, yaw=0, body_height=0),
            ]
        )
    )
    # __queue.push(ActionGroup(actions=[PoseAction(start_ms=4000, end_ms=5000, roll=0, pitch=None, yaw=0.2)]))

    # __queue.push(ActionGroup(actions=[PoseAction(start_ms=3000, end_ms=3500, roll=0, pitch=None, yaw=-0.001)]))

    from app import Application

    app = Application(action_queue=__queue)

    app.start()
