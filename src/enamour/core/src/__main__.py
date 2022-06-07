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

    __queue = ActionQueue()

    from app import Application

    app = Application(action_queue=__queue)

    app.start()
