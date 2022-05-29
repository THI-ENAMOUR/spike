#!/usr/bin/env python2.7
import os

import rospy

from util.config import Config

# The path to the base directory of the project. Used for creating the log file.
PROJECT_BASE_PATH = os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))

if __name__ == "__main__":
    rospy.init_node("enamour_core")
    Config.init_config(PROJECT_BASE_PATH)

    from core.model.action.group.action_group import ActionGroup
    from core.action_queue import ActionQueue
    from core.model.action.atomic.pose_action import PoseAction

    __queue = ActionQueue()

    from app import Application

    app = Application(action_queue=__queue)

    __queue.push(
        ActionGroup(
            actions=[
                PoseAction(start_ms=0, end_ms=1000, x=10, y=5, z=8, ax=10, ay=10, az=10),

            ]
        )
    )

    app.start()
