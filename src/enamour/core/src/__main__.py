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
                PoseAction(start_ms=0, end_ms=2000, roll=0, pitch=0.2, yaw=0.5),
		PoseAction(start_ms=2000, end_ms=3000, roll=0, pitch=0, yaw=0.5),
		PoseAction(start_ms=3000, end_ms=5000, roll=0, pitch=-0.2, yaw=0.5),
		PoseAction(start_ms=5000, end_ms=6000, roll=0, pitch=0, yaw=0),
		PoseAction(start_ms=6000, end_ms=9000, roll=0, pitch=0.2, yaw=-0.5),
		PoseAction(start_ms=9000, end_ms=10000, roll=0, pitch=0, yaw=-0.5),
                PoseAction(start_ms=10000, end_ms=12000, roll=0, pitch=-0.2, yaw=-0.5),

		#PoseAction(start_ms=0, end_ms=5000, roll=0, pitch=-0.5, yaw=0),
		#PoseAction(start_ms=5000, end_ms=10000, roll=0, pitch=0.5, yaw=0),
		#PoseAction(start_ms=15000, end_ms=20000, roll=0, pitch=0, yaw=0),
            ]
        )
    )

    app.start()
