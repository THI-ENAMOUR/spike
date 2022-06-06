#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client():

    rospy.loginfo("Starting client")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    rospy.loginfo("Setting parameters")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    x_input = float(input("Input meters to move forwards/backwards: "))
    y_input = float(input("Input meters to move left/right: "))
    goal.target_pose.pose.position.x = x_input
    goal.target_pose.pose.position.y = y_input
    goal.target_pose.pose.orientation.w = 1.0

    print("Sending command to action server")
    client.send_goal(goal)
    wait = client.wait_for_result()
    print("Action server received result")
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == "__main__":
    try:
        rospy.init_node("movebase_client_py")
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
