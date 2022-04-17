import rospy


def start_ros_node():
    rospy.init_node("core")


def get_ros_parameter(name: str, default):
    return rospy.get_param(name, default)
