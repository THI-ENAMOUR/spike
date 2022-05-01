#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests

# Publisher Message Format:
# '{"expression:" "<type>"}'
# e.g. with rostopic command:
# rostopic pub /cmd_facial_expression std_msgs/String \
# "data: '{\"expression\": \"default\"}'"

URL = "http://localhost:5000/update-facial-expression"
HEADERS = {"content-type": "application/json"}


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "publish %s", data.data)
    # Post command to web server
    try:
        requests.post(URL, json=data.data, headers=HEADERS)
    except Exception as exp:
        print("Post to server unsuccessful", exp)


def start_listener():

    rospy.init_node("facial_expression_listener", anonymous=True)
    # Get cmd from backend controller
    rospy.Subscriber("cmd_facial_expression", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    start_listener()
