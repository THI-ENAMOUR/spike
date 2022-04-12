#!/usr/bin/env python

import rospy
from std_msgs.msg import String

topic_name = "dummy_topic_name"
pub = rospy.Publisher(topic_name, String, queue_size=10)


def pub_message(self):
    while not rospy.is_shutdown():
        test_message = "Dummy message %s" % rospy.get_time()
        pub.publish(test_message)


def init_sub():
    rospy.Subscriber(topic_name, String, received_message)


def received_message(string):
    rospy.loginfo(rospy.get_caller_id() + "Received message: ", string.data)


if __name__ == '__main__':
    try:
        rospy.init_node('dummy')
        init_sub()



        loop_rate = rospy.get_param("loop_rate", default=10)  # Use tick from parameter server
        timer = rospy.Timer(rospy.Duration(1/loop_rate), pub_message)

        rospy.spin()
        timer.shutdown()

    except rospy.ROSInterruptException:
        pass
