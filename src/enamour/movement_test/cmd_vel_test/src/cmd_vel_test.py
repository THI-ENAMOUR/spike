#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist

def cmd_vel_test():
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('rospy_cmd_vel_test')
    rate = rospy.Rate(10)
    speed = float(input("Input your speed: "))
    vel_msg = Twist()
    vel_msg.linear.x = speed
    i = 0
    
    while not rospy.is_shutdown() and i < 20:
            velocity_publisher.publish(vel_msg)
            #rospy.loginfo(vel_msg)
            i+=1
            rate.sleep()

if __name__ == '__main__':
    try:
        cmd_vel_test()
        print("Finished!!!")
    except rospy.ROSInterruptException: pass
