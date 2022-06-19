#!/usr/bin/env python

import rospy
import threading
import tf2_msgs.msg
from unitree_legged_msgs.msg import HighState
from geometry_msgs.msg import Twist, TransformStamped
from tf.transformations import quaternion_from_euler
import math
import sys

# Get position updates from robot
def postion_update(state, position):
    position.forwardPosition = state.forwardPosition
    position.sidePosition = state.sidePosition
    position.bodyHeight = 0
    position.imu.quaternion[1] = state.imu.quaternion[1]
    position.imu.quaternion[2] = state.imu.quaternion[2]
    position.imu.quaternion[3] = state.imu.quaternion[3]
    position.imu.quaternion[0] = state.imu.quaternion[0]


# Test position updates without robot
# Calculate from velocity and direction
def postion_update_vel(vel, position):

    # Calc next postion from m/s (timestep 200 ms)
    delta_x = (vel.linear.x * math.cos(position.rotateSpeed) - vel.linear.y * math.sin(position.rotateSpeed)) * 0.2;
    delta_y = (vel.linear.x * math.sin(position.rotateSpeed) + vel.linear.y * math.cos(position.rotateSpeed)) * 0.2;
    position.forwardPosition = position.forwardPosition + delta_x
    position.sidePosition = position.sidePosition + delta_y
    
    # Calc quaternion from rad/s
    rad = vel.angular.z * 0.2
    # Save rotation 
    position.rotateSpeed += rad
    odom_quat =  quaternion_from_euler( 0, 0, position.rotateSpeed)
    position.imu.quaternion[1] = odom_quat[0]
    position.imu.quaternion[2] = odom_quat[1]
    position.imu.quaternion[3] = odom_quat[2]
    position.imu.quaternion[0] = odom_quat[3] 


# Send new postion updates form robot to move_base (tf-topic)
def send_position(state):
    seq = 0
    rate = rospy.Rate(500)
    publisher = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
    
    while not rospy.is_shutdown():
        
        now = rospy.Time.now()

        # Base frame
        postion = TransformStamped()
        postion.header.seq = seq
        postion.header.stamp = now
        postion.header.frame_id = "odom"
        postion.child_frame_id = "base_footprint"
        postion.transform.translation.x = state.forwardPosition
        postion.transform.translation.y = state.sidePosition
        postion.transform.translation.z = 0
        postion.transform.rotation.x = state.imu.quaternion[1]
        postion.transform.rotation.y = state.imu.quaternion[2]
        postion.transform.rotation.z = state.imu.quaternion[3]
        postion.transform.rotation.w = state.imu.quaternion[0] or 1.0 # set w to 1 if calc quaternion is 0
        
        seq = seq + 1

        tm = tf2_msgs.msg.TFMessage([postion])
        publisher.publish(tm)

        rate.sleep()


if __name__ == '__main__':
    try:
        last_postion = HighState()
        rospy.init_node('tf_transform', anonymous=True)
        postion_thread = threading.Thread(target=send_position, args=(last_postion,))
        postion_thread.start()
        hardware_connected = sys.argv[1]

        if hardware_connected == 'true':
            rospy.Subscriber("/high_state", HighState, postion_update, callback_args=last_postion)
            rospy.loginfo('Send real robot position')
        else:
            rospy.Subscriber("/cmd_vel", Twist, postion_update_vel, callback_args=last_postion)
            rospy.loginfo('Simulate robot position')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass