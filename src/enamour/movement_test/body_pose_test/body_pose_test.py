#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose

def body_pose_test():
	body_pose_publisher = rospy.Publisher('body_pose', Pose, queue_size=10)
	rospy.init_node('rospy_body_pose_test')
	while not rospy.is_shutdown():
		pose = str(input("Chose desired pose [sit/reset]: "))
		pose_msg = Pose()
		if pose == "sit":
			print("Executing 'sit'")
			pose_msg.position.x = -0.15
			pose_msg.position.z = 0.03
			pose_msg.orientation.y = -0.4
			pose_msg.orientation.w = 1
			body_pose_publisher.publish(pose_msg)
		elif pose == "reset":
			print("Executing 'reset'")
			pose_msg.position.x = 0
			pose_msg.position.y = 0
			pose_msg.position.z = 0
			pose_msg.orientation.x = 0
			pose_msg.orientation.y = 0
			pose_msg.orientation.z = 0
			pose_msg.orientation.w = 1
			body_pose_publisher.publish(pose_msg)
		else:
			print("Uknown command")

if __name__ == '__main__':
	try:
		body_pose_demo()
		print("Finished!!!")
	except rospy.ROSInterruptException: pass
