#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import time
from tf.transformations import quaternion_from_euler
from unitree_legged_msgs.msg import HighCmd

body_pose_publisher = rospy.Publisher("body_pose", Pose, queue_size=10)
high_cmd_publisher = rospy.Publisher("high_command", HighCmd, queue_size=10)
rospy.init_node("pose_test")

elapsed = 0.0

last_roll = 0.0
last_pitch = 0.0
last_yaw = 0.0
last_x = 0.0
last_y = 0.0
last_z = 0.5


class Pose_cmd:
    def __init__(self, x, y, z, yaw, pitch, roll, start, end):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.start = start
        self.end = end


class Action_Group:
    def __init__(self, poses):
        self.elapsed = 0.0
        self.poses = poses

    def progress(self, loop_rate):
        self.elapsed += loop_rate
        print(self.elapsed)

    def get_current_pose(self):
        for pose in self.poses:
            if self.elapsed > pose.start:
                if self.elapsed <= pose.end:
                    return pose


def make_pose(final_pose, action_group_elapsed, loop_rate):
    global last_roll, last_pitch, last_yaw

    start_time = final_pose.start
    end_time = final_pose.end

    elapsed = action_group_elapsed - start_time
    duration = (end_time - elapsed) - start_time
    if duration == 0:
        return

    high_cmd = HighCmd()

    pose_msg = Pose()
    x = final_pose.x
    y = final_pose.y
    # z = final_pose.z
    yaw = final_pose.yaw
    pitch = final_pose.pitch
    roll = final_pose.roll

    # linear interpolation for positions
    y_x = ((x - last_x) / duration) * loop_rate + last_x
    y_y = ((y - last_y) / duration) * loop_rate + last_y
    # y_z = ((z - last_z)/duration)* elapsed + last_z
    pose_msg.position.x = y_x
    pose_msg.position.y = y_y
    # pose_msg.position.z = y_z

    # linear interpolation for orientation
    y_yaw = ((yaw - last_yaw) / duration) * loop_rate + last_yaw
    y_pitch = ((pitch - last_pitch) / duration) * loop_rate + last_pitch
    y_roll = ((roll - last_roll) / duration) * loop_rate + last_roll
    quaternion = quaternion_from_euler(y_roll, y_pitch, y_yaw)
    pose_msg.orientation.x = quaternion[0]
    pose_msg.orientation.y = quaternion[1]
    pose_msg.orientation.z = quaternion[2]
    pose_msg.orientation.w = quaternion[3]
    # body_pose_publisher.publish(pose_msg)
    high_cmd.roll = y_roll
    high_cmd.pitch = y_pitch
    high_cmd.yaw = y_yaw
    high_cmd.mode = 1  # 1. force stand (controlled by dBodyHeight + ypr)
    high_cmd_publisher.publish(high_cmd)

    last_yaw = y_yaw
    last_pitch = y_pitch
    last_roll = y_roll


def test_high_cmd():
    high_cmd = HighCmd()

    high_cmd.mode = 1
    # 0. idle, default stand
    # 1. force stand (controlled by dBodyHeight + ypr)
    # 2. target velocity walking (controlled by velocity + yawSpeed)
    # 3. target position walking (controlled by position + ypr[0])
    # 4. path mode walking (reserve for future release)
    # 5. position stand down.
    # 6. position stand up
    # 7. damping mode
    # 8. recovery stand
    # 9. backflip
    # 10. jumpYaw
    # 11. straightHand
    # 12. dance1
    # 13. dance2
    # 14. two leg stand

    # high_cmd.gaitType = 0
    # 0.idle  1.trot  2.trot running  3.climb stair

    # high_cmd.speedLevel = 0
    # 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3

    # high_cmd.footRaiseHeight = 0.08
    # (unit: m, default: 0.08m), foot up height while walking

    # high_cmd.bodyHeight = 0.28
    # (unit: m, default: 0.28m),

    # high_cmd.postion[0] = 0.0
    # high_cmd.postion[1] = 0.0
    # (unit: m), desired position in inertial frame

    # high_cmd.euler[0] = 0.0     # roll
    high_cmd.pitch = 0.01  # pitch
    # high_cmd.euler[2] = 0.0     # yaw
    # (unit: rad), roll pitch yaw in stand mode

    # high_cmd.velocity[0] = 0.0
    # high_cmd.velocity[1] = 0.0
    # (unit: m/s), forwardSpeed, sideSpeed in body frame

    # high_cmd.yawSpeed = 0.0
    # (unit: rad/s), rotateSpeed in body frame

    high_cmd_publisher.publish(high_cmd)


if __name__ == "__main__":
    loop_rate = 0.0005

    # while 1:
    #   test_high_cmd()
    #   time.sleep(loop_rate)

    start = time.time()
    # yaw_right = Pose_cmd(x=0, y=0, z=0, yaw=-0.3, pitch=0, roll=0, start=0.0, end=6.0)
    # yaw_left = Pose_cmd(x=0, y=0, z=0, yaw=0.3, pitch=0, roll=0, start=6.0, end=12.0)
    # sit = Pose_cmd(x=0, y=0, z=0, yaw=0, pitch=-0.3, roll=0, start=12.0, end=20.0)

    # action_group = Action_Group(poses={yaw_right, yaw_left, sit})

    roll_right = Pose_cmd(x=0, y=0, z=0, yaw=0, pitch=0, roll=0.7, start=0.0, end=1.5)
    roll_left = Pose_cmd(x=0, y=0, z=0, yaw=0, pitch=0, roll=-0.7, start=1.5, end=3)
    # yaw_right2 = Pose_cmd(x=0, y=0, z=0, yaw=-0.2, pitch=0.2, roll=0, start=12.0, end=20.0)
    # yaw_left2 = Pose_cmd(x=0, y=0, z=0, yaw=0.2, pitch=0.2, roll=0, start=20.0, end=28.0)
    # yaw_right3 = Pose_cmd(x=0, y=0, z=0, yaw=-0.2, pitch=0.2, roll=0, start=28.0, end=36.0)
    # reset = Pose_cmd(x=0, y=0, z=0, yaw=0, pitch=0, roll=0, start=36.0, end=42.0)

    # action_group = Action_Group(poses={yaw_right, yaw_left, sit})
    action_group = Action_Group(poses={roll_right, roll_left})  # ,yaw_right2,yaw_left2,yaw_right3,reset})

    action_group.progress(loop_rate)
    desired_pose = action_group.get_current_pose()
    while desired_pose is not None:
        # input("Press button")
        make_pose(final_pose=desired_pose, action_group_elapsed=action_group.elapsed, loop_rate=loop_rate)
        action_group.progress(loop_rate)
        time.sleep(loop_rate)
        desired_pose = action_group.get_current_pose()
