/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <ros/ros.h>
#include <algorithm>

// All motor names from champ A1 config. Sorted by unitree joint number (see unitree_legged_sdk/quadruped.h)
// eg. FR_hip_joint = 0, FR_thigh_joint = 1, etc.
static ros::V_string joint_names = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

// STATES
sensor_msgs::Imu LcmToRos(UNITREE_LEGGED_SDK::IMU &lcm_imu)
{
    sensor_msgs::Imu imu;

    // LCM Order w, x, y, z
    imu.orientation.w = lcm_imu.quaternion[0];
    imu.orientation.x = lcm_imu.quaternion[1];
    imu.orientation.y = lcm_imu.quaternion[2];
    imu.orientation.z = lcm_imu.quaternion[3];

    imu.angular_velocity.x = lcm_imu.gyroscope[0];
    imu.angular_velocity.y = lcm_imu.gyroscope[1];
    imu.angular_velocity.z = lcm_imu.gyroscope[2];

    imu.linear_acceleration.x = lcm_imu.accelerometer[0];
    imu.linear_acceleration.y = lcm_imu.accelerometer[1];
    imu.linear_acceleration.z = lcm_imu.accelerometer[2];
    return imu;
}

sensor_msgs::JointState LcmToRos(UNITREE_LEGGED_SDK::LowState &lcm_low_state)
{
    sensor_msgs::JointState joint_state;

    joint_state.name = joint_names;

    joint_state.position[12] = {0};

    // Add motor state
    for (int i = 0; i < 12; ++i)
    {
        joint_state.position[i] = lcm_low_state.motorState[i].q;
    }

    // TODO Missing Variables
    // joint_state.header.stamp = ;
    // joint_state.header.frame_id = ;

    return joint_state;
}

// CMD
UNITREE_LEGGED_SDK::LowCmd RosToLcm(trajectory_msgs::JointTrajectory &ros_joint_trajectory)
{
    UNITREE_LEGGED_SDK::LowCmd low_cmd = {0};

    // Set motor values
    for (int i = 0; i < 12; ++i)
    {
        // Get index of joint name.
        // Map the joint name of the champ framework to the robot joint index.
        auto iterator = std::find(joint_names.begin(), joint_names.end(), ros_joint_trajectory.joint_names[i]);
        if (iterator == joint_names.end())
        {
            ROS_ERROR("Joint trajectory names are wrong. Please check the simulation settings or edit the convert.h in the ros-to-a1 package");
            exit(-1);
        }
        // Calculate index of value and array size
        int joint_index = iterator - joint_names.begin();
        int joint_trajectory_length = ros_joint_trajectory.points[0].positions.size();

        if (joint_index < joint_trajectory_length)
        {
            low_cmd.motorCmd[joint_index].q = ros_joint_trajectory.points[0].positions[i];
        }

        int joint_kp = 0;
        int joint_kd = 0;
        // Each leg has three elements (hip, thigh, calf)
        int joint_element = i % 3;

        // Set values for kd (velocity stiffness) and kp (position stiffness) of each joint.
        if (joint_element == 0)
        {
            // kd and kp for hip
            joint_kp = 70.0;
            joint_kd = 3.0;
        }
        else if (joint_element == 1)
        {
            // kd and kp for thigh
            joint_kp = 180.0;
            joint_kd = 8.0;
        }
        else if (joint_element == 2)
        {
            // kd and kp for calf
            joint_kp = 300.0;
            joint_kd = 15.0;
        }

        low_cmd.motorCmd[joint_index].Kp = joint_kp;
        low_cmd.motorCmd[joint_index].Kd = joint_kd;

        // Servo mode
        low_cmd.motorCmd[joint_index].mode = 0x0A;
    }

    // Header
    low_cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

    return low_cmd;
}

#endif // _CONVERT_H_
