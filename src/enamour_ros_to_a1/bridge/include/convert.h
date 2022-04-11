/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <ros/ros.h>
#include <algorithm>

// All motor names from champ A1 config. Sorted by unitree engine number (see unitree_legged_sdk/quadruped.h)
static ros::V_string joint_names = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

// STATES

sensor_msgs::Imu LcmToRos(UNITREE_LEGGED_SDK::IMU &lcm_imu, int32_t sequence_number)
{
    sensor_msgs::Imu imu;
    imu.header.seq = sequece_number;
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

    joint_state.header.seq = lcm_low_state.SN;

    joint_state.name = joint_names;

    joint_state.position [12] = {0};
    joint_state.velocity [12] = {0};

    // Add motor state
    for (int i = 0; i < 12; i++)
    {
        joint_state.position[i] = lcm_low_state.motorState[i].q;
        joint_state.velocity[i] = lcm_low_state.motorState[i].dq;
    }

    // TODO Missing Variables
    // joint_state.header.stamp = ;
    // joint_state.header.frame_id = ;
    
    return joint_state;
}

// CMD

UNITREE_LEGGED_SDK::LowCmd RosToLcm(trajectory_msgs::JointTrajectory &ros_joint_trajectory)
{
    UNITREE_LEGGED_SDK::LowCmd low_cmd;

    // Header
    low_cmd.SN = ros_joint_trajectory.header.seq;
    low_cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

    // Set motor values
    for (int i = 0; i < 12; ++i)
    {
        // Get index of joint name
        auto iterator = std::find(joint_names.begin(), joint_names.end(), ros_joint_trajectory.joint_names[i]);
        if (iterator == joint_names.end())
        {
            ROS_ERROR("Joint trajectory names are wrong. Please edit convert.h");
            exit(-1);
        }
        // Calculate index of value
        int index = iterator - joint_names.begin();

        low_cmd.motorCmd[index].mode = 0x0A;

        if (index < ros_joint_trajectory.points[0].positions.size())
        {
            low_cmd.motorCmd[index].q = ros_joint_trajectory.points[0].positions[i];
        }
        if (index < ros_joint_trajectory.points[0].velocities.size())
        {
            low_cmd.motorCmd[index].dq = ros_joint_trajectory.points[0].velocities[i];
        }

        // Default values
        low_cmd.motorCmd[index].tau = 0;
        low_cmd.motorCmd[index].Kd = 0;
        low_cmd.motorCmd[index].Kd = 0;
    }

    // TODO Missing Variables
    // low_cmd.bandWidth;
    // low_cmd.commVersion;
    // low_cmd.crc;
    // low_cmd.led;
    // low_cmd.robotID;
    // low_cmd.wirelessRemote;

    return low_cmd;
}

// Unitree message format

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU &lcm)
{
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    // ros.rpy[0] = lcm.rpy[0];
    // ros.rpy[1] = lcm.rpy[1];
    // ros.rpy[2] = lcm.rpy[2];
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState &lcm)
{
    unitree_legged_msgs::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.ddq = lcm.ddq;
    ros.tauEst = lcm.tauEst;
    ros.q_raw = lcm.q_raw;
    ros.dq_raw = lcm.dq_raw;
    ros.ddq_raw = lcm.ddq_raw;
    ros.temperature = lcm.temperature;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    return ros;
}

UNITREE_LEGGED_SDK::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd &ros, UNITREE_LEGGED_SDK::MotorCmd lcmType)
{
    UNITREE_LEGGED_SDK::MotorCmd lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.tau = ros.tau;
    lcm.Kp = ros.Kp;
    lcm.Kd = ros.Kd;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    lcm.reserve[2] = ros.reserve[2];
    return lcm;
}

unitree_legged_msgs::LowState ToRos(UNITREE_LEGGED_SDK::LowState &lcm)
{
    unitree_legged_msgs::LowState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.imu = ToRos(lcm.imu);
    for (int i = 0; i < 20; i++)
    {
        ros.motorState[i] = ToRos(lcm.motorState[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for (int i = 0; i < 40; i++)
    {
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::LowCmd &ros, UNITREE_LEGGED_SDK::LowCmd lcmType)
{
    UNITREE_LEGGED_SDK::LowCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    for (int i = 0; i < 20; i++)
    {
        lcm.motorCmd[i] = ToLcm(ros.motorCmd[i], lcm.motorCmd[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for (int i = 0; i < 40; i++)
    {
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

#endif // _CONVERT_H_
