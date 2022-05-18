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
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
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

    joint_state.position.resize(12);

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
            printf("%s \n", ros_joint_trajectory.joint_names[i].c_str());
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
        // If the values are too low, the robot does not move. If they are too high, the robot shuts down.
        if (joint_element == 0)
        {
            // kd and kp for hip
            // Tested Values: Kp: 35 - 50; Kd: 1 - 5;
            joint_kp = 50.0;
            joint_kd = 1.0;
        }
        else if (joint_element == 1)
        {
            // kd and kp for thigh
            // Tested Values: Kp: 35 - 50; Kd: 1 - 5;
            joint_kp = 50.0;
            joint_kd = 1.0;
        }
        else if (joint_element == 2)
        {
            // kd and kp for calf
            // Tested Values: Kp: 35 - 50; Kd: 1 - 5;
            joint_kp = 50.0;
            joint_kd = 1.0;
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

unitree_legged_msgs::IMU ToRos(const UNITREE_LEGGED_SDK::IMU &lcm)
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
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::Cartesian ToRos(const UNITREE_LEGGED_SDK::Cartesian &lcm)
{
    unitree_legged_msgs::Cartesian ros;
    ros.x = lcm.x;
    ros.y = lcm.y;
    ros.z = lcm.z;
    return ros;
}

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState &lcm)
{
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.mode = lcm.mode;
    ros.progress = lcm.progress;
    ros.imu = ToRos(lcm.imu);
    ros.gaitType = lcm.gaitType;
    ros.footRaiseHeight = lcm.footRaiseHeight;
    ros.bodyHeight = lcm.bodyHeight;
    ros.yawSpeed = lcm.yawSpeed;
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;

    for (int i(0); i < 3; ++i)
    {
        ros.position[i] = lcm.position[i];
        ros.velocity[i] = lcm.velocity[i];
    }

    for (int i(0); i < 4; ++i)
    {
        ros.footPosition2Body[i] = ToRos(lcm.footPosition2Body[i]);
        ros.footSpeed2Body[i] = ToRos(lcm.footSpeed2Body[i]);
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }

    // for(int i(0); i<20; ++i){
    //     ros.temperature[i] = lcm.temperature[i];
    // }

    for (int i(0); i < 40; ++i)
    {
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }

    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros, UNITREE_LEGGED_SDK::HighCmd lcmType)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.mode = ros.mode;
    lcm.gaitType = ros.gaitType;
    lcm.speedLevel = ros.speedLevel;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.yawSpeed = ros.yawSpeed;
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;

    for(int i(0); i<2; ++i){
        lcm.postion[i] = ros.postion[i];
        lcm.velocity[i] = ros.velocity[i];
    }

    for(int i(0); i<3; ++i){
        lcm.euler[i] = ros.euler[i];
    }

    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }

    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }

    return lcm;
}

#endif // _CONVERT_H_
