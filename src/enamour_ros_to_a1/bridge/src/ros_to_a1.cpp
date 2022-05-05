#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

template <typename TLCM>
void *update_loop(void *param)
{
    TLCM *data = (TLCM *)param;
    while (ros::ok)
    {
        data->Recv();
        usleep(2000);
    }
}

template <typename TCmd, typename TLCM>
class RosCommandHandler
{
public:
    RosCommandHandler(TLCM &roslcm) : roslcm(roslcm)
    {
    }
    void handleRosCommand(const trajectory_msgs::JointTrajectory &msg);
    UNITREE_LEGGED_SDK::LowCmd SendLowLCM;
    TLCM roslcm;
    TCmd SendLowROS;
};

template <typename TCmd, typename TLCM>
void RosCommandHandler<TCmd, TLCM>::handleRosCommand(const trajectory_msgs::JointTrajectory &msg)
{
    SendLowROS = msg;
    ROS_INFO("Command received");
    SendLowLCM = RosToLcm(SendLowROS);
    printf("Low_Cmd position [0]: %f\n", SendLowLCM.motorCmd[0].q);
    printf("Low_Cmd position [1]: %f\n", SendLowLCM.motorCmd[1].q);
    printf("Low_Cmd position [2]: %f\n", SendLowLCM.motorCmd[2].q);
    roslcm.Send(SendLowLCM);
}

template <typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    TState RecvLowLCM = {0};
    sensor_msgs::Imu RecvImuROS;
    UNITREE_LEGGED_SDK::IMU RecvImuLCM;
    sensor_msgs::JointState RecvStateROS;

    RosCommandHandler<TCmd, TLCM> rcm{roslcm};
    // Todo: Subscribe to relative path if possible
    // Todo: Define optimal length of buffer
    ros::Subscriber sub = n.subscribe(
        "/joint_group_position_controller/command",
        1000,
        &RosCommandHandler<TCmd, TLCM>::handleRosCommand,
        &rcm);
    // Todo: Define optimal length of buffer
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1000);
    // Todo: Define optimal length of buffer
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    roslcm.SubscribeState();
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (ros::ok())
    {
        roslcm.Get(RecvLowLCM);
        RecvImuLCM = RecvLowLCM.imu;

        // Send status only when robot is powered up.
        if (RecvLowLCM.SN != 0)
        {
            RecvImuROS = LcmToRos(RecvImuLCM);
            RecvStateROS = LcmToRos(RecvLowLCM);
            imu_pub.publish(RecvImuROS);
            joint_state_pub.publish(RecvStateROS);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_to_real_bridge");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    UNITREE_LEGGED_SDK::LeggedType rname;
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
    // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
    // UNITREE_LEGGED_SDK::InitEnvironment();

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<trajectory_msgs::JointTrajectory, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}