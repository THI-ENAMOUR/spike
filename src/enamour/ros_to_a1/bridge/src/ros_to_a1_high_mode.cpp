#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "std_msgs/String.h"

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
    void handleRosCommand(const unitree_legged_msgs::HighCmd &msg);
    UNITREE_LEGGED_SDK::HighCmd SendHighLCM;
    TLCM roslcm;
    unitree_legged_msgs::HighCmd SendHighROS;
};

template <typename TCmd, typename TLCM>
void RosCommandHandler<TCmd, TLCM>::handleRosCommand(const unitree_legged_msgs::HighCmd &msg)
{
    SendHighROS = msg;
    ROS_INFO("High Command received");
    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    printf("High_cmd body_height %f\n", SendHighLCM.bodyHeight);
    printf("High_cmd roll: %f\n", SendHighLCM.roll);
    printf("High_cmd pitch: %f\n", SendHighLCM.pitch);
    printf("High_cmd yaw: %f\n", SendHighLCM.yaw);
    roslcm.Send(SendHighLCM);
}

template <typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    RosCommandHandler<TCmd, TLCM> rcm{roslcm};
    ros::Subscriber sub = n.subscribe(
        "/high_command",
        1000,
        &RosCommandHandler<TCmd, TLCM>::handleRosCommand,
        &rcm);

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    ros::Publisher highState_pub = n.advertise<unitree_legged_msgs::HighState>("/high_state", 1000);

    while (ros::ok())
    {
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        highState_pub.publish(RecvHighROS);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_to_a1_high_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}
