#include <ros/ros.h>
#include <string>
//#include <std_msgs/String.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

template<typename TLCM>
void *update_loop(void *param){
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TLCM>
class RosCommandHandler{
public:
    RosCommandHandler(TLCM &roslcm) : roslcm(roslcm)
    {}
    void handleRosCommand(const unitree_legged_msgs::LowCmd& msg);
private:
    TLCM roslcm;
    unitree_legged_msgs::LowCmd SendLowROS;
    TCmd SendLowLCM = {0};
};

void RosCommandHandler::handleRosCommand(const unitree_legged_msgs::LowCmd& msg){
    SendLowROS = *msg;
    ROS_INFO("Command received");
    SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
    roslcm.Send(SendLowLCM);
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm){
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowState RecvLowROS;

    RosCommandHandler rcm(roslcm);
    //Todo: Subscribe to relative path if possible
    //Todo: Define optimal length of buffer
    ros::Subscriber sub = n.subscribe(
        "/joint_group_position_controller/command", 
        1000, 
        &RosCommandHandler::handleRosCommand,
        &rcm
    );
    //Todo: Define optimal length of buffer
    ros::Publisher imu_pub = 
        n.advertise<unitree_legged_msgs::LowState>("/imu/data", 1000);
    //Todo: Define optimal length of buffer
    ros::Publisher joint_state_pub = 
        n.advertise<unitree_legged_msgs::LowState>("/joint_states", 1000);

    roslcm.SubscribeState();
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    for(int i=0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;
        SendLowROS.motorCmd[i].q = PosStopF;
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    while(ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FR_2 position: %f\n",  RecvLowROS.motorState[FR_2].q);

        imu_pub.publish(RecvLowROS.imu);
        joint_state_pub.publish(RecvLowROS.motorState);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "ros_to_real_bridge");
    std::string firmwork;
    ros::param::get("/firmwork")

    UNITREE_LEGGED_SDK::LeggedType rname;
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
    // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
    // UNITREE_LEGGED_SDK::InitEnvironment();

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}