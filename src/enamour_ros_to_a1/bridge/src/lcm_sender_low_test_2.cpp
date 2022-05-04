/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

#include <nlohmann/json.hpp>
#include <mutex> // std::mutex, std::lock_guard

using namespace UNITREE_LEGGED_SDK;
using json = nlohmann::json;
using namespace std;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(1){ //ros::ok){
        data->Recv();
        usleep(2000);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

LowCmd createCmd(json jf)
{
    LowCmd cmd = {0};
    cmd.levelFlag = jf["levelFlag"];
    cmd.commVersion = jf["commVersion"];
    cmd.robotID = jf["robotID"];
    cmd.SN = jf["SN"];
    cmd.bandWidth = jf["bandWidth"];
    cmd.reserve = jf["reserve"];
    cmd.crc = jf["crc"];
    auto motor = jf["MotorCmd"];

    for (int i = 0; i < 12; ++i)
    {
        cmd.motorCmd[i].mode = motor[i]["mode"];

        auto rad = motor[i]["q"];
        // Hip
        if (i == FR_0 || i == FL_0 || i == RR_0 || i == RL_0)
        {
            if (rad > UNITREE_LEGGED_SDK::a1_Hip_max)
            {
                rad = a1_Hip_max;
            }
            if (rad < a1_Hip_min)
            {
                rad = a1_Hip_min;
            }
        }
        // Thigh
        else if (i == FR_1 || i == FL_1 || i == RR_1 || i == RL_1)
        {
            if (rad > a1_Thigh_max)
            {
                rad = a1_Thigh_max;
            }
            if (rad < a1_Thigh_min)
            {
                rad = a1_Thigh_min;
            }
        }
        // calf
        else if (i == FR_2 || i == FL_2 || i == RR_2 || i == RL_2)
        {
            if (rad > a1_Calf_max)
            {
                rad = a1_Calf_max;
            }
            if (rad < a1_Calf_min)
            {
                rad = a1_Calf_min;
            }
        }

        cmd.motorCmd[i].q = rad;
        cmd.motorCmd[i].dq = motor[i]["dq"];
        cmd.motorCmd[i].tau = motor[i]["tau"];
        cmd.motorCmd[i].Kp = motor[i]["Kp"];
        cmd.motorCmd[i].Kd = motor[i]["Kd"];
        cmd.motorCmd[i].reserve[0] = motor[i]["reserve"][0];
        cmd.motorCmd[i].reserve[1] = motor[i]["reserve"][1];
        cmd.motorCmd[i].reserve[2] = motor[i]["reserve"][2];
    }

    for (int i = 0; i < 4; ++i)
    {
        cmd.led[i].r = jf["LED"][i]["r"];
        cmd.led[i].g = jf["LED"][i]["g"];
        cmd.led[i].b = jf["LED"][i]["b"];
    }

    return cmd;
}

void newCmd()
{
    cout << "Press enter to send the data";
    cin.ignore();
    // read json file
    std::ifstream ifs("/home/lars/Desktop/project/src/enamour_ros_to_a1/bridge/src/test.json");
    json jf = json::parse(ifs);
    ifs.close();
    LowCmd newCmd = createCmd(jf);
    setCMD(newCmd);
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // ros::NodeHandle n;
    // ros::Rate loop_rate(500);

    Safety safe;
    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    //TCmd SendLowLCM = {0};
    // TState RecvLowLCM = {0};
    // unitree_legged_msgs::LowCmd SendLowROS;
    // unitree_legged_msgs::LowState RecvLowROS;
    LowCmd cmd = {0};
    LowState state = {0};

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    cmd.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        cmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = PosStopF;  // 禁止位置环 disable position loop
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = VelStopF;  // 禁止速度环 disable velocity loop
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }

    while (1){ //(ros::ok){
        roslcm.Get(state);
        // RecvLowROS = ToRos(RecvLowLCM);
        printf("FR_2 position: %f\n",  state.motorState[FR_2].q);

        if(initiated_flag){
          newCmd();
          safe.PositionLimit(cmd);
          safe.PowerProtect(cmd, state, 1);
        }

        // SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(cmd);
        // ros::spinOnce();
        // loop_rate.sleep();

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
        }
    }
    return 0;
}

int main(int argc, char *argv[]){
    // ros::init(argc, argv, "position_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}
