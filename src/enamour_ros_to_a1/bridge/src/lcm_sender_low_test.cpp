/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_sdk/a1_const.h>
#include <mutex> // std::mutex, std::lock_guard

using namespace UNITREE_LEGGED_SDK;
using json = nlohmann::json;
using namespace std;

class Custom
{
public:
    Custom(uint8_t level) : mylcm(level)
    {
        cmd.levelFlag = level;
        for (int i = 0; i < 12; i++)
        {
            cmd.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
        }
    }
    void LCMRecv();
    void RobotControl();
    void NewCmd();
    LowCmd getCMD();
    void setCMD(LowCmd newCmd);
    LowCmd create_cmd(json jf);

    LCM mylcm;
    LowCmd cmd = {0};
    LowState state = {0};
    mutex mtx;
};

LowCmd Custom::getCMD()
{
    std::lock_guard<std::mutex> lck(mtx);
    return cmd;
}

void Custom::setCMD(LowCmd newCmd)
{
    std::lock_guard<std::mutex> lck(mtx);
    cmd = newCmd;
}

void Custom::LCMRecv()
{
    mylcm.Recv();
}

void Custom::RobotControl()
{
    // mylcm.Recv();
    // mylcm.Get(state);
    /** printf("State size: %ld, tick: %d, robotid: %d\n", sizeof(state), state.tick, state.robotID);
     printf("%d   %f %f %f\n", state.tick, state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
     printf("%d %d %d %d\n", state.footForce[0], state.footForce[1],state.footForce[2],state.footForce[3]);*/
    // printf("%d\n",cmd.led[0].r);
    auto nextCmd = getCMD();
    mylcm.Send(nextCmd);
}

LowCmd Custom::create_cmd(json jf)
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

void Custom::NewCmd()
{
    cout << "Press enter to send the data";
    cin.ignore();
    // read json file
    std::ifstream ifs("/home/lars/Desktop/project/src/enamour_ros_to_a1/bridge/src/test.json");
    json jf = json::parse(ifs);
    ifs.close();
    LowCmd newCmd = create_cmd(jf);
    setCMD(newCmd);
}

int main(void)
{
    Custom custom(LOWLEVEL);
    InitEnvironment();
    // custom.mylcm.SubscribeCmd();
    custom.mylcm.SubscribeState();
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));
    LoopFunc loop_control("control_loop", 0.002, 3, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_cmd("cmd_loop", 0.5, 3, boost::bind(&Custom::NewCmd, &custom));

    loop_lcm.start();
    loop_control.start();
    loop_cmd.start();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
