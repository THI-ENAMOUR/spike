/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>

using namespace UNITREE_LEGGED_SDK;

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

    LCM mylcm;
    LowCmd cmd = {0};
    LowState state = {0};
};

void Custom::LCMRecv()
{
    mylcm.Recv();
}

void Custom::RobotControl()
{
    // mylcm.Recv();
    mylcm.Get(state);
    auto imu = state.imu;
    auto motor = state.motorState;

    /*
    auto motor_FL_1 = state.motorState[FL_1];
    auto motor_FL_2 = state.motorState[FL_2];

    auto motor_FR_0 = state.motorState[FR_0];
    auto motor_FR_1 = state.motorState[FR_1];
    auto motor_FR_2 = state.motorState[FR_2];

    auto motor_RL_0 = state.motorState[RL_0];
    auto motor_RL_1 = state.motorState[RL_1];
    auto motor_RL_2 = state.motorState[RL_2];

    auto motor_RR_0 = state.motorState[RR_0];
    auto motor_RR_1 = state.motorState[RR_1];
    auto motor_RR_2 = state.motorState[RR_2];
    */

    // printf("State size: %ld, tick: %d, robotid: %d\n", sizeof(state), state.tick, state.robotID);
    // printf("%d   %f %f %f\n", state.tick, state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
    // printf("%d %d %d %d\n", state.footForce[0], state.footForce[1],state.footForce[2],state.footForce[3]);
    printf("################ LOW STATE ################\n");
    printf("levelFlag: %d \n", state.levelFlag);
    printf("commVersion: %d \n", state.commVersion);
    printf("robotID: %d \n", state.robotID);
    printf("SN: %d \n", state.SN);
    printf("bandWidth: %d \n", state.bandWidth);
    printf("tick: %d \n", state.tick);
    printf("reserve: %d \n", state.reserve);
    printf("crc: %d \n", state.crc);

    printf("footForce[0]: %d \n", state.footForce[0]);
    printf("footForce[1]: %d \n", state.footForce[1]);
    printf("footForce[2]: %d \n", state.footForce[2]);
    printf("footForce[3]: %d \n", state.footForce[3]);

    printf("footForceEst[0]: %d \n", state.footForceEst[0]);
    printf("footForceEst[1]: %d \n", state.footForceEst[1]);
    printf("footForceEst[2]: %d \n", state.footForceEst[2]);
    printf("footForceEst[3]: %d \n", state.footForceEst[3]);

    printf("################ IMU ################\n");
    printf("quaternion[0]: %f \n", imu.quaternion[0]);
    printf("quaternion[1]: %f \n", imu.quaternion[1]);
    printf("quaternion[2]: %f \n", imu.quaternion[2]);
    printf("quaternion[3]: %f \n", imu.quaternion[3]);

    printf("gyroscope[0]: %f \n", imu.gyroscope[0]);
    printf("gyroscope[1]: %f \n", imu.gyroscope[1]);
    printf("gyroscope[2]: %f \n", imu.gyroscope[2]);

    printf("rpy[0]: %f \n", imu.rpy[0]);
    printf("rpy[1]: %f \n", imu.rpy[1]);
    printf("rpy[2]: %f \n", imu.rpy[2]);

    printf("temperature: %d \n", imu.temperature);

    for (int i = 0; i < 12; ++i)
    {
        printf("################ MOTOR STATE %d ################\n", i);
        printf("mode: %d \n", motor[i].mode);
        printf("dq: %f \n", motor[i].dq);
        printf("ddq: %f \n", motor[i].ddq);
        printf("q_raw: %f \n", motor[i].q_raw);
        printf("dq_raw: %f \n", motor[i].dq_raw);
        printf("ddq_raw: %f \n", motor[i].ddq_raw);
        printf("temperature: %d \n", motor[i].temperature);
        printf("reserve[0]: %d \n", motor[i].reserve[0]);
        printf("reserve[1]: %d \n", motor[i].reserve[1]);
    }
    printf("\n######################################\n");
    printf("################ NEXT ################\n");
    printf("######################################\n\n");

    /*
       printf("################ MOTOR STATE FL ################\n");
       printf("mode: %d \n",motor_FL_0.q);
       printf("dq: %d \n",motor_FL_0.dq);
       printf("ddq: %d \n",motor_FL_0.ddq);
       printf("q_raw: %d \n",motor_FL_0.q_raw);
       printf("dq_raw: %d \n",motor_FL_0.dq_raw);
       printf("ddq_raw: %d \n",motor_FL_0.ddq_raw);
       printf("temperature: %d \n",motor_FL_0.temperature);
       printf("reserve[0]: %d \n",motor_FL_0.reserve[0]);
       printf("reserve[1]: %d \n",motor_FL_0.reserve[1]);
       printf("---------------------------------");
       */

    mylcm.Send(cmd);
}

int main(void)
{

    Custom custom(LOWLEVEL);
    InitEnvironment();
    // custom.mylcm.SubscribeCmd();
    custom.mylcm.SubscribeState();
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));
    LoopFunc loop_control("control_loop", 0.002, 3, boost::bind(&Custom::RobotControl, &custom));
    loop_lcm.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
