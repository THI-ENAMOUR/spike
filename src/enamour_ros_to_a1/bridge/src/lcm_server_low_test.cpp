/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : udp(level), mylcm(level){}
    void UDPRecv(){
        udp.Recv();
    }
    void UDPSend(){
        udp.Send();
    }
    void LCMRecv();
    void RobotControl() ;

    UDP udp;
    LCM mylcm;
    LowCmd cmd = {0};
    LowState state = {0};
    u_int32_t error = 0;
};

void Custom::LCMRecv()
{
    if(mylcm.lowCmdLCMHandler.isrunning){
        pthread_mutex_lock(&mylcm.lowCmdLCMHandler.countMut);
        mylcm.lowCmdLCMHandler.counter++;
        if(mylcm.lowCmdLCMHandler.counter > 20){
            printf("counter: %d, error: %d\n",mylcm.lowCmdLCMHandler.counter, ++error);
        }
        if(mylcm.lowCmdLCMHandler.counter > 1000){
            printf("Error! LCM Time out.\n");
            exit(-1);              // can be commented out
        }
        pthread_mutex_unlock(&mylcm.lowCmdLCMHandler.countMut);
    }
    mylcm.Recv();
}

void Custom::RobotControl() 
{
    udp.GetRecv(state);
    mylcm.Send(state);
    mylcm.Get(cmd);
    auto motor = cmd.motorCmd[0];
    printf("motor FL_0 - q: %f \n",motor.q);
    printf("motor FL_0 - kd: %f \n",motor.Kd);
    printf("motor FL_0 - kp: %f \n",motor.Kp);
    motor = cmd.motorCmd[1];
    printf("motor FL_1 - q: %f \n",motor.q);
    printf("motor FL_1 - kd: %f \n",motor.Kd);
    printf("motor FL_1 - kp: %f \n",motor.Kp);
    motor = cmd.motorCmd[2];
    printf("motor FL_2 - q: %f \n",motor.q);
    printf("motor FL_2 - kd: %f \n",motor.Kd);
    printf("motor FL_2 - kp: %f \n",motor.Kp);
    udp.SetSend(cmd);
}

int main(void) 
{
    
    Custom custom(LOWLEVEL);
    // InitEnvironment();
    custom.mylcm.SubscribeCmd();

    LoopFunc loop_control("control_loop", 0.002, 3, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("UDP_Send", 0.002, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("UDP_Recv", 0.002, 3, boost::bind(&Custom::UDPRecv, &custom));
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_lcm.start();
    loop_control.start();

    while(1){
        sleep(10);
    }

    return 0; 
}
