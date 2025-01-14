/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.10", 8007)
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3] = {0};
    float qInit_fl[3] = {0};
    float qInit_rr[3] = {0};
    float qInit_rl[3] = {0};
    float qDes[3] = {0};
    float qDes_fl[3] = {0};
    float qDes_rr[3]= {0};
    float qDes_rl[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);
    printf("%d  %f  %f\n", motiontime, state.motorState[FL_1].q, state.motorState[FL_1].dq);


    // gravity compensation
    cmd.motorCmd[FR_0].tau = -5.0f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 100){
    if (motiontime >= 0)
    {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if (motiontime >= 0 && motiontime < 10)
        {
            //front right
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;

            //front left
            qInit_fl[0] = state.motorState[FL_0].q;
            qInit_fl[1] = state.motorState[FL_1].q;
            qInit_fl[2] = state.motorState[FL_2].q;

            //rear right
            qInit_rr[0] = state.motorState[RR_0].q;
            qInit_rr[1] = state.motorState[RR_1].q;
            qInit_rr[2] = state.motorState[RR_2].q;

            //rear left
            qInit_rl[0] = state.motorState[RL_0].q;
            qInit_rl[1] = state.motorState[RL_1].q;
            qInit_rl[2] = state.motorState[RL_2].q;

        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if (motiontime >= 10 && motiontime < 400)
        {
            rate_count++;
            double rate = rate_count / 200.0; // needs count to 200
            // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
            // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            Kp[0] = 20.0;
            Kp[1] = 20.0;
            Kp[2] = 20.0;
            Kd[0] = 2.0;
            Kd[1] = 2.0;
            Kd[2] = 2.0;

            //front right
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
            //front left
            qDes_fl[0] = jointLinearInterpolation(qInit_fl[0], sin_mid_q[0], rate);
            qDes_fl[1] = jointLinearInterpolation(qInit_fl[1], sin_mid_q[1], rate);
            qDes_fl[2] = jointLinearInterpolation(qInit_fl[2], sin_mid_q[2], rate);
             //rear right
            qDes_rr[0] = jointLinearInterpolation(qInit_rr[0], sin_mid_q[0], rate);
            qDes_rr[1] = jointLinearInterpolation(qInit_rr[1], sin_mid_q[1], rate);
            qDes_rr[2] = jointLinearInterpolation(qInit_rr[2], sin_mid_q[2], rate);
            //rear left
            qDes_rl[0] = jointLinearInterpolation(qInit_rl[0], sin_mid_q[0], rate);
            qDes_rl[1] = jointLinearInterpolation(qInit_rl[1], sin_mid_q[1], rate);
            qDes_rl[2] = jointLinearInterpolation(qInit_rl[2], sin_mid_q[2], rate);


        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        float freq_Hz = 1;
        // float freq_Hz = 5;
        float freq_rad = freq_Hz * 2 * M_PI;
        float t = dt * sin_count;
        if (motiontime >= 400)
        {
            sin_count++;
            // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
            sin_joint1 = 0.6 * sin(t * freq_rad);
            sin_joint2 = -0.9 * sin(t * freq_rad);

            //front right
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;

            //front left
            qDes_fl[0] = sin_mid_q[0];
            qDes_fl[1] = sin_mid_q[1] + sin_joint1;
            qDes_fl[2] = sin_mid_q[2] + sin_joint2;

            //rear right
            qDes_rr[0] = sin_mid_q[0];
            qDes_rr[1] = sin_mid_q[1] + sin_joint1;
            qDes_rr[2] = sin_mid_q[2] + sin_joint2;

            //rear left
            qDes_rl[0] = sin_mid_q[0];
            qDes_rl[1] = sin_mid_q[1] + sin_joint1;
            qDes_rl[2] = sin_mid_q[2] + sin_joint2;





            // qDes[2] = sin_mid_q[2];


        }

        cmd.motorCmd[FR_0].q = qDes[0];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp[0];
        cmd.motorCmd[FR_0].Kd = Kd[0];
        cmd.motorCmd[FR_0].tau = -4.0f;

        cmd.motorCmd[FR_1].q = qDes[1];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp[1];
        cmd.motorCmd[FR_1].Kd = Kd[1];
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q = qDes[2];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp[2];
        cmd.motorCmd[FR_2].Kd = Kd[2];
        cmd.motorCmd[FR_2].tau = 0.0f;
        
        // FL
        cmd.motorCmd[FL_0].q = qDes_fl[0];
        cmd.motorCmd[FL_0].dq = 0;
        cmd.motorCmd[FL_0].Kp = Kp[0];
        cmd.motorCmd[FL_0].Kd = Kd[0];
        cmd.motorCmd[FL_0].tau = -4.0f;

        cmd.motorCmd[FL_1].q = qDes_fl[1];
        cmd.motorCmd[FL_1].dq = 0;
        cmd.motorCmd[FL_1].Kp = Kp[1];
        cmd.motorCmd[FL_1].Kd = Kd[1];
        cmd.motorCmd[FL_1].tau = 0.0f;

        cmd.motorCmd[FL_2].q = qDes_fl[2];
        cmd.motorCmd[FL_2].dq = 0;
        cmd.motorCmd[FL_2].Kp = Kp[2];
        cmd.motorCmd[FL_2].Kd = Kd[2];
        cmd.motorCmd[FL_2].tau = 0.0f;

        //RR
        cmd.motorCmd[RR_0].q = qDes_rr[0];
        cmd.motorCmd[RR_0].dq = 0;
        cmd.motorCmd[RR_0].Kp = Kp[0];
        cmd.motorCmd[RR_0].Kd = Kd[0];
        cmd.motorCmd[RR_0].tau = -4.0f;

        cmd.motorCmd[RR_1].q = qDes_rr[1];
        cmd.motorCmd[RR_1].dq = 0;
        cmd.motorCmd[RR_1].Kp = Kp[1];
        cmd.motorCmd[RR_1].Kd = Kd[1];
        cmd.motorCmd[RR_1].tau = 0.0f;

        cmd.motorCmd[RR_2].q = qDes_rr[2];
        cmd.motorCmd[RR_2].dq = 0;
        cmd.motorCmd[RR_2].Kp = Kp[2];
        cmd.motorCmd[RR_2].Kd = Kd[2];
        cmd.motorCmd[RR_2].tau = 0.0f;
        
        // RL
        cmd.motorCmd[RL_0].q = qDes_rl[0];
        cmd.motorCmd[RL_0].dq = 0;
        cmd.motorCmd[RL_0].Kp = Kp[0];
        cmd.motorCmd[RL_0].Kd = Kd[0];
        cmd.motorCmd[RL_0].tau = -4.0f;

        cmd.motorCmd[RL_1].q = qDes_rl[1];
        cmd.motorCmd[RL_1].dq = 0;
        cmd.motorCmd[RL_1].Kp = Kp[1];
        cmd.motorCmd[RL_1].Kd = Kd[1];
        cmd.motorCmd[RL_1].tau = 0.0f;

        cmd.motorCmd[RL_2].q = qDes_rl[2];
        cmd.motorCmd[RL_2].dq = 0;
        cmd.motorCmd[RL_2].Kp = Kp[2];
        cmd.motorCmd[RL_2].Kd = Kd[2];
        cmd.motorCmd[RL_2].tau = 0.0f;
        // cmd.motorCmd[FR_2].tau = 2 * sin(t*freq_rad);
    }

    // if(motiontime > 10){
    //     // safe.PositionLimit(cmd);
    //     safe.PowerProtect(cmd, state, 1);
    //     // safe.PositionProtect(cmd, state, 0.087);
    // }

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}
