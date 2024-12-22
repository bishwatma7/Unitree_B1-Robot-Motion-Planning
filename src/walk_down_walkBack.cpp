//This code will make the robot walk forwards, go down, stand up, turn around and walk back.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.220", 8082),
                            startTime(high_resolution_clock::now()) 
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    time_point<high_resolution_clock> startTime;
    float dt = 0.002; // 0.001~0.01
    bool running = true; // Control variable to stop loops
};

void Custom::UDPRecv()
{
    if (running) udp.Recv();
}

void Custom::UDPSend()
{
    if (running) udp.Send();
}

void Custom::RobotControl()
{
    if (!running) return;

    auto now = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(now - startTime).count();

    udp.GetRecv(state);
    printf("%ld   %f\n", duration, state.imu.rpy[2]);

    cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    if (duration > 0 && duration < 2000) 
    {
        cmd.mode = 1; //wait
    }
    else if(duration >= 2000 && duration < 7000) 
    {
        cmd.mode = 2; //walk forwards
        cmd.velocity[0]=0.5;
        cmd.yawSpeed = 0;
    }
    //---------------------------------------------------------------------------
    else if (duration >= 7000 && duration < 10000)
    {
        cmd.mode = 0;  //stop
        cmd.velocity[0]=0;
        cmd.yawSpeed = 0;
    }
   
    else if(duration >= 10000 && duration < 12000)
    {
        cmd.mode = 6; //get ready to sit down
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0;
    }
    else if(duration >= 12000 && duration < 15000)
    {
        cmd.mode = 5; //sit down
        cmd.bodyHeight = 0;
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0;
    }
      
     else if(duration >= 15000 && duration < 17000)
    {
        cmd.mode = 7; //get ready to stand up
        cmd.bodyHeight = 0;
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0;
    }
    else if(duration >= 17000 && duration < 22000)
    {
        cmd.mode = 6;//stand up
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0;
    }
     else if(duration >= 22000 && duration < 24000)
    {
        cmd.mode = 1; //get ready to walk again
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0;
    }

    //----------------------------------------------------------------------------
    else if(duration >= 24000 && duration < 28000) 
    {
        cmd.mode = 2; //turn around
        cmd.velocity[0] = 0;
        cmd.yawSpeed = 0.98;
    }
    else if(duration >= 28000 && duration < 33000)
    {
        cmd.mode = 2; //walk back
        cmd.velocity[0]=0.5;
        cmd.yawSpeed = 0;
    } 
    else 
    {
        cmd.mode=0; 
        running = false; // Stop loops after this duration
    }
    
    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    auto start = std::chrono::high_resolution_clock::now();
    while (custom.running)
    {
        sleep(1);
    }

    return 0;
}
