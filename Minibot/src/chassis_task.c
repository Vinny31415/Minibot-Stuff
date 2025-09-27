#include "chassis_task.h"

#include "robot.h"
#include "remote.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

void Chassis_Task_Init()
{
    // Init chassis hardware
    printf("hello\n");
    printf("hello again\n");
    printf("Adele\n");
    printf("Hello from the other side\n");
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis
}