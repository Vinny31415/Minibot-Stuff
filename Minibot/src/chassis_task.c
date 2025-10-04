#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "motor.h"
#include "dji_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

void Chassis_Task_Init()
{
    // Init chassis hardware        
    Motor_Config_t chassis_w1 = {
        .can_bus = 1, // what can bus the motor is on
        .speed_controller_id = 1, // identifier for each motor
        .offset = 0, // Initial offset of the motor (used for encoder)
        .control_mode = VELOCITY_CONTROL, // Control mode of the motor
        .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
        .velocity_pid = // pid
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT, // m2006 is the motor
            },
    };
    DJI_Motor_Handle_t* motor_w1 = DJI_Motor_Init(&chassis_w1, M2006); // Initializing motor 1

    Motor_Config_t chassis_w2 = {
        .can_bus = 1, // what can bus the motor is on
        .speed_controller_id = 2, // identifier for each motor
        .offset = 0, // Initial offset of the motor (used for encoder)
        .control_mode = VELOCITY_CONTROL, // Control mode of the motor
        .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
        .velocity_pid = // pid
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT, // m2006 is the motor
            },
    };
    DJI_Motor_Handle_t* motor_w2 = DJI_Motor_Init(&chassis_w2, M2006); // Initializing motor 2

    Motor_Config_t chassis_w3 = {
        .can_bus = 1, // what can bus the motor is on
        .speed_controller_id = 3, // identifier for each motor
        .offset = 0, // Initial offset of the motor (used for encoder)
        .control_mode = VELOCITY_CONTROL, // Control mode of the motor
        .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
        .velocity_pid = // pid
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT, // m2006 is the motor
            },
    };
    DJI_Motor_Handle_t* motor_w3 = DJI_Motor_Init(&chassis_w3, M2006); // Initializing motor 3

    Motor_Config_t chassis_w4 = {
        .can_bus = 1, // what can bus the motor is on
        .speed_controller_id = 4, // identifier for each motor
        .offset = 0, // Initial offset of the motor (used for encoder)
        .control_mode = VELOCITY_CONTROL, // Control mode of the motor
        .motor_reversal = MOTOR_REVERSAL_NORMAL, // Direction of the motor
        .velocity_pid = // pid
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT, // m2006 is the motor
            },
    };
    DJI_Motor_Handle_t* motor_w4 = DJI_Motor_Init(&chassis_w4, M2006); // Initializing motor 1
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis
}