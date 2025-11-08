#include "chassis_task.h"
#include <math.h>
#include "robot.h"
#include "remote.h"
#include "motor.h"
#include "dji_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad = 0.035; // radius of the chassis wheels
DJI_Motor_Handle_t* motor_w1;
DJI_Motor_Handle_t* motor_w2;
DJI_Motor_Handle_t* motor_w3;
DJI_Motor_Handle_t* motor_w4;


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
                .output_limit = M2006_MAX_CURRENT_INT, // m2006 is the motor
            },
    };
    motor_w1 = DJI_Motor_Init(&chassis_w1, M2006); // Initializing motor 1

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
                .output_limit = M2006_MAX_CURRENT_INT, // m2006 is the motor
            },
    };
    motor_w2 = DJI_Motor_Init(&chassis_w2, M2006); // Initializing motor 2

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
                .output_limit = M2006_MAX_CURRENT_INT, // m2006 is the motor
            },
    };
    motor_w3 = DJI_Motor_Init(&chassis_w3, M2006); // Initializing motor 3

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
                .output_limit = M2006_MAX_CURRENT_INT, // m2006 is the motor
            },
    };
    motor_w4 = DJI_Motor_Init(&chassis_w4, M2006); // Initializing motor 1
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis

    float d = .26;        
    float rad = PI/4;
    float omega = 0.0;
    // we've swapped x and y somehow
    float dy = g_robot_state.input.vx;    
    float dx = g_robot_state.input.vy;
    // and the 'y' inputs are inverted
    dx *= -1;

    float Phi1 = -sin(rad)*dx + cos(rad)*dy + omega*d;
    float Phi2 = -cos(rad)*dx - sin(rad)*dy + omega*d;
    float Phi3 =  sin(rad)*dx - cos(rad)*dy + omega*d;
    float Phi4 =  cos(rad)*dx + sin(rad)*dy + omega*d;

    DJI_Motor_Set_Velocity(motor_w1, Phi1*(1/chassis_rad));
    DJI_Motor_Set_Velocity(motor_w2, Phi2*(1/chassis_rad));
    DJI_Motor_Set_Velocity(motor_w3, Phi3*(1/chassis_rad));
    DJI_Motor_Set_Velocity(motor_w4, Phi4*(1/chassis_rad));
}
