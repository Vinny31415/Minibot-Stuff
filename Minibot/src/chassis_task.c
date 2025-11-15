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
    float d = 0.26f;
    float R = chassis_rad;

    float dx = g_robot_state.input.vx;
    float dy = g_robot_state.input.vy;
    float omega = 0.0;
    float theta = PI/4;

    float s = sin(theta);
    float c = cos(theta);

    float Phi1 = (-s * dx) + ( c * dy) + omega * d;
    float Phi2 = (-c * dx) + (-s * dy) + omega * d;
    float Phi3 = ( s * dx) + (-c * dy) + omega * d;
    float Phi4 = ( c * dx) + ( s * dy) + omega * d;

    Phi1 /= R;
    Phi2 /= R;
    Phi3 /= R;
    Phi4 /= R;

    const float RADS_TO_RPM = 9.5492966f;
    Phi1 *= RADS_TO_RPM;
    Phi2 *= RADS_TO_RPM;
    Phi3 *= RADS_TO_RPM;
    Phi4 *= RADS_TO_RPM;

    DJI_Motor_Set_Velocity(motor_w1, Phi4);
    DJI_Motor_Set_Velocity(motor_w2, Phi1);
    DJI_Motor_Set_Velocity(motor_w3, Phi2);
    DJI_Motor_Set_Velocity(motor_w4, Phi3);
}
