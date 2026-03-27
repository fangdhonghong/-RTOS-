#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

/************************ 结构体声明 ************************/
/*=================== 直立环 ===================*/
typedef struct 
{
    float Kp;
    float Kd;
}Upright;

/*=================== 速度环 ===================*/
typedef struct 
{
    float Kp;
    float Ki;
}Speed;

/*=================== 转向环 ===================*/
typedef struct
{
    float Kp;
}Turn;

/************************ 变量声明 ************************/
extern Upright UprightRing;  // 直立环
extern Speed SpeedRing;      // 速度环
extern Turn TurnRing;        // 转向环

extern float mechanical_balance_angle;  // 机械中值

extern volatile int target_speed;  // 目标速度

extern volatile uint8_t IsFall;           // 倒地标识位

extern float speed_filter_old;  // 保存上一次的滤波结果

extern volatile int turn_cmd;   // 转向指令

/************************ 函数声明 ************************/
void PID_Init(void);

int Upright_Control(float current_angle, float gyro_speed, float target_angle);

float Speed_Control(int enc_left, int enc_right, float current_angle);

int Turn_Control(float target_turn_speed, float current_gyro_z);

#endif