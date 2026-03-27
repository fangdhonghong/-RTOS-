#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"
void Set_Motor_Speed(int moto1,int moto2);

extern int g_moto1_pwm;
extern int g_moto2_pwm;

#endif
