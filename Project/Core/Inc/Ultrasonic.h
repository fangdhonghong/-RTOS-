#ifndef __UlTRSONIC_H
#define __UlTRSONIC_H

#include "stm32f1xx_hal.h"
#include "dwt_delay.h"   // DWT延时
#include "tim.h"
#include "gpio.h"

extern volatile uint8_t isFinish;  // 测量完成标志位
extern volatile uint16_t count;  // 定时器的值（高电平持续时间）

// 发起一次测距
void Get_Distance(void);

#endif
