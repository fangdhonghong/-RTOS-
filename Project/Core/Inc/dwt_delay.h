#ifndef __DWT_DELAY_H
#define __DWT_DELAY_H

#include "stm32f1xx_hal.h"

/**
 * @brief  初始化 DWT 延时计数器
 * @note   必须在 HAL_RCC_ClockConfig 配置完时钟后调用
 */
void DWT_Delay_Init(void);

/**
 * @brief  微秒级延时函数
 * @param  us: 延时的微秒数
 */
void Delay_us(uint32_t us);

/**
 * @brief  毫秒级延时函数 (基于 DWT，比 HAL_Delay 更精准，但会阻塞)
 * @param  ms: 延时的毫秒数
 */
void Delay_ms(uint32_t ms);

#endif
