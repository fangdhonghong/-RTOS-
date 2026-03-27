#include "encoder.h"

int Read_Speed(TIM_HandleTypeDef *htim)
{
    int temp = (short)__HAL_TIM_GetCounter(htim);  // 获取定时器数据
    __HAL_TIM_SetCounter(htim, 0);   // 将定时器数据清空
    return temp;
}