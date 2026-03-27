#include "Ultrasonic.h"

/*
void 开始测距()
{
    拉高PA3
    延时12us(大于10即可)
    拉高PA3
}

// 测量PA2高电平时间：用定时器3 + 外部中断 来实现
PA2外部中断复位程序
{
    if (高电平进入的中断)
        把计算器清零
        开启定时器3
    else if(低电平进入的中断)
        关闭定时器
        获取定时器的值
        距离为定时器的值 * 0.034 \ 2  // 不要在中断里算距离 => 测量完成标志置1
}

main函数
{
    if(完成标志 == 1) 
        距离为定时器的值 * 0.034 \ 2
        测量完成标志置0
}

别忘了异步处理
*/

volatile uint8_t isFinish = 0;  // 测量完成标志位
volatile uint16_t count;  // 定时器的值（高电平持续时间）

void Get_Distance(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    Delay_us(12);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
        {
            __HAL_TIM_SetCounter(&htim3, 0);
            HAL_TIM_Base_Start(&htim3);
        }
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
        {
            HAL_TIM_Base_Stop(&htim3);
            count = __HAL_TIM_GetCounter(&htim3);
            isFinish = 1; 
        }
    }
}