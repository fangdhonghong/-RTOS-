#include "motor.h"
#include <math.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;
#define PWM_MAX_LIMIT 7200

// 死区宏定义  
#define L_DEAD_BACK  64  // 左轮反向死区 320
#define L_DEAD_FOR   76  // 左轮正向死区 380
#define R_DEAD_BACK  83  // 右轮反向死区 415
#define R_DEAD_FOR   62  // 右轮正向死区 310
/**
 * @description: 线性死区补偿映射函数
 * @param input:  原始速度 (-7200 到 7200)
 * @param dead_for: 正向死区临界点
 * @param dead_back: 反向死区临界点
 * @return: 映射后的实际PWM值
 */
int32_t Linear_Mapping(int32_t input, int32_t dead_for, int32_t dead_back)
{
    if (input > 0) {
        // 公式: (input * (最大可用区间)) / 最大输入区间 + 偏移量
        // 即: (input * (7200 - 300)) / 7200 + 300
        return (input * (PWM_MAX_LIMIT - dead_for)) / PWM_MAX_LIMIT + dead_for;
    } 
    else if (input < 0) {
        return (abs(input) * (PWM_MAX_LIMIT - dead_back)) / PWM_MAX_LIMIT + dead_back;
    }
    return 0; // 真正为0时输出0
}

/**
 * @description: 电机驱动
 * @param moto1 左轮速度  数值越大速度越快
 * @param moto2 右轮速度  数值越大速度越快
 */
int g_moto1_pwm = 0;
int g_moto2_pwm = 0;

void Set_Motor_Speed(int moto1, int moto2)
{
	// 保存PWM值供VOFA显示
	g_moto1_pwm = moto1;
	g_moto2_pwm = moto2;

	// 限幅处理
    if (moto1 > PWM_MAX_LIMIT)  moto1 = PWM_MAX_LIMIT;
    if (moto1 < -PWM_MAX_LIMIT) moto1 = -PWM_MAX_LIMIT;
    if (moto2 > PWM_MAX_LIMIT)  moto2 = PWM_MAX_LIMIT;
    if (moto2 < -PWM_MAX_LIMIT) moto2 = -PWM_MAX_LIMIT;

	// 进行线性死区补偿映射
	uint32_t input1 = Linear_Mapping(moto1, L_DEAD_FOR, L_DEAD_BACK);
	uint32_t input2 = Linear_Mapping(moto2, R_DEAD_FOR, R_DEAD_BACK);

	if(moto1<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,input1);
	if(moto2<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,input2);
}