/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "MPU6050.h"
#include "string.h"
#include "stdio.h"
#include "dwt_delay.h"
#include "Ultrasonic.h"
#include <math.h>
#include "motor.h"
#include <encoder.h>
#include "vofa.h"
#include "pid.h"
#include "BluetoothControl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 为了让中断也可以用，声明放在main函数外

extern volatile float accel_angle;  // 加速度计获取的角度
extern volatile float gyro_angle;   // 陀螺仪获取的角度
extern volatile float fused_angle;  // 融合后的角度
extern float gyro_x_speed;          // x轴的角速度
extern float gyro_z_speed;          // z轴的角速度

extern volatile uint8_t isFinish;  // 测量完成标志位
extern volatile uint16_t count;  // 定时器的值（高电平持续时间）

int Encoder_Left = 0;      // 左轮速度
int Encoder_Right = 0;     // 右轮速度

uint8_t System_Ready = 0; // 系统就绪标志位

extern VofaData g_vofa_frame;  // VOFA+公告板
extern float mechanical_balance_angle;  // 机械中值
extern volatile uint8_t IsParse ;           // 解析标识位
extern uint8_t Process_buffer[RX_BUFFER_SIZE + 1]; // 解析缓冲区
extern volatile uint8_t IsFall;           // 倒地标识位
extern float speed_filter_old;  // 经过滤波后的速度
extern volatile int target_speed;  // 目标速度
extern volatile int turn_cmd;  // 转向指令

extern TickType_t last_heartbeat_tick; // 上次心跳时间戳（FreeRTOS tick）

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /*====================== MPU6050初始化 ======================*/
  MPU6050_Init();
  HAL_Delay(100);   // 等待传感器稳定
  // 上电静止校准
  MPU6050_Calibrate_Gyro();
  // 让陀螺仪初始角度等于加速度计角度
  MPU6050_Get_AccelAngle(); 
  gyro_angle = accel_angle; 
  fused_angle = accel_angle;
  // 允许滴答定时器中断开始工作
  System_Ready = 1;    // 不然在校准时，中断已经偷偷运行很久了

  /*====================== OLED初始化 ======================*/
  OLED_Init();  // 初始化oled屏幕
  OLED_Clear();  // 清屏
  // --- 打印固定不动的标签 (静态背景) ---
  // Line 0: "Angle:" (16号字体, 6个字符占 6*8=48 像素)
  OLED_ShowString(0, 0, (uint8_t*)"Angle:", 16); 

  // Line 3: 分隔线
  OLED_ShowString(0, 3, (uint8_t*)"--------------", 8); 

  // Line 4: "Dis:" 和 "cm" 
  OLED_ShowString(0, 4, (uint8_t*)"Dis:", 12);   
  OLED_ShowString(78, 4, (uint8_t*)"cm", 12);    // 24 + (9*6) = 78 (预留显示位)

  // Line 5: "L:" 和 "R:"
  OLED_ShowString(0, 6, (uint8_t*)"L:", 12);
  OLED_ShowString(64, 6, (uint8_t*)"R:", 12);    // 在屏幕中间 64 像素处开始打印 R:

  /*====================== 电机驱动模块初始化 ======================*/  
  // 启动定时器1的通道1和4的PWM脉冲
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // 关闭电机
  Set_Motor_Speed(0, 0);

  /*====================== 编码器测速模块初始化 ======================*/ 
  // 启动定时器2和4的所有通道
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /*====================== VOFA+模块初始化 ======================*/ 
  VOFA_Init();

  /*====================== PID模块初始化 ======================*/ 
  PID_Init();
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* 滴答定时器的中断回调 - FreeRTOS模式下由OS接管，此处不再使用 */
/* 控制逻辑已移至 Control_Task */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
