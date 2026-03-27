/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "MPU6050.h"
#include "string.h"
#include "stdio.h"
#include "dwt_delay.h"
#include "Ultrasonic.h"
#include <math.h>
#include <stdlib.h>
#include "motor.h"
#include <encoder.h>
#include "vofa.h"
#include "pid.h"
#include "BluetoothControl.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern VofaData g_vofa_frame;  // VOFA+公告板
extern float mechanical_balance_angle;  // 机械中值
extern volatile uint8_t IsParse ;           // 解析标识位
extern uint8_t Process_buffer[RX_BUFFER_SIZE + 1]; // 解析缓冲区
extern volatile uint8_t IsFall;           // 倒地标识位
extern float speed_filter_old;  // 经过滤波后的速度
extern volatile int target_speed;  // 目标速度
extern volatile int turn_cmd;  // 转向指令
extern TickType_t last_heartbeat_tick; // 上次心跳时间戳（FreeRTOS tick）

extern volatile float accel_angle;  // 加速度计获取的角度
extern volatile float gyro_angle;   // 陀螺仪获取的角度
extern volatile float fused_angle;  // 融合后的角度
extern float gyro_x_speed;          // x轴的角速度
extern float gyro_z_speed;          // z轴的角速度

extern volatile uint8_t isFinish;  // 测量完成标志位
extern volatile uint16_t count;  // 定时器的值（高电平持续时间）

extern Kalman_t KalmanX;

extern int Encoder_Left;      // 左轮速度 (定义在main.c)
extern int Encoder_Right;     // 右轮速度 (定义在main.c)

extern uint8_t System_Ready; // 系统就绪标志位 (定义在main.c)

SemaphoreHandle_t g_uart_parse_semaphore;  // UART解析信号量

float g_distance = 0;  // 超声波测距距离，全局共享
float target_angle_from_speed = 0; // 速度环输出的目标角度，全局共享供Com_Task上发

/* Control Task Definition */
osThreadId_t Control_TaskHandle;
const osThreadAttr_t Control_Task_attributes = {
  .name = "Control_Task",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Ultrasonic Task Definition */
osThreadId_t Ultrasonic_TaskHandle;
const osThreadAttr_t Ultrasonic_Task_attributes = {
  .name = "Ultrasonic_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Com Task Definition */
osThreadId_t Com_TaskHandle;
const osThreadAttr_t Com_Task_attributes = {
  .name = "Com_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* OLED Task Definition */
osThreadId_t OLED_TaskHandle;
const osThreadAttr_t OLED_Task_attributes = {
  .name = "OLED_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Start_Control_Task(void *argument);
void Start_Ultrasonic_Task(void *argument);
void Start_Com_Task(void *argument);
void Start_OLED_Task(void *argument);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* UART解析信号量 */
  g_uart_parse_semaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* start threads, ... */

  /* Control Task - 最高优先级 */
  Control_TaskHandle = osThreadNew(Start_Control_Task, NULL, &Control_Task_attributes);

  /* Ultrasonic Task */
  Ultrasonic_TaskHandle = osThreadNew(Start_Ultrasonic_Task, NULL, &Ultrasonic_Task_attributes);

  /* Com Task */
  Com_TaskHandle = osThreadNew(Start_Com_Task, NULL, &Com_Task_attributes);

  /* OLED Task */
  OLED_TaskHandle = osThreadNew(Start_OLED_Task, NULL, &OLED_Task_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Control_Task */
/**
  * @brief  Function implementing the Control_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Control_Task */
void Start_Control_Task(void *argument)
{
  /* USER CODE BEGIN Start_Control_Task */

  /* 等待系统就绪（传感器校准完成） */
  while (!System_Ready)
  {
    osDelay(pdMS_TO_TICKS(10));
  }

  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint8_t speed_loop_counter = 0;
  // target_angle_from_speed 已改为全局变量，在文件顶部定义

  /* Infinite loop */
  for(;;)
  {
    /* ===== 40ms 任务：速度环 ===== */
    if (speed_loop_counter >= 8)
    {
      /* 编码器测速 */
      Encoder_Left = Read_Speed(&htim2);
      Encoder_Right = -1 * Read_Speed(&htim4);

      /* 速度环 PI */
      target_angle_from_speed = Speed_Control(Encoder_Left, Encoder_Right, fused_angle);

      speed_loop_counter = 0;
    }

    /* ===== 5ms 任务：直立环 + 转向环 ===== */
    /* 姿态解算 */
    MPU6050_get_FusedAngle_Optimized(0.005f);

    /* 倒地检测 */
    if (fabs(fused_angle) > 40.0f)
      IsFall = 1;
    else if (fabs(fused_angle) < 5.0f)
      IsFall = 0;

    /* 直立环 PD */
    float pure_gyro_x_speed = gyro_x_speed - KalmanX.bias;
    int Upright_pwm = Upright_Control(fused_angle, pure_gyro_x_speed, target_angle_from_speed);

    /* 转向环 P */
    int Turn_pwm = Turn_Control(turn_cmd, gyro_z_speed);

    /* 电机输出 */
    if (IsFall == 1)
    {
      Set_Motor_Speed(0, 0);
      target_angle_from_speed = 0;
    }
    else
    {
      Set_Motor_Speed(Upright_pwm + Turn_pwm, Upright_pwm - Turn_pwm);
    }

    speed_loop_counter++;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
  /* USER CODE END Start_Control_Task */
}

/* USER CODE BEGIN Header_Start_Ultrasonic_Task */
/**
  * @brief  Function implementing the Ultrasonic_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Ultrasonic_Task */
void Start_Ultrasonic_Task(void *argument)
{
  /* USER CODE BEGIN Start_Ultrasonic_Task */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t isObstacle = 0;
  (void)isObstacle; // 预留避障逻辑

  /* Infinite loop */
  for(;;)
  {
    if (isFinish == 0)
    {
      Get_Distance();
    }

    if (isFinish == 1)
    {
      g_distance = count * 0.017;
      if (g_distance > 0 && g_distance <= 30)
        isObstacle = 1;
      else
        isObstacle = 0;
      isFinish = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60));
  }
  /* USER CODE END Start_Ultrasonic_Task */
}

/* USER CODE BEGIN Header_Start_Com_Task */
/**
  * @brief  Function implementing the Com_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Com_Task */
void Start_Com_Task(void *argument)
{
  /* USER CODE BEGIN Start_Com_Task */
  TickType_t xLastWakeTime = xTaskGetTickCount();

  /* 本地副本，用于VOFA发送 */
  float reality_angle = 0;
  float expect_angle = 0;
  float pwm_output = 0;
  uint8_t local_IsParse = 0;
  uint8_t local_Process_buffer[RX_BUFFER_SIZE + 1];

  /* Infinite loop */
  for(;;)
  {
    /* 等待UART解析信号量（由DMA中断给出） */
    if (xSemaphoreTake(g_uart_parse_semaphore, pdMS_TO_TICKS(50)) == pdTRUE)
    {
      /* 原子复制数据，避免竞态条件 */
      local_IsParse = IsParse;
      memcpy(local_Process_buffer, Process_buffer, RX_BUFFER_SIZE);
    }

    /* 蓝牙解析（使用本地副本） */
    if (local_IsParse == 1)
    {
      local_IsParse = 0;
      /* 根据数据内容判断来源并解析 */
      Bluetooth_Parse_Binary(local_Process_buffer);
    }

    /* 软件看门狗 - 检查控制指令超时 */
    if (xTaskGetTickCount() - last_heartbeat_tick > pdMS_TO_TICKS(CONTROL_TIMEOUT))
    {
      /* 超时，认为通信丢失 */
      target_speed = 0;  // 目标速度清零
      turn_cmd = 0;       // 转向指令清零
    }

    /* VOFA 数据上发 */
    reality_angle = fused_angle;
    expect_angle = mechanical_balance_angle + target_angle_from_speed;
    /* 从全局变量获取PWM值 */
    pwm_output = (abs(g_moto1_pwm) + abs(g_moto2_pwm)) / 2.0f;

    g_vofa_frame.Reality_angle = reality_angle;
    g_vofa_frame.expect_angle = expect_angle;
    g_vofa_frame.PWM_output_value = pwm_output;
    VOFA_SendDta();

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
  }
  /* USER CODE END Start_Com_Task */
}

/* USER CODE BEGIN Header_Start_OLED_Task */
/**
  * @brief  Function implementing the OLED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_OLED_Task */
void Start_OLED_Task(void *argument)
{
  /* USER CODE BEGIN Start_OLED_Task */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t oled_stage = 0;

  /* Infinite loop */
  for(;;)
  {
    switch (oled_stage)
    {
      case 0:
        /* 角度数据可视化 */
        OLED_ShowSignedFloat(48, 0, fused_angle, 3, 2, 16);
        break;
      case 1:
        /* 速度数据可视化 */
        OLED_ShowSignedNum(12, 6, Encoder_Left, 4, 12);
        OLED_ShowSignedNum(76, 6, Encoder_Right, 4, 12);
        break;
      case 2:
        /* 障碍数据可视化 */
        OLED_ShowSignedFloat(24, 4, g_distance, 3, 1, 12);
        break;
    }
    oled_stage = (oled_stage + 1) % 3;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(125));
  }
  /* USER CODE END Start_OLED_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */