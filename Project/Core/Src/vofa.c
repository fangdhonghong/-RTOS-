#include "vofa.h"
#include <stdlib.h> // 用于 atof 函数
#include <string.h> // 用于 strncmp 函数
#include "FreeRTOS.h"    // 必须先包含
#include "semphr.h"      // 然后再包含 semphr.h

VofaData g_vofa_frame; // 公告板

extern SemaphoreHandle_t g_uart_parse_semaphore;  // UART解析信号量（定义在freertos.c）

uint8_t Rx_buffer[RX_BUFFER_SIZE];      // 接收缓冲区
uint8_t Process_buffer[RX_BUFFER_SIZE + 1]; // 解析缓冲区
volatile uint8_t IsParse = 0;           // 解析标识位

// 初始化函数
void VOFA_Init(void)
{
    /*=================== 上行发送初始化 ===================*/
    // 数据段清零
    g_vofa_frame.expect_angle = 0;
    g_vofa_frame.Reality_angle = 0;
    g_vofa_frame.PWM_output_value = 0;

    // 帧尾段赋予 0x7F800000
    g_vofa_frame.tail = 0x7F800000;

    /*=================== 下行接收初始化 ===================*/
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_buffer, RX_BUFFER_SIZE); // 因为作为STM32方，我并不知道数据长度
}

/**
 * @brief  数据上发
 */
void VOFA_SendDta(void)
{
    // 0.保证后缀
    g_vofa_frame.tail = 0x7F800000;

    // 1. 安全检查
    if (huart3.gState != HAL_UART_STATE_READY)
        return;

    // 2.数据搬运
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&g_vofa_frame, sizeof(g_vofa_frame));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        // 记忆上一次的位置
        static uint16_t old_pos = 0;
        // 确定当前位置
        uint16_t current_pos = Size;
        // 获取DMA搬运的数据长度
        uint16_t len = 0;
        // 获取数据
        // 数据没溢出
        if (old_pos < current_pos)
        {
            len = current_pos - old_pos;                      // 获取DMA搬运长度
            memcpy(Process_buffer, Rx_buffer + old_pos, len); // 复制到解析区
        }
        else
        {
            uint16_t tail_len = RX_BUFFER_SIZE - old_pos;
            memcpy(Process_buffer, &Rx_buffer[old_pos], tail_len);
            uint16_t head_len = current_pos;
            memcpy(&Process_buffer[tail_len], &Rx_buffer[0], head_len);
            len = tail_len + head_len;
        }
        // 解析
        if (len > 0)
        {
            Process_buffer[len] = '\0';
            IsParse = 1;
            /* 给出信号量通知 Com_Task 有新数据 */
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(g_uart_parse_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        // 更新上一次的位置
        old_pos = current_pos;
    }
}

/*
U：直立环
S：速度环
T：转向环
比如：UKp15\n，说明直立环的p项改成15，\n表示数据完毕
*/

// 数据解析
void VOFA_Parse_Command(char *data)
{
    // 遍历字符串，只要遇到换行符 (\r 或 \n)，就强制截断
    for (int i = 0; data[i] != '\0'; i++)
    {
        if (data[i] == '\r' || data[i] == '\n')
        {
            data[i] = '\0'; // 替换为结束符
            break;          // 只要处理了第一个换行，后面就不用管了
        }
    }
    
    float val = 0.0f;

    // 1. 判断是哪个环 (根据首字母)
    switch (data[0])
    {
    /* ================= 直立环 (Upright) ================= */
    case 'U':
        // 跳过 'U'，检查接下来的两个字符
        if (strncmp(data + 1, "Kp", 2) == 0) // 指令: UKp...
        {
            val = atof(data + 3); // 跳过 "UKp" (3个字符)，转换后面的数字
            UprightRing.Kp = val;
        }
        else if (strncmp(data + 1, "Kd", 2) == 0) // 指令: UKd...
        {
            val = atof(data + 3);
            UprightRing.Kd = val;
        }
        break;

    /* ================= 速度环 (Speed) ================= */
    case 'S':
        if (strncmp(data + 1, "Kp", 2) == 0) // 指令: SKp...
        {
            val = atof(data + 3);
            SpeedRing.Kp = val;
        }
        else if (strncmp(data + 1, "Ki", 2) == 0) // 指令: SKi...
        {
            val = atof(data + 3);
            SpeedRing.Ki = val;
        }
        break;

    /* ================= 转向环 (Turn) ================= */
    case 'T':
        if (strncmp(data + 1, "Kp", 2) == 0) // 指令: TKp...
        {
            val = atof(data + 3);
            TurnRing.Kp = val;
        }
        break;
    
    /* ================= 目标速度 (target_speed) ================= */
    case 'W':
        if (strncmp(data + 1, "speed", 5) == 0)
        {
            val = atof(data + 6);
            target_speed = (int)val;
        }
        break;
    /* ================= 转向指令 (turn_cmd) ================= */
    case 'V':
        if (strncmp(data + 1, "turn", 4) == 0)
        {
            val = atof(data + 5);
            turn_cmd = (int)val;
        }
        break;
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 判断是否是我们用的串口3
    if (huart->Instance == USART3)
    {
        // 这里的逻辑非常重要：
        // 1. 发生错误时，HAL库通常会关掉DMA
        // 2. 我们需要清除错误标志（HAL库底层已处理），并重新启动DMA接收
        
        // 重新开启接收（这行代码是救命稻草）
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_buffer, RX_BUFFER_SIZE);
    }
}
