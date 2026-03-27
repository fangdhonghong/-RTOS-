#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "pid.h"    // 必须包含，用于访问 PID 结构体

#define RX_BUFFER_SIZE  128  // 接收缓冲区的长度

typedef struct __packed  //  __packed 告诉编译器：别乱动，字节给我挤紧点
{ 
    // --- 数据段 ---
    float Reality_angle;    // 现实角度
    float expect_angle;     // 期望角度
    float PWM_output_value; // PWM输出值

    // --- 帧尾段 ---
    uint32_t tail;          // 必须放在最后，用于存储 0x7F800000
}VofaData;

extern VofaData g_vofa_frame;  // 公告板

extern volatile uint8_t IsParse ;           // 解析标识位

extern uint8_t Process_buffer[RX_BUFFER_SIZE + 1]; // 解析缓冲区


void VOFA_Init(void);
void VOFA_SendDta(void);
void VOFA_Parse_Command(char *data);

#endif
