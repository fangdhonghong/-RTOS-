#include "BluetoothControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <math.h>

Joystick_TypeDef Rocker; // 声明一个结构体变量

TickType_t last_heartbeat_tick = 0; // 上次心跳时间戳（FreeRTOS tick）

/**
 * @brief  解析蓝牙二进制数据包 (7字节模式)
 * @param  data: 串口接收缓冲区 Process_buffer
 */
void Bluetooth_Parse_Binary(uint8_t *data)
{
    // 1. 验证包头包尾
    if (data[0] == 0xA5 && data[6] == 0x5A)
    {
        // 2. 验证校验和 (原数据之和)
        uint8_t sum = data[1] + data[2] + data[3] + data[4];
        if (sum == data[5])
        {
            // 3. 提取 short 数据 
            Rocker.speed = (int16_t)((data[2] << 8) | data[1]); 
            Rocker.turn  = (int16_t)((data[4] << 8) | data[3]);  
            // 4. 处理速度
            static float speed_lpf = 0.0f; // 速度低通滤波变量
            float speed_temp = 0;

            if (abs(Rocker.speed) < 5)
            {
                speed_temp = 0;
            } else {
                speed_temp = (float)Rocker.speed;
            }
            // 数据会猛扑，所以要一点一点增加步长：斜坡函数
            // 想到做一个低通滤波：大概率相信旧数据
            speed_lpf = speed_lpf + 0.15f * (speed_temp - speed_lpf);
            target_speed = -1 * speed_lpf;

            // 5. 处理转向 (turn_cmd)
            if (abs(Rocker.turn) < 5) {
                turn_cmd = 0;
            } else {
                turn_cmd = -1 * Rocker.turn;
            }

            // 6. 喂狗
            last_heartbeat_tick = xTaskGetTickCount();
        }
    }
}

