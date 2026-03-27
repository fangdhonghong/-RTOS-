#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdio.h>
#include "stm32f1xx_hal.h"

#define IIC_WR  0  // 写命令
#define IIC_RD  1  // 读命令

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_GPIO_Init(void);


#endif
