#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include "soft_i2c.h"
#include "math.h"

/*================== 宏定义 ==================*/

#define MPU_IIC_ADDR  0x68   // // MPU6050地址 (AD0接地)
#define PI 3.1415926

// 寄存器定义

#define MPU_SELF_TESTX_REG      0X0D    // 自测寄存器X
#define MPU_SELF_TESTY_REG      0X0E    // 自测寄存器Y
#define MPU_SELF_TESTZ_REG      0X0F    // 自测寄存器Z
#define MPU_SELF_TESTA_REG      0X10    // 自测寄存器A
#define MPU_SAMPLE_RATE_REG     0X19    // 采样频率分频器
#define MPU_CFG_REG             0X1A    // 配置寄存器
#define MPU_GYRO_CFG_REG        0X1B    // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG       0X1C    // 加速度计配置寄存器
#define MPU_MOTION_DET_REG      0X1F    // 运动检测阈值设置寄存器
#define MPU_FIFO_EN_REG         0X23    // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG     0X24    // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG    0X25    // IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG         0X26    // IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG    0X27    // IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG    0X28    // IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG         0X29    // IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG    0X2A    // IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG    0X2B    // IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG         0X2C    // IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG    0X2D    // IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG    0X2E    // IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG         0X2F    // IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG    0X30    // IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG    0X31    // IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG         0X32    // IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG      0X33    // IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG    0X34    // IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG      0X35    // IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG      0X36    // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG       0X37    // 中断/旁路设置寄存器
#define MPU_INT_EN_REG          0X38    // 中断使能寄存器
#define MPU_INT_STA_REG         0X3A    // 中断状态寄存器

#define MPU_ACCEL_XOUTH_REG     0X3B    // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG     0X3C    // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG     0X3D    // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG     0X3E    // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG     0X3F    // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG     0X40    // 加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG       0X41    // 温度值高8位寄存器
#define MPU_TEMP_OUTL_REG       0X42    // 温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG      0X43    // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG      0X44    // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG      0X45    // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG      0X46    // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG      0X47    // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG      0X48    // 陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG      0X63    // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG      0X64    // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG      0X65    // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG      0X66    // IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG    0X67    // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG     0X68    // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG    0X69    // 运动检测控制寄存器
#define MPU_USER_CTRL_REG       0X6A    // 用户控制寄存器
#define MPU_PWR_MGMT1_REG       0X6B    // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG       0X6C    // 电源管理寄存器2 
#define MPU_FIFO_CNTH_REG       0X72    // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG       0X73    // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG         0X74    // FIFO读写寄存器
#define MPU_DEVICE_ID_REG       0X75    // 器件ID寄存器

/*================== 全局变量声明 ==================*/

extern volatile float accel_angle;
extern volatile float gyro_angle;
extern volatile float gyro_x_offset;
extern volatile float fused_angle;
extern float gyro_x_speed;
extern float gyro_z_speed;

/*================== 卡尔曼滤波 ==================*/
typedef struct {
    float Q_angle;   // 过程噪声的协方差（加速度计信任度，越小越信任）
    float Q_gyro;    // 过程噪声的协方差（陀螺仪漂移信任度）
    float R_angle;   // 测量噪声的协方差（测量噪声，越大表示越不信任加速度计）
    float angle;     // 滤波后的角度
    float bias;      // 陀螺仪漂移（零偏）
    float P[2][2];   // 误差协方差矩阵
} Kalman_t;

extern Kalman_t KalmanX;

/*================== 函数声明 ==================*/

uint8_t MPU6050_WriteByte(uint8_t reg_addr, uint8_t write_byte);
uint8_t MPU6050_writeBytes(uint8_t reg_addr, uint8_t write_bytes[], uint8_t size);
uint8_t MPU6050_ReadByte(uint8_t reg_addr, uint8_t *byte);
uint8_t MPU6050_ReadBytes(uint8_t reg_addr, uint8_t *bytes, uint8_t size);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_Get_Gyro(short *gx, short *gy, short *gz);
uint8_t MPU6050_Get_Accel(short *ax, short *ay, short *az);
void MPU6050_Get_AccelAngle(void);
void MPU6050_Calibrate_Gyro(void);
void MPU6050_get_FusedAngle_Optimized(float dt);

#endif
