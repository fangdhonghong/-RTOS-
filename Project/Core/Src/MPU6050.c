#include "MPU6050.h"

/**
 * @description: 向MPU6050写入一个字节
 * @param {uint8_t} reg_addr 寄存器地址
 * @param {uint8_t} write_byte 要写入的数据
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_WriteByte(uint8_t reg_addr, uint8_t write_byte)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(write_byte);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Stop();
    return 0;
}



/**
 * @description: 向MPU6050写入多个字节
 * @param {uint8_t} reg_addr 寄存器起始地址
 * @param {uint8_t} *write_bytes 数据缓冲区
 * @param {uint8_t} size 数据长度
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_writeBytes(uint8_t reg_addr, uint8_t write_bytes[], uint8_t size)
{
    IIC_Start();

    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    for (int i = 0; i < size; i++)
    {
        IIC_Send_Byte(write_bytes[i]);
        if (IIC_Wait_Ack())
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * @description: 读取指定寄存器一个字节数据
 * @param {uint8_t} reg_addr 寄存器地址
 * * @param {uint8_t} *byte 接收的数据
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_ReadByte(uint8_t reg_addr, uint8_t *byte)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_RD);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    *byte = IIC_Read_Byte(0);
    IIC_Stop();

    return 0;
}



/**
 * @description: 读取指定寄存器多个字节数据
 * @param {uint8_t} reg_addr 寄存器起始地址
 * @param {uint8_t} *bytes 接收缓冲区
 * @param {uint8_t} size 读取长度
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_ReadBytes(uint8_t reg_addr, uint8_t *bytes, uint8_t size)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_RD);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    for (int i = 0; i < size; i++)
    {
        if (i == size - 1)  bytes[i] = IIC_Read_Byte(0);
        else bytes[i] = IIC_Read_Byte(1);
    }

    IIC_Stop();
    return 0;
}

/**
 * @description: 根据采样率设置低通滤波器
 * @param {uint16_t} rate
 * @return {*}
 */
void MPU6050_Set_DLPF_CFG(uint16_t rate)
{
    /* 采样定理 采样率 >= 2*带宽 才不失真 ===> 带宽 <= 采样率/2 */
    uint8_t cfg = 0;
    rate = rate / 2;
    if (rate > 188) cfg = 1;
    else if (rate > 98) cfg = 2;
    else if (rate > 42) cfg = 3;
    else if (rate > 20) cfg = 4;
    else if (rate > 10) cfg = 5;
    else cfg = 6;
    
    MPU6050_WriteByte(MPU_CFG_REG, cfg << 0);
}

/**
 * @description: 设置陀螺仪的采样率
 * @param {uint16_t} rate
 * @return {*}
 */
void MPU6050_SetGyroRate(uint16_t rate)
{
    /* 采样率=陀螺仪输出频率/(1+分频值) ===> 分频值 = (陀螺仪输出频率/采样率) -1 */
    uint8_t sample_div = 0;
    /* 1. 采样率范围: 芯片说明书12页，采样频率最小值=4，陀螺仪8k或1k */
    if (rate < 4) rate = 4;
    else if (rate > 1000) rate = 1000;

    /* 2. 根据需要的采样率，计算分频值 */
    sample_div = 1000 / rate - 1;

    /* 3. 将分频值设置到寄存器中 */
    MPU6050_WriteByte(MPU_SAMPLE_RATE_REG, sample_div);

    /* 4. 根据采样率去设置低通滤波器 */
    MPU6050_Set_DLPF_CFG(rate);
}


/**
 * @description: 初始化MPU6050
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Init(void)
{
    uint8_t dev_id = 0;

    // 0.初始化GPIO
    IIC_GPIO_Init();

    // 1.复位 - 延时 - 唤醒
    MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x80);  // 往 0X6B 写 0x80
    HAL_Delay(100); 
    MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x00);  // 往 0X6B 写 0x00

    // 2.陀螺仪量程
    MPU6050_WriteByte(MPU_GYRO_CFG_REG, 0x18);   // 往 0X1B 写 0x18

    // 3.加速度量程
    MPU6050_WriteByte(MPU_ACCEL_CFG_REG, 0x00);  // 往 0X1C 写 0x00

    // 4.中断/FIFO/I2C主机配置
    MPU6050_WriteByte(MPU_INT_EN_REG, 0x00);   // 关闭所有中断：往 0x38 写 0x00
    MPU6050_WriteByte(MPU_USER_CTRL_REG, 0x00); // 关闭第二IIC：往 0X6A 写 0x00
    MPU6050_WriteByte(MPU_FIFO_EN_REG, 0x00);   // 关闭FIFO：往 0x23 写 0x00
    MPU6050_WriteByte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效

    // 5.读取ID
    MPU6050_ReadByte(MPU_DEVICE_ID_REG, &dev_id);

    // 检查ID
    if (dev_id == MPU_IIC_ADDR)
    {
        /* 5.1 设置时钟源，PLL X轴 */
        MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x01);
        /* 5.2 设置采样率 100Hz */
        MPU6050_SetGyroRate(100);
        /* 5.3 开启工作 (虽然前面复位唤醒了，这里确保不进入睡眠) */
        MPU6050_WriteByte(MPU_PWR_MGMT2_REG, 0x00);
    }
    else
    {
        return 1; // ID不对，返回错误
    }

    return 0; // 初始化成功
}


/* ========================== 上层应用 ========================== */

/**
 * @description: 读取陀螺仪角速度的数据
 * @param {short} *gx, *gy, *gz 指针
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Get_Gyro(short *gx, short *gy, short *gz)
{
    uint8_t buff[6];
    uint8_t res;
    
    res = MPU6050_ReadBytes(MPU_GYRO_XOUTH_REG, buff, 6);
    
    if (res == 0)
    {
        *gx = ((uint16_t)buff[0] << 8) | buff[1];
        *gy = ((uint16_t)buff[2] << 8) | buff[3];
        *gz = ((uint16_t)buff[4] << 8) | buff[5];
        return 0; // 成功
    }
    
    return 1; // 失败
}

/**
 * @description: 读取加速度计的加速度数据
 * @param {short} *ax, *ay, *az 指针
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Get_Accel(short *ax, short *ay, short *az)
{
    uint8_t buff[6];
    uint8_t res;
    
    res = MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buff, 6);
    
    if (res == 0)
    {
        *ax = ((uint16_t)buff[0] << 8) | buff[1];
        *ay = ((uint16_t)buff[2] << 8) | buff[3];
        *az = ((uint16_t)buff[4] << 8) | buff[5];
        return 0; // 成功
    }
    
    return 1; // 失败
}

/*================================== 根据加速度求的角度 ==================================*/

volatile float accel_angle = 0; 

// 根据加速度求的角度
void MPU6050_Get_AccelAngle(void)
{
    short ax, ay, az;

    // 获取MPU6050的加速度
    MPU6050_Get_Accel(&ax, &ay, &az);

    // 通过受力分析得：角度的tan值为ay / az (为什么不需要除灵敏度比例因子，因为会给约掉)
    // atan2反正切函数：得到的是弧度,角度 = 弧度 * 180 / PI
    accel_angle = atan2(ay, az) * 180 / PI - 1.5f;
}


/*================================== 上电自校准 ==================================*/

volatile float gyro_x_offset = 0; // X轴零偏（直立环用）
volatile float gyro_z_offset = 0; // Z轴零偏（转向环用）

// 角速度 X Y轴校准函数
void MPU6050_Calibrate_Gyro(void)
{
    short gx, gy, gz;
    long sum_gx = 0;
    long sum_gz = 0;
    uint16_t sample_count = 200; // 采集200次数据
    
    for (uint16_t i = 0; i < sample_count; i++)
    {
        // 调用你原来的读取函数
        if (MPU6050_Get_Gyro(&gx, &gy, &gz) == 0)
        {
            sum_gx += gx; // 只累加 X 轴
            sum_gz += gz; // 只累加 Z 轴
        }
        else
        {
            i--; // 如果 I2C 读取失败，这次不算，重读
        }
        
        HAL_Delay(3); // 每次采集停 3ms，防止读取太快拿到重复数据
    }
    // 计算平均偏移量
    gyro_x_offset = (float)sum_gx / sample_count;
    gyro_z_offset = (float)sum_gz / sample_count;
}

/*================================== 卡尔曼滤波 ==================================*/

// 初始化参数
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_gyro  = 0.003f,
    .R_angle = 0.5f,
    .angle   = 0.0f,
    .bias    = 0.0f,
    .P = {{1, 0}, {0, 1}}
};


/**
 * @description: 卡尔曼滤波函数
 * @param {Kalman_t} *k 卡尔曼滤波器结构体指
 * @param {float} newAngle 新的角度测量值
 * @param {float} newGyro 新的陀螺仪角速度测量值
 * @param {float} dt 时间间隔
 * @return {float} 滤波后的角度值
 */
float Kalman_Filter(Kalman_t *k, float newAngle, float newGyro, float dt) {
    // 1.根据陀螺仪猜当前角度：角度 = 旧角度 + (角速度 - 零偏) * 时间
    k->angle += dt * (newGyro - k->bias);

    // 看看我这次猜得准不准，P 越大 → 说明我对自己的预测越不自信
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    // 陀螺零偏是会慢慢漂移的，因此给 bias 加入过程噪声 Q_gyro
    k->P[1][1] += k->Q_gyro * dt;
    
    // 3.看看该信谁多一点
    float S = k->P[0][0] + k->R_angle;
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    // 4.用加速度计获取的实时角度看看我猜错了多少
    float y = newAngle - k->angle;  // 加速度计测得的角度 - 刚刚用陀螺猜的角度

    // 5.根据猜错的值来修正我的猜测
    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    // 5.看看我的不自信程度降低了多少
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];
    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    // 返回滤波后的角度
    return k->angle;
}

/*================================== 姿态解算 ==================================*/

volatile float gyro_angle = 0; // X轴角度
volatile float fused_angle = 0; // 融合后的角度
float gyro_x_speed;  // x轴角速度(直立环需要)
float gyro_z_speed;  // Z轴角速度（转向环用）

void MPU6050_get_FusedAngle_Optimized(float dt)
{
    uint8_t buff[14];
    short ax, ay, az, gx, gy, gz;
    float accel_angle_local;
    
    // 1. 一次性读取 14 个字节 (Accel X/Y/Z, Temp, Gyro X/Y/Z)
    if (MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buff, 14) != 0) {
        return; // 读取失败直接跳过
    }
    // 2. 原始数据解析 (大端转小端)
    ax = ((uint16_t)buff[0] << 8) | buff[1];
    ay = ((uint16_t)buff[2] << 8) | buff[3];
    az = ((uint16_t)buff[4] << 8) | buff[5];
    // buff[6][7] 是温度数据，如果不显示可以不管
    gx = ((uint16_t)buff[8] << 8) | buff[9];
    gy = ((uint16_t)buff[10] << 8) | buff[11];
    gz = ((uint16_t)buff[12] << 8) | buff[13];

    // 加速度计获取的角度（直接测量值）
    accel_angle_local = atan2((float)ay, (float)az) * 180.0f / PI;
    // 陀螺仪获取的X轴角速度（动态变化量）
    gyro_x_speed = (gx - gyro_x_offset) / 16.4f;
    // 陀螺仪获取的Z轴角速度
    gyro_z_speed = (gz - gyro_z_offset) / 16.4f;
    // 卡尔曼滤波
    fused_angle = Kalman_Filter(&KalmanX, accel_angle_local, gyro_x_speed, dt);
}
