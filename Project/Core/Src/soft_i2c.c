#include "soft_i2c.h"

// 定义软件I2C的引脚
#define GPIO_PORT_IIC GPIOB                         // GPIO端口
#define RCC_IIC_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE() // GPIO端口时钟使能
#define IIC_SCL_PIN GPIO_PIN_4                      // 连接SCL时钟线的引脚
#define IIC_SDA_PIN GPIO_PIN_3                      // 连接SDA数据线的引脚

// // 上下拉SCL和上下拉SDA
// #define IIC_SCL_0() HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET) // 下拉SCL
// #define IIC_SCL_1() HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)   // 上拉SCL
// #define IIC_SDA_0() HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET) // 下拉SDA
// #define IIC_SDA_1() HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)   // 上拉SDA

// // 忘记了要读数据线的数据
// #define IIC_SDA_READ() HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN) // 读数据线数据

// 因为我们是读取MPU6050，所以会频繁的读取数据 => 寄存器来读取
// ================= 寄存器极速操作宏 =================
#define IIC_SCL_1()  GPIO_PORT_IIC->BSRR = IIC_SCL_PIN
#define IIC_SCL_0()  GPIO_PORT_IIC->BSRR = (uint32_t)IIC_SCL_PIN << 16U

#define IIC_SDA_1()  GPIO_PORT_IIC->BSRR = IIC_SDA_PIN
#define IIC_SDA_0()  GPIO_PORT_IIC->BSRR = (uint32_t)IIC_SDA_PIN << 16U

// 读取输入数据寄存器 IDR
#define IIC_SDA_READ()  ((GPIO_PORT_IIC->IDR & IIC_SDA_PIN) != 0)

// for循环延时空转
void IIC_Delay(void)
{
    volatile uint8_t t;  // 告诉编译器这个变量可能会给修改，不要动他
    for (t = 0; t < 15; t++)
    {
        __NOP();   // 告诉编译器，这个不是我漏写逻辑，是真的要空转
    }
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Start
*	功能说明: CPU发起IIC总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Start(void)
{
    // 先拉高SCL再拉高SDA
    IIC_SDA_1(); // 不需要加延时，因为处于准备阶段
    IIC_SCL_1();
    IIC_Delay(); // 延时给总线反应时间

    // 拉低SDA
    IIC_SDA_0();
    IIC_Delay();

    // 拉低SCL
    IIC_SCL_0();
    IIC_Delay();
    return;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Stop
*	功能说明: CPU发起IIC总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Stop(void)
{
    // 保证SDA为低电平
    IIC_SCL_0();
    IIC_Delay();
    IIC_SDA_0();
    IIC_Delay();

    // 拉高SCL
    IIC_SCL_1();
    IIC_Delay();

    // 再上拉SDA
    IIC_SDA_1();
    IIC_Delay();

    return;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_SendByte
*	功能说明: CPU向IIC总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Send_Byte(uint8_t _ucByte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80) // 获取最高位 0x80 = 1000 0000
        {
            IIC_SDA_1();
        }
        else IIC_SDA_0();
        IIC_Delay();
        IIC_SCL_1();  // 读取数据
        IIC_Delay();
        IIC_SCL_0();  // 让SDA电平变化
        if (i == 7)
        {
            IIC_SDA_1();  // 最后一次结束后释放 SDA 总线(置 1)
        }
        _ucByte <<= 1;  // 高位前挪
        IIC_Delay();
    }
    return;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_Ack(void)
{
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
    IIC_SDA_1();
    IIC_Delay();
    return;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_NAck(void)
{
    IIC_SDA_1();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
    return;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_ReadByte
*	功能说明: CPU从IIC总线设备读取8bit数据
*	形    参：ack : 0=不应答，1=应答
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t value = 0;
    IIC_SDA_1(); // 关键：读取前释放SDA，让从机控制
    IIC_Delay();
    for (uint8_t i = 0; i < 8; i++)
    {
        value <<= 1;  // 最高位读取
        IIC_SCL_0();      
        IIC_Delay();
        IIC_SCL_1();        
        IIC_Delay();
        if (IIC_SDA_READ()) {
            value |= 0x01;   
        }
        // 为什么不放在循环外面：让从机有充足时间放数据
        IIC_SCL_0();
        IIC_Delay();
    }
    if (ack == 0) IIC_NAck();
    else IIC_Ack();
    return value;
}

/*
*********************************************************************************************************
*	函 数 名: IIC_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件应答
*********************************************************************************************************
*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;   // SCL高电平期间SDA的电平
    IIC_SDA_1();  // 释放SDA
    IIC_Delay();
    IIC_SCL_1();  // 上拉SCL进入高电平
    IIC_Delay();
    if (IIC_SDA_READ()) re = 1;
    else re = 0;
    IIC_SCL_0();  // 下拉SCL，高电平结束
    IIC_Delay();
    return re;
}


/*
*********************************************************************************************************
*	函 数 名: IIC_GPIO_Init
*	功能说明: 配置IIC总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_IIC_ENABLE;	/* 打开GPIO时钟 */

    // 2. 开启 AFIO 时钟 (必须！)
    __HAL_RCC_AFIO_CLK_ENABLE(); 

    // 3. 关闭 JTAG，保留 SWD，释放 PB3/PB4 (必须！)
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitStructure.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  	/* 开漏输出 */
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStructure);

    /* 给一个停止信号, 复位IIC总线上的所有设备到待机模式 */
    IIC_Stop();
}

