#include "pid.h"
#include <math.h>

/************************ 结构体实例化 ************************/
Upright UprightRing;  // 直立环
Speed SpeedRing;      // 速度环
Turn TurnRing;        // 转向环

/************************ 全局变量声明 ************************/
/*=================== 直立环 ===================*/
float mechanical_balance_angle;   // 机械中值

/*=================== 速度环 ===================*/
volatile uint8_t IsFall = 0;               // 倒地标识位
volatile int target_speed = 0;             // 目标速度
float speed_integral = 0.0f;    // 速度环积分项累加和
float speed_filter_old = 0.0f;  // 保存上一次的滤波结果

/*=================== 转向环 ===================*/
volatile int turn_cmd = 0;   // 转向指令

/************************ 函数定义 ************************/
void PID_Init(void)
{
    // 初始化直立环初始
    UprightRing.Kp = -580.0f; // 极性为-  -580f
    UprightRing.Kd = -3.95f; // 极性为-   -3.95f
    mechanical_balance_angle = 1.9f;

    // 初始化速度环
    SpeedRing.Kp = -0.05f;  // 极性为-    -0.05f
    SpeedRing.Ki = -0.00004f;  // 极性为-  -0.00004f

    // 初始化转向环
    TurnRing.Kp = -19.0f;  // 极性为-  -19f
}

/**
 * @brief  直立环PD控制器
 * @param  current_angle   当前融合角度 (来自 MPU6050)
 * @param  gyro_speed      当前角速度 (来自 MPU6050)
 * @param  target_angle    速度环输出的目标角度
 * @retval int             计算出的直立控制PWM输出值 (-7200 ~ 7200)
 */
int Upright_Control(float current_angle, float gyro_speed, float target_angle)
{
    // 0.计算总的目标期望角度
    // 总期望角度 = 机械中值(静止平衡点) + 速度环想让你倾斜的角度
    float final_target = mechanical_balance_angle + target_angle;

    // 1.计算角度误差
    // 误差 = 当前角度 - 目标角度（机械中值）
    float error = current_angle - final_target;

    // 2.PD控制核心计算
    // 输出PWM = P * 角度误差 + D * 角速度
    int pwm_output = (int)(UprightRing.Kp * error + UprightRing.Kd * gyro_speed);

    // 3.PWM 输出限幅
    if (pwm_output > 7200)  pwm_output = 7200;
    if (pwm_output < -7200)  pwm_output = -7200;

    return pwm_output;
}

/**
 * @brief  速度环PI控制器
 * @param  enc_left   40ms内左轮编码器累计读数
 * @param  enc_right  40ms内右轮编码器累计读数
 * @param  current_angle 当前融合角度
 * @retval float     计算出的目标倾角输出值 (单位：度)
 */
float Speed_Control(int enc_left, int enc_right, float current_angle) 
{
    // 1.计算当前速度
    float current_speed = (float)(enc_left + enc_right) / 2.0f;

    // 2.一阶低通滤波
    float current_speed_filtered = 0.3f * current_speed + 0.7f * speed_filter_old;
    speed_filter_old = current_speed_filtered;

    // 3.计算速度误差 (直接访问全局变量 target_speed)
    float speed_error = (float)target_speed - current_speed_filtered;

    // 4.积分分离与状态检测
    const float stand_up_threshold = 40.0f;  
    
    // 如果角度超过阈值，判定为倒地
    if (fabs(current_angle) > stand_up_threshold) 
    {
        speed_integral = 0.0f; 
        IsFall = 1;
        return 0;         // 倒地直接返回0，不输出角度
    }

    // 5. 动态限幅

    // A. 先计算比例项（P）
    float p_out = SpeedRing.Kp * speed_error;

    // B.定义速度环输出的最大倾角
    const float max_target_angle = 18.0f;

    // C.试探性的累加积分
    float next_integral = speed_integral + speed_error;
    float i_out = SpeedRing.Ki * next_integral;

    // D.检查总输出是否超限
    float total_out = p_out + i_out;
    if (total_out > max_target_angle)  // 超出正限幅
    {
        if(fabs(SpeedRing.Ki) > 0.0001f)  // 防止除0错误
            // 输出的角度 = speed_integral * SpeedRing.Ki + p_out
            speed_integral = (max_target_angle - p_out) / SpeedRing.Ki; // 调整积分到最大允许值
        else
            speed_integral = 0;
    }
    else if (total_out < -max_target_angle)
    {
        if(fabs(SpeedRing.Ki) > 0.0001f)  // 防止除0错误
            speed_integral = (-max_target_angle - p_out) / SpeedRing.Ki; // 调整积分到最小允许值
        else
            speed_integral = 0;
    }
    else
    {
        speed_integral = next_integral; // 积分正常累加
    }

    // 6. 计算最终输出
    float target_angle_out = SpeedRing.Kp * speed_error + SpeedRing.Ki * speed_integral;

    // 6.硬限幅
    if (target_angle_out > max_target_angle) target_angle_out = max_target_angle;
    if (target_angle_out < -max_target_angle) target_angle_out = -max_target_angle;

    return target_angle_out;
}

/**
 * @brief  转向环P控制器
 * @param  target_turn_speed  转向目标值（遥控指令，不转向时为0）
 * @param  current_gyro_z     MPU6050测得的Z轴角速度
 * @retval int                转向补偿PWM
 */
int Turn_Control(float target_turn_speed, float current_gyro_z)
{
    // 1. 计算误差：目标角速度 - 当前角速度
    float error = target_turn_speed - current_gyro_z;
    
    // 2. 计算输出（这里最简单只用P也行）
    // 如果没有遥控指令，target_turn_speed就是0，系统会全力对抗 z 轴的转动
    int pwm_output = (int)(TurnRing.Kp * error);

    // 3. 转向限幅
    if (pwm_output > 1500)  pwm_output = 1500;
    if (pwm_output < -1500) pwm_output = -1500;
    
    return pwm_output;
}
