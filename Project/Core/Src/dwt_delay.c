#include "dwt_delay.h"

// 用于保存每微秒的 CPU 周期数
static uint32_t cycles_per_us = 0;

/**
 * @brief  初始化 DWT 计数器
 */
void DWT_Delay_Init(void)
{
    /* 1. 确保 SystemCoreClock 变量是最新的 */
    SystemCoreClockUpdate();

    /* 2. 计算每微秒需要的时钟周期数 */
    // SystemCoreClock 是 HAL 库维护的全局变量，单位是 Hz (例如 72000000)
    cycles_per_us = SystemCoreClock / 1000000;

    /* 3. 开启 DWT 外设 */
    // CoreDebug->DEMCR 的 TRCENA 位必须置 1，才能使用 DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 4. 清除周期计数器CYCCNT */
    DWT->CYCCNT = 0;

    /* 5. 开启 CYCCNT 计数 */
    // DWT_CTRL 的 CYCCNTENA 位置 1
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  微秒级延时
 * @param  us: 延时微秒数
 */
void Delay_us(uint32_t us)
{
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = us * cycles_per_us;

    /* 
     * 使用无符号 32 位减法处理溢出回绕。
     * 即使 DWT->CYCCNT 溢出（从 0xFFFFFFFF 变回 0），
     * (current_tick - start_tick) 的结果仍然是正确的差值。
     */
    while ((DWT->CYCCNT - start_tick) < delay_ticks);
}

/**
 * @brief  毫秒级延时 (封装 Delay_us)
 * @param  ms: 延时毫秒数
 */
void Delay_ms(uint32_t ms)
{
    // 将毫秒转换为微秒进行延时
    // 注意：若 ms 很大，ms*1000 可能会导致 uint32_t 溢出，但在 Delay_us 内部处理即可
    // 为了安全，可以用循环
    while (ms--)
    {
        Delay_us(1000);
    }
}