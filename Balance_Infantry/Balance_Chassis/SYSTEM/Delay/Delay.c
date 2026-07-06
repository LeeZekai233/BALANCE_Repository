#include "main.h"

/**
  * @brief  微秒级延时（基于SysTick）
  * @param  xus 延时时长（微秒），最大允许值取决于当前系统时钟：
  *             max = 0xFFFFFF / (SystemCoreClock/1000000)
  *             例如 168MHz 时约为 99864 μs (~0.1s)
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * xus;   // 计算所需时钟周期数
    
    // 防止溢出（SysTick为24位计数器）
    if (ticks > 0xFFFFFF) {
        ticks = 0xFFFFFF;
    }
    
    SysTick->LOAD = ticks;               // 设置重装载值
    SysTick->VAL  = 0x00;                // 清空当前计数值
    SysTick->CTRL = 0x00000005;          // 使能定时器，使用内核时钟（HCLK），无中断
    
    while (!(SysTick->CTRL & 0x00010000)); // 等待COUNTFLAG置位（计数到0）
    
    SysTick->CTRL = 0x00000004;          // 关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长（毫秒），范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    while (xms--) {
        Delay_us(1000);
    }
}

/**
  * @brief  秒级延时
  * @param  xs  延时时长（秒），范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    while (xs--) {
        Delay_ms(1000);
    }
}


