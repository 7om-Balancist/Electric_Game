/**
 * delay.c - 简单忙等待延时。
 *
 * 学习重点：
 * 1. 这是裸机工程中最基础的阻塞式延时，适合初始化、短 I2C 时序等场景。
 * 2. 忙等待期间 CPU 不能处理主循环任务，所以不能在实时控制路径里大量使用。
 * 3. tick 换算依赖 SYS_FREQ_HZ=32MHz；如果系统主频修改，需要同步调整这里。
 */

#include "delay.h"

#define SYS_FREQ_HZ 32000000U

void delay_ms(uint32_t ms)
{
    /* 经验换算系数，循环体不是精确定时器，只适合作粗略延时。 */
    uint32_t tick = ms * (SYS_FREQ_HZ / 4000);
    for (volatile uint32_t i = 0; i < tick; ) i++;
}

void delay_us(uint32_t us)
{
    /* 微秒延时用于软件 I2C 的短时序，仍然属于阻塞式延时。 */
    uint32_t tick = us * (SYS_FREQ_HZ / 4000000U);
    for (volatile uint32_t i = 0; i < tick; ) i++;
}
