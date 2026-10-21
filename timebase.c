#include "timebase.h"

/*
 * timebase.c - 5ms 毫秒时基实现。
 *
 * 学习重点：
 * 这里只有一个全局计数器，而且只被定时器中断修改、被任意上下文读取。
 * Cortex-M0+ 对 32 位对齐变量的读写是单条指令，因此"读"不需要关中断；
 * 这也正是为什么本工程所有跨上下文的时间戳都可以直接赋值，而不需要
 * 临界区或内存屏障（单核、无 cache、无乱序执行）。
 */

static volatile uint32_t g_ms_ticks;

void Timebase_Reset(void)
{
    g_ms_ticks = 0U;
}

void Timebase_Advance5ms(void)
{
    g_ms_ticks += 5U;
}

uint32_t Timebase_NowMs(void)
{
    return g_ms_ticks;
}
