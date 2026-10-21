#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/*
 * timebase.h - 全工程统一的毫秒时基。
 *
 * 学习重点：
 * 1. 5ms 控制节拍由 TIMG6 产生，时基只在定时器中断里推进。
 * 2. 任何模块需要打时间戳，都调用 Timebase_NowMs()，不要自己去读别的
 *    模块里的静态变量，也不要各模块各维护一个计数器。
 * 3. 时间比较一律写成无符号差值形式：
 *        (uint32_t)(now - then) <= timeout_ms
 *    这样天然可以安全跨越 32 位回绕（即使用 49 天也不会出错）。
 *
 * 分辨率的已知限制：
 * 本时基的分辨率是 5ms（等于控制节拍周期）。因此两帧之间的时间差只能
 * 取到 15ms / 20ms 这种 5 的倍数，而真实帧间隔是 16.67ms（60Hz），
 * 于是速度估计会带上约 +/-15% 的量化抖动。
 * 若将来要消除它，TIMG6 的预分频是 256、装载值是 624，即 625 个计数对应
 * 5ms，一个计数 = 8us。可以在本模块里补一个 Timebase_NowUs()：
 * 读 CNT 与读 g_ms_ticks 之间有竞态，需要用"读-读-再读"的方式重试，
 * 或者干脆把 g_us_ticks 也放在定时器中断里累加。
 * 目前控制律对这点抖动不敏感，所以先保持毫秒精度，把问题记录在这里。
 */

/* 上电或软复位时清零。必须在使能定时器中断之前调用。 */
void Timebase_Reset(void);

/* 由 CTRL_TIMER 的 5ms 中断调用，把时基推进 5ms。 */
void Timebase_Advance5ms(void);

/* 读取当前毫秒数。中断和前台任务都可以安全调用。 */
uint32_t Timebase_NowMs(void);

#endif
