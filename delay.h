#ifndef __DELAY_H
#define __DELAY_H

/*
 * delay.h - 忙等待延时接口。
 *
 * 学习重点：
 * 适合初始化、软件 I2C 等短时间阻塞场景；不要在 5ms 实时控制路径中
 * 使用长延时，否则会影响控制周期和串口协议处理。
 */

#include <stdint.h>

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
