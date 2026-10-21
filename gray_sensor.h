/**
 * gray_sensor.h — 8路灰度传感器 (串行移位输出)
 *
 * 依赖: SysConfig 生成的 ti_msp_dl_config.h
 * 引脚: PB6(CLK 输出), PB7(DAT 输入)
 * 协议: CLK 发 8 个脉冲, DAT 线串行读出 8 路灰度值
 *
 * 返回约定：
 * GraySensor_Read() 返回 bit0~bit7；bit=1 表示该通道看到黑线。
 * GraySensor_GetPosition(raw) 是简单平均位置算法，主线巡线状态机使用
 * 更细的加权算法。
 */

#ifndef __GRAY_SENSOR_H__
#define __GRAY_SENSOR_H__

#include "ti_msp_dl_config.h"

uint8_t GraySensor_Read(void);
int16_t GraySensor_GetPosition(uint8_t raw);       /* 用已读 raw 直接算, 不重复读 */

#endif
