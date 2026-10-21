/**
 * ir_sensor.h — 红外测距传感器 (ADC, PA15, 2.5V 内部参考)
 */
#ifndef __IR_SENSOR_H__
#define __IR_SENSOR_H__

/*
 * ir_sensor.h - 红外/模拟量 ADC 读取接口。
 *
 * 学习重点：
 * IR_Read() 返回 ADC 原始采样值。若要得到距离，需要根据实际传感器曲线
 * 做标定或查表换算。
 */

#include "ti_msp_dl_config.h"

uint16_t IR_Read(void);   /* 返回 0~4095 */

#endif
