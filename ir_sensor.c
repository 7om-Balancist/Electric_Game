/**
 * ir_sensor.c — 红外测距 ADC 采集 (PA15)
 *
 * 学习重点：
 * 1. SysConfig 中将 PA15 配置为 ADC12 输入，并开启 repeatMode。
 * 2. IR_Read() 主动触发一次转换，然后读取 MEM0 结果。
 * 3. 当前返回的是 ADC 原始值，不在驱动层做距离换算；距离标定可在上层完成。
 */
#include "ir_sensor.h"

uint16_t IR_Read(void)
{
    /* 启动 ADC12 转换；这里使用短忙等待等待采样完成。 */
    DL_ADC12_startConversion(ir_sensor_INST);
    for (volatile uint16_t d = 0; d < 500; d++);
    return (uint16_t)DL_ADC12_getMemResult(ir_sensor_INST, DL_ADC12_MEM_IDX_0);
}
