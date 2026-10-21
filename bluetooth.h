#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

/*
 * bluetooth.h - HC-05 蓝牙调参接口。
 *
 * 本模块提供一个轻量“串口命令行”：
 *   KP 1.8      修改巡线比例
 *   KD 0.1      修改微分
 *   BASE 1200   修改基础 PWM
 *   START/STOP  发送路线启动/停止请求
 *   SHOW        回传当前参数和运行结果
 *
 * 设计原则：UART 中断只接收字符，命令解析在 BT_Poll() 中完成。
 */

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 这些是蓝牙侧维护的一份参数快照，真正生效还要同步到 Route_* setter。 */
    float   KP;
    float   KI;
    int16_t BASE_PWM;
    int16_t TURN_SPEED;
    int16_t OUTPUT_LIM;
} TuningParams_t;

void BT_Init(void);
void BT_SetTickPtr(volatile uint32_t *tick_ptr);
bool BT_Poll(TuningParams_t *params);
void BT_Send(const char *str);
void BT_Printf(const char *fmt, ...);
void BT_SendParams(const TuningParams_t *p);

void UART_PB_INST_IRQHandler(void);

#endif /* __BLUETOOTH_H__ */
