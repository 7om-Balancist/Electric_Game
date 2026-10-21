/*
 * zdt_stepper.h - ZDT Emm V5 闭环步进驱动器串口协议封装。
 *
 * 本模块把驱动器命令抽象为：
 * - Enable/Disable：电机使能或失能
 * - Stop：停止当前运动
 * - MoveAbsolute：发送绝对位置目标
 * - RequestPosition：读取驱动器编码器位置
 *
 * 上层不直接拼帧，只调用这些函数。
 *
 * 学习重点：
 * 这是典型的“协议封装层”：把串口帧、ACK、超时和错误统计藏在模块内部。
 */

#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* 驱动器未使能或已失能。 */
    ZDT_STATE_DISABLED = 0,
    /* UART 空闲且可接收新命令。 */
    ZDT_STATE_READY,
    /* 已发送运动命令，等待 ACK 或运动状态刷新。 */
    ZDT_STATE_MOVING,
    /* NACK、连续超时或解析错误导致的故障状态。 */
    ZDT_STATE_FAULT
} ZdtState_t;

void Zdt_Init(void);
void Zdt_UART_IRQHandler(void);
void Zdt_Poll(uint32_t now_ms);

bool Zdt_Enable(uint32_t now_ms);
bool Zdt_Disable(uint32_t now_ms);
bool Zdt_Stop(uint32_t now_ms);
bool Zdt_MoveAbsolute(int32_t target_pulse, uint16_t rpm,
                      uint8_t acceleration, uint32_t now_ms);
bool Zdt_RequestPosition(uint32_t now_ms);

bool Zdt_IsCommandPending(void);
bool Zdt_HasPosition(void);
bool Zdt_PositionIsFresh(uint32_t now_ms, uint32_t max_age_ms);
int32_t Zdt_GetPositionPulse(void);
uint32_t Zdt_GetPositionAgeMs(uint32_t now_ms);
ZdtState_t Zdt_GetState(void);
uint32_t Zdt_GetTimeoutCount(void);
uint32_t Zdt_GetErrorCount(void);

#endif
