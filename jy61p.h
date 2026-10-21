#ifndef JY61P_H
#define JY61P_H

/*
 * jy61p.h - JY61P IMU 数据接口。
 *
 * 调用关系：
 * 1. main 初始化时调用 JY61P_Init()。
 * 2. UARTx_IRQHandler 中调用 JY61P_UART_IRQHandler()。
 * 3. 5ms 定时器中调用 JY61P_UpdateTick()，供在线检测使用。
 * 4. 控制层通过 GetYaw()/GetAccelX() 等函数读取最新物理量。
 */

#include <stdbool.h>
#include <stdint.h>

void JY61P_Init(void);
void JY61P_UART_IRQHandler(void);
void JY61P_UpdateTick(void);

float JY61P_GetRoll(void);
float JY61P_GetPitch(void);
float JY61P_GetYaw(void);
float JY61P_GetGyroX(void);
float JY61P_GetGyroY(void);
float JY61P_GetGyroZ(void);
float JY61P_GetAccelX(void);
float JY61P_GetAccelY(void);
float JY61P_GetAccelZ(void);
int16_t JY61P_GetYawRaw(void);

void JY61P_ZeroYaw(void);
bool JY61P_IsOnline(void);
bool JY61P_AccelIsOnline(void);
uint32_t JY61P_GetFrameCount(void);
uint32_t JY61P_GetAccelFrameCount(void);
uint32_t JY61P_GetAngleFrameCount(void);
uint32_t JY61P_GetGyroFrameCount(void);

#endif
