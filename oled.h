#ifndef __OLED_H
#define __OLED_H

/*
 * oled.h - 128x64 SSD1306 OLED 显示接口（硬件 I2C 版）。
 *
 * 学习重点：
 * 1. OLED_GRAM 是软件显存，绘图函数先改显存，OLED_Refresh() 再刷屏。
 * 2. 坐标系：x=0~127，y=0~63；SSD1306 内部按 8 像素一页(page)组织。
 * 3. 传输层已从"软件 I2C(PA28/PA31 忙等)"换成"硬件 I2C0(PA0/PA1, 400kHz)"，
 *    对外接口完全不变，区别只在时序开销：
 *      软件 I2C：每写 1 个字节发一次完整事务，整屏约 1000+ 次事务
 *      硬件 I2C：整页 = 1 次命令事务 + 1 次数据事务，整屏 16 次事务
 * 4. 本项目主要用于调试显示，不追求复杂 UI，重点显示状态、参数和故障。
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* SysConfig 里实例名是 I2C_Screen（I2C0，SDA=PA0/SCL=PA1，400kHz）。 */
#define OLED_INST I2C_Screen_INST

/* ========== 基本参数 ========== */
#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_PAGES        8
#define OLED_ADDR         0x3C    // 7位I2C地址

/* ========== 命令/数据模式（SSD1306 控制字节） ========== */
#define OLED_CMD          0
#define OLED_DATA         1

/* ========== 显存缓冲区 ========== */
extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];

/* ========== 基础操作 ========== */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearBuffer(void);
void OLED_Refresh(void);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

/* 诊断：一次 I2C 事务失败（超时 / NACK / 总线错误）就 +1。 */
uint32_t OLED_GetErrorCount(void);

/* ========== 显示控制 ========== */
void OLED_ColorTurn(uint8_t i);       // 0:正常 1:反色
void OLED_DisplayTurn(uint8_t i);     // 0:正常 1:旋转180°

/* ========== 绘图功能 ========== */
void OLED_DrawPoint(uint8_t x, uint8_t y);
void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);

/* ========== 显示功能 ========== */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1);
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1);
void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]);

/* ========== 工具函数 ========== */
void OLED_WR_BP(uint8_t x, uint8_t y);

#endif
