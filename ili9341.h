/**
 * ili9341.h — ILI9341 SPI TFT 屏幕驱动 (320x240)
 *
 * 引脚: PB8=MOSI, PB9=SCK, PB10=RES, PB11=DC, PB14=CS, PB13=BLK
 */

#ifndef ILI9341_H
#define ILI9341_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define ILI9341_WIDTH  240
#define ILI9341_HEIGHT 320

#define COLOR_WHITE     0xFFFF
#define COLOR_BLACK     0x0000
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0

void ILI9341_Init(void);
void ILI9341_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_ClearScreen(uint16_t color);
void ILI9341_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);

#endif
