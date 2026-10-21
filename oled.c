#include "oled.h"
#include "oledfont.h"
#include "delay.h"
#include <string.h>

/*
 * oled.c - SSD1306 OLED 硬件 I2C 驱动 + 显存绘图。
 *
 * 学习重点：
 * 1. OLED_GRAM[128][8] 是本地显存，8 个 page 每个 page 对应竖向 8 像素。
 * 2. DrawPoint/ShowChar/ShowString 只修改显存并标记 dirty。
 * 3. OLED_Refresh() 把脏页写到屏幕，避免每画一个点就发一次 I2C。
 * 4. 传输层已从"PA28/PA31 软件 I2C"换成"硬件 I2C0（PA0/PA1，400kHz）"：
 *    软件 I2C 每写 1 字节要发一次完整事务（Start+地址+控制+数据+Stop），
 *    整屏 1048 次事务、每 bit 还有 2 次 delay_us(2) 忙等，主循环被占死；
 *    硬件 I2C 一次事务可以连续发任意长度，整页只需 2 次事务。
 * 5. 这次换血同时解掉了"相机 UART0 与 OLED 抢 PA28/PA31"的引脚冲突。
 */

uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];
static uint8_t OLED_Dirty;
static uint8_t OLED_First = 1;

/* 注意：旧的软件 I2C 引脚定义(PA28/PA31)已删除——那对引脚现在是 UART0(MaixCAM) 在用，
 * 两套功能抢同一对 PINCM 是之前相机收不到帧的根因，改用硬件 I2C0 后彻底分开。 */


/*
 * ============================ 硬件 I2C 传输层 ============================
 * SysConfig 实例：I2C_Screen = I2C0，SDA=PA0(PINCM1)，SCL=PA1(PINCM2)，400kHz。
 *
 * 采用"轮询 + FIFO 流式"写法，不占用任何中断：
 *   1. 等总线 IDLE；
 *   2. startControllerTransfer 一次性声明本次总长度（控制字节 + 数据）；
 *   3. 按 FIFO 空位逐个塞字节（TX FIFO 只有 8 级，129 字节不可能一次填完）；
 *   4. 等 BUSY 清零，再检查 ERROR 位。
 *
 * SSD1306 的控制字节：0x00 = 后面全是命令，0x40 = 后面全是显存数据。
 * 400kHz 下整屏 8 页约 25ms；旧的软件 I2C 版本约 250ms。
 */

#include <stdbool.h>

/* SSD1306 用 7 位地址，驱动库要求 8 位（左移一位）。 */
#define OLED_ADDR_8BIT        ((uint16_t)(OLED_ADDR << 1))
#define OLED_CTRL_CMD         0x00U
#define OLED_CTRL_DATA        0x40U

/* 三个超时保护：任何一处卡住都返回失败，绝不让主循环无限死等。 */
#define OLED_IDLE_GUARD       20000U
#define OLED_FIFO_GUARD       20000U
#define OLED_BUSY_GUARD       200000U

/* 同一页的 128 字节在 GRAM[x][page] 里跨步为 8，先搬到这个缓冲再整页发。 */
static uint8_t s_page_buf[OLED_WIDTH];
static uint32_t s_i2c_errors;

uint32_t OLED_GetErrorCount(void) { return s_i2c_errors; }

static bool OLED_I2C_WaitIdle(void)
{
    uint32_t guard = OLED_IDLE_GUARD;
    while ((DL_I2C_getControllerStatus(I2C_Screen_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) return false;
    }
    return true;
}

static bool OLED_I2C_Push(uint8_t byte)
{
    uint32_t guard = OLED_FIFO_GUARD;
    while (DL_I2C_isControllerTXFIFOFull(I2C_Screen_INST)) {
        if (--guard == 0U) return false;
    }
    DL_I2C_transmitControllerData(I2C_Screen_INST, byte);
    return true;
}

/*
 * 发一次完整事务：1 个控制字节 + length 个数据字节。
 * 返回 false 表示超时 / NACK / 总线错误；调用者可以忽略，
 * 因为显示出问题不该打断电机控制。
 */
static bool OLED_I2C_Write(uint8_t control, const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint32_t guard;
    bool ok;

    if (!OLED_I2C_WaitIdle()) { s_i2c_errors++; return false; }

    DL_I2C_startControllerTransfer(I2C_Screen_INST, OLED_ADDR_8BIT,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   (uint16_t)(length + 1U));
    /* errata I2C_ERR_13：启动传输后要等几个功能时钟再填 FIFO。 */
    DL_Common_delayCycles(8U);

    ok = OLED_I2C_Push(control);
    for (i = 0U; ok && (i < length); i++) {
        ok = OLED_I2C_Push(data[i]);
    }
    if (!ok) { s_i2c_errors++; return false; }

    guard = OLED_BUSY_GUARD;
    while ((DL_I2C_getControllerStatus(I2C_Screen_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (--guard == 0U) { s_i2c_errors++; return false; }
    }

    if ((DL_I2C_getControllerStatus(I2C_Screen_INST) &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        s_i2c_errors++;
        return false;
    }
    return OLED_I2C_WaitIdle();
}

/* 单字节兼容层：初始化序列、旋转、开关显示等老调用点完全不用改。 */
static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    (void)OLED_I2C_Write(mode ? OLED_CTRL_DATA : OLED_CTRL_CMD, &dat, 1U);
}

/* 连续几个命令字节合并成一次事务。 */
static void OLED_WR_Cmds(const uint8_t *cmds, uint16_t count)
{
    (void)OLED_I2C_Write(OLED_CTRL_CMD, cmds, count);
}
void OLED_ColorTurn(uint8_t i)
{
    if (i == 0) OLED_WR_Byte(0xA6, OLED_CMD);  // 姝ｅ父鏄剧ず
    if (i == 1) OLED_WR_Byte(0xA7, OLED_CMD);  // 鍙嶈壊鏄剧ず
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);  // 姝ｅ父鏄剧ず
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD);  // 鏃嬭浆180掳
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

/* ========== 鍩虹鎿嶄綔 ========== */
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x14, OLED_CMD);  // 寮€鍚數鑽锋车
    OLED_WR_Byte(0xAF, OLED_CMD);  // 鐐逛寒灞忓箷
}

void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x10, OLED_CMD);  // 鍏抽棴鐢佃嵎娉?
    OLED_WR_Byte(0xAE, OLED_CMD);  // 鍏抽棴灞忓箷
}

void OLED_Refresh(void)
{
    /*
     * 只刷新 dirty page：首次刷新写 8 页，之后只写被修改过的页。
     * OLED_Dirty 的每一位对应一个 page。
     * 每页 = 2 次 I2C 事务（3 字节定位命令 + 128 字节显存数据）。
     */
    uint8_t mask = OLED_First ? 0xFFU : OLED_Dirty;
    OLED_First = 0U;
    OLED_Dirty = 0U;

    for (uint8_t p = 0U; p < OLED_PAGES; p++) {
        uint8_t hdr[3];

        if ((mask & (uint8_t)(1U << p)) == 0U) continue;

        for (uint8_t n = 0U; n < OLED_WIDTH; n++) {
            s_page_buf[n] = OLED_GRAM[n][p];
        }
        hdr[0] = (uint8_t)(0xB0U + p);   /* 设置 page 地址 */
        hdr[1] = 0x00U;                  /* 列地址低 4 位 */
        hdr[2] = 0x10U;                  /* 列地址高 4 位 */
        OLED_WR_Cmds(hdr, 3U);
        (void)OLED_I2C_Write(OLED_CTRL_DATA, s_page_buf, OLED_WIDTH);
    }
}

void OLED_Clear(void)
{
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
    OLED_Dirty = 0xFF;
    OLED_First = 1;
    OLED_Refresh();
}

void OLED_ClearBuffer(void)
{
    /* 只清软件显存，不立即写屏；后续 OLED_Refresh() 才真正更新屏幕。 */
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
    OLED_Dirty = 0xFF;
    OLED_First = 1;
}

/* ========== 缁樺浘鍔熻兘 ========== */
void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    /* y/8 选择 page，y%8 选择该 page 内的具体 bit。 */
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t pg = y / 8;
    OLED_GRAM[x][pg] |= (1 << (y % 8));
    OLED_Dirty |= (1 << pg);
}

void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t pg = y / 8;
    OLED_GRAM[x][pg] &= ~(1 << (y % 8));
    OLED_Dirty |= (1 << pg);
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    if (x1 >= OLED_WIDTH || x2 >= OLED_WIDTH || y1 >= OLED_HEIGHT || y2 >= OLED_HEIGHT) return;
    if (x1 > x2 || y1 > y2) return;

    if (x1 == x2) {  // 绔栫嚎
        for (uint8_t i = 0; i < (y2 - y1); i++)
            OLED_DrawPoint(x1, y1 + i);
    } else if (y1 == y2) {  // 妯嚎
        for (uint8_t i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1);
    } else {  // 鏂滅嚎
        uint8_t k1 = y2 - y1;
        uint8_t k2 = x2 - x1;
        uint8_t k = k1 * 10 / k2;
        for (uint8_t i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a = 0, b = r, num;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);
        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);

        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

/* ========== 鏄剧ず鍔熻兘 ========== */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t y0 = y;
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';

    for (i = 0; i < size2; i++) {
        if (size1 == 12) temp = asc2_1206[chr1][i];
        else if (size1 == 16) temp = asc2_1608[chr1][i];
        else if (size1 == 24) temp = asc2_2412[chr1][i];
        else return;

        for (m = 0; m < 8; m++) {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1)
{
    /* 从左到右逐字符绘制，只显示 ASCII 可见字符。 */
    while ((*chr >= ' ') && (*chr <= '~')) {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > OLED_WIDTH - size1) {  // 鎹㈣
            x = 0;
            y += size1;
        }
        chr++;
    }
}

static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1)
{
    uint8_t t, temp;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1)
{
    uint8_t i, m, n = 0, temp, chr1;
    uint8_t x0 = x, y0 = y;
    uint8_t size3 = size1 / 8;

    while (size3--) {
        chr1 = num * size1 / 8 + n;
        n++;
        for (i = 0; i < size1; i++) {
            if (size1 == 16) temp = Hzk1[chr1][i];
            else if (size1 == 24) temp = Hzk2[chr1][i];
            else if (size1 == 32) temp = Hzk3[chr1][i];
            else if (size1 == 64) temp = Hzk4[chr1][i];
            else return;

            for (m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y);
                else OLED_ClearPoint(x, y);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == size1) { x = x0; y0 = y0 + 8; }
            y = y0;
        }
    }
}

void OLED_WR_BP(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xB0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0F) | 0x01, OLED_CMD);
}

void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[])
{
    uint32_t j = 0;
    uint8_t x, y;

    if (y0 % 8 == 0) y = 0;
    else y = y0 + 1;

    for (y = y0; y < y1; y++) {
        OLED_WR_BP(x0, y);
        for (x = x0; x < x1; x++) {
            OLED_WR_Byte(BMP[j], OLED_DATA);
            j++;
        }
    }
}

/* ========== 鍒濆鍖?========== */
void OLED_Init(void)
{
    /*
     * SSD1306 初始化序列：关闭显示 -> 配置时钟/扫描方向/寻址模式/电荷泵 ->
     * 清屏 -> 开启显示。初始化命令通常直接来自芯片手册或常用模板。
     */
    delay_ms(100);  // 绛夊緟涓婄數澶嶄綅

    OLED_WR_Byte(0xAE, OLED_CMD);  // 鍏抽棴鏄剧ず
    OLED_WR_Byte(0xD5, OLED_CMD);  // 璁剧疆鏃堕挓鍒嗛
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);  // 璁剧疆澶嶇敤鐜?
    OLED_WR_Byte(0x3F, OLED_CMD);  // 1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD);  // 璁剧疆鏄剧ず鍋忕Щ
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);  // 璁剧疆璧峰琛?
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);  // 璁剧疆瀵诲潃妯″紡
    OLED_WR_Byte(0x02, OLED_CMD);  // Page addressing
    OLED_WR_Byte(0xA1, OLED_CMD);  // Segment remap
    OLED_WR_Byte(0xC8, OLED_CMD);  // COM鎵弿鏂瑰悜
    OLED_WR_Byte(0xDA, OLED_CMD);  // COM寮曡剼閰嶇疆
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);  // 璁剧疆瀵规瘮搴?
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);  // 棰勫厖鐢靛懆鏈?
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);  // VCOMH鐢靛帇
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);  // 鎭㈠鏄剧ず鍐呭
    OLED_WR_Byte(0xA6, OLED_CMD);  // 姝ｅ父鏄剧ず

    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);  // 寮€鍚樉绀?
}
