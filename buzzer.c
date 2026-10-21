/**
 * buzzer.c - 蜂鸣器的非阻塞鸣叫驱动。
 *
 * 学习重点：
 * 1. Buzzer_Beep(ms) 只设置剩余 tick，不会阻塞等待鸣叫结束。
 * 2. Buzzer_Update5ms() 由 5ms 控制节拍周期调用，负责倒计时关闭蜂鸣器。
 * 3. 当前硬件为低电平有效：GPIO 输出低电平时蜂鸣器响。
 */

#include "buzzer.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_buzzer_ticks;

static void Buzzer_Set(bool on)
{
    /* 低电平有效的蜂鸣器：on=true 时清 GPIO，on=false 时置 GPIO。 */
    if (on) {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
    } else {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
    }
}

void Buzzer_Init(void)
{
    Buzzer_Stop();
}

void Buzzer_Beep(uint32_t duration_ms)
{
    /*
     * 将毫秒转换成 5ms tick，+4 用于向上取整。
     * 例如 1~5ms 都至少响 1 个 tick。
     */
    g_buzzer_ticks = (duration_ms + 4U) / 5U;
    Buzzer_Set(g_buzzer_ticks != 0U);
}

void Buzzer_Update5ms(void)
{
    /* 周期服务函数：每 5ms 调用一次，实现“到时自动关闭”。 */
    if (g_buzzer_ticks == 0U) {
        return;
    }

    g_buzzer_ticks--;
    if (g_buzzer_ticks == 0U) {
        Buzzer_Set(false);
    }
}

void Buzzer_Stop(void)
{
    g_buzzer_ticks = 0U;
    Buzzer_Set(false);
}
