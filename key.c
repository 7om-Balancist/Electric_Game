/**
 * key.c - 四个独立按键的“中断唤醒 + 周期消抖 + 事件读取”驱动。
 *
 * 学习重点：
 * 1. 按键硬件通常使用上拉输入，按下时 GPIO 读到低电平，所以
 *    Key_ReadRawPressed() 中用 `readPins(...) == 0` 判断“按下”。
 * 2. GPIO 中断只负责告诉软件“可能有按键变化”，真正的消抖放在
 *    Key_Scan5ms() 里做。这样可以避免在中断里延时等待机械抖动结束。
 * 3. 对外不直接暴露电平状态，而是暴露“一次按下事件”。主循环调用
 *    Key_GetPressEvent() 后事件会被清除，适合 START/STOP/RESET 这类命令。
 */

#include "key.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_TICKS 4U

static uint8_t g_key_cnt[KEY_ID_NUM];
static bool g_key_state[KEY_ID_NUM];
static volatile bool g_key_press_event[KEY_ID_NUM];
static volatile bool g_key_irq_pending;

static bool Key_ReadRawPressed(KeyId_t id)
{
    uint32_t pin = 0U;

    switch (id) {
    case KEY_ID_1:
        pin = KEY_GPIO_PB1_PIN;
        break;
    case KEY_ID_2:
        pin = KEY_GPIO_PB10_PIN;
        break;
    case KEY_ID_3:
        pin = KEY_GPIO_PB11_PIN;
        break;
    case KEY_ID_4:
        pin = KEY_GPIO_PB14_PIN;
        break;
    default:
        return false;
    }

    /* 按键引脚配置为内部上拉：未按下=1，按下接地=0。 */
    return (DL_GPIO_readPins(KEY_PORT, pin) == 0U);
}

void Key_Init(void)
{
    /* 初始化软件状态时读取一次真实电平，避免上电瞬间产生假按下事件。 */
    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        g_key_cnt[i] = 0U;
        g_key_state[i] = Key_ReadRawPressed((KeyId_t)i);
        g_key_press_event[i] = false;
    }

    g_key_irq_pending = false;
    /*
     * 四个按键均使用下降沿中断：松开时为高电平，按下瞬间出现下降沿。
     * 注意：这里的中断不是用来“确认按下”，只是用来提示扫描逻辑有变化。
     */
    DL_GPIO_setLowerPinsPolarity(KEY_PORT,
        DL_GPIO_PIN_1_EDGE_FALL | DL_GPIO_PIN_10_EDGE_FALL |
        DL_GPIO_PIN_11_EDGE_FALL | DL_GPIO_PIN_14_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(KEY_PORT,
        KEY_GPIO_PB1_PIN | KEY_GPIO_PB10_PIN |
        KEY_GPIO_PB11_PIN | KEY_GPIO_PB14_PIN);
    NVIC_ClearPendingIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(KEY_INT_IRQN);
}

void Key_GPIO_IRQHandler(void)
{
    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(KEY_PORT);

    /*
     * 只记录 pending 标志，不在 ISR 中做业务动作。
     * 这样可以保证按键中断不会直接打断电机控制状态机。
     */
    if ((iidx == KEY_GPIO_PB1_IIDX) ||
        (iidx == KEY_GPIO_PB10_IIDX) ||
        (iidx == KEY_GPIO_PB11_IIDX) ||
        (iidx == KEY_GPIO_PB14_IIDX)) {
        g_key_irq_pending = true;
        DL_GPIO_clearInterruptStatus(KEY_PORT,
            KEY_GPIO_PB1_PIN | KEY_GPIO_PB10_PIN |
            KEY_GPIO_PB11_PIN | KEY_GPIO_PB14_PIN);
    }
}

void Key_Scan5ms(void)
{
    (void)g_key_irq_pending;

    /*
     * 周期消抖算法：
     * - 如果当前原始电平等于稳定状态，计数清零；
     * - 如果连续 KEY_DEBOUNCE_TICKS 次扫描都不同，才承认状态翻转；
     * - 只有“翻转为按下”时产生事件，松开不产生事件。
     *
     * 当前扫描周期为 5ms，KEY_DEBOUNCE_TICKS=4，相当于约 20ms 消抖。
     */
    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        bool raw_pressed = Key_ReadRawPressed((KeyId_t)i);

        if (raw_pressed == g_key_state[i]) {
            g_key_cnt[i] = 0U;
        } else if (++g_key_cnt[i] >= KEY_DEBOUNCE_TICKS) {
            g_key_cnt[i] = 0U;
            g_key_state[i] = raw_pressed;

            if (raw_pressed) {
                g_key_press_event[i] = true;
            }
        }
    }

    g_key_irq_pending = false;
}

bool Key_GetPressEvent(KeyId_t id)
{
    bool event;

    if (id >= KEY_ID_NUM) {
        return false;
    }

    /*
     * 事件标志可能在 5ms 扫描中被写入，读取并清除必须做成临界区，
     * 否则主循环和中断可能同时访问同一个 volatile 标志。
     */
    __disable_irq();
    event = g_key_press_event[id];
    g_key_press_event[id] = false;
    __enable_irq();

    return event;
}
