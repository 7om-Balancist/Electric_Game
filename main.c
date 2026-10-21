#include "ti_msp_dl_config.h"

/*
 * main.c - 当前 BUJIN 综合工程主程序。
 *
 * 学习主线：
 * 1. SysConfig 初始化所有外设；
 * 2. 初始化按键、OLED、视觉链路、JY61P、ZDT 步进驱动和小球控制器；
 * 3. 5ms 定时器产生统一时基和控制节拍，主循环按节拍调用 BallControl_Update5ms()；
 * 4. UART0/1/2 中断分别转发给 Vision/JY61P/ZDT 协议层；
 * 5. OLED 以 100ms 周期刷新，用于观察状态、传感器在线情况和控制量。
 *
 * 设计原则：
 * 中断服务函数尽量短，复杂控制、协议解析和显示刷新都放在前台循环执行。
 *
 * 调度模型（理解这一点就理解了整个工程）：
 *   视觉 PD 由相机帧驱动（来一帧新样本才算一次），
 *   ZDT 命令由驱动器 ACK 驱动（上一条回了才发下一条），
 *   5ms 节拍只负责"来看一眼"。
 * 因此 5ms 节拍一旦被前台卡顿丢掉，损失的是更新率而不是相位，
 * 这也是全工程各处都用"最新覆盖"而不用"排队"的原因。
 */

#include "ball_stepper_control.h"
#include "jy61p.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "timebase.h"
#include "vision_link.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 上电电机自检演示。
 *
 * 这是台架调试阶段的残留功能：它在进入主循环之前使能 ZDT 并走 3200 脉冲
 * （3200 pulse/rev，等于电机整转一圈）。正式跑比赛前必须保持关闭，
 * 否则机构每次上电都会先动一下。
 *
 * 代码保留下来是因为它示范了"如何用 ACK 同步串起两条命令"这个模式：
 * 发使能 → 轮询等 ACK → 再发位置。需要台架自检时把它改成 1。
 */
#define ENABLE_ZDT_BOOT_DEMO    0

static volatile uint8_t g_control_ticks_pending;

static void oled_show_signed(uint8_t x, uint8_t y, int32_t value,
                             uint8_t digits)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t)(x + 6U), y, (uint32_t)value, digits, 12);
}

static void update_oled(uint32_t now_ms)
{
    /*
     * OLED 刷新周期比控制周期慢得多：控制是 5ms，显示是 100ms。
     * 这样既能看清状态，又不会让 I2C/OLED 刷新占用过多 CPU 时间。
     */
    static uint32_t last_update_ms;
    if ((now_ms - last_update_ms) < 100U) return;
    last_update_ms = now_ms;

    OLED_ClearBuffer();

    OLED_ShowString(0, 0, BallControl_GetStateName(), 12);
    OLED_ShowString(50, 0, "F", 12);
    OLED_ShowNum(58, 0, (uint32_t)BallControl_GetFault(), 1, 12);

    OLED_ShowString(0, 13, "X", 12);
    oled_show_signed(8, 13, BallControl_GetBallX(), 3);
    OLED_ShowString(48, 13, "V", 12);
    oled_show_signed(56, 13, (int32_t)BallControl_GetBallVelocity(), 3);

    OLED_ShowString(0, 26, "I", 12);
    oled_show_signed(8, 26,
                     (int32_t)(BallControl_GetImuAcceleration() * 100.0f), 3);
    OLED_ShowString(48, 26, "F", 12);
    oled_show_signed(56, 26,
                     (int32_t)(BallControl_GetFeedforwardDeg() * 10.0f), 3);

    OLED_ShowString(0, 39, "B", 12);
    oled_show_signed(8, 39,
                     (int32_t)(BallControl_GetFeedbackDeg() * 10.0f), 3);
    OLED_ShowString(48, 39, "M", 12);
    oled_show_signed(56, 39,
                     (int32_t)(BallControl_GetCommandDeg() * 10.0f), 3);

    /*
     * 最下面一行是链路健康：
     * CAM 表示传输层在线（在收到 CRC 正确的帧）；C-- 表示连帧都收不到。
     * 注意"CAM 亮着"不等于"控制律有输入"：相机丢球时会持续发无效帧，
     * 那种情况由控制器的 BALL_LOST 故障（F8）负责报出来。
     */
    OLED_ShowString(0, 52, Vision_IsOnline(100U) ? "CAM" : "C--", 12);
    OLED_ShowString(28, 52, JY61P_IsOnline() ? "IMU" : "I--", 12);
    OLED_ShowString(56, 52,
                    (Zdt_GetState() == ZDT_STATE_FAULT) ? "ZERR" : "ZDT", 12);
    OLED_Refresh();
}

#if ENABLE_ZDT_BOOT_DEMO
static void zdt_start_position_demo(void)
{
    /*
     * 上电测试步进电机：
     * 先发送使能命令，等驱动器 ACK 后，再用位置模式走 3200 脉冲。
     * 参考 Emm_V5 例程协议，但复用本工程已有的 Zdt_* 封装。
     */
    uint32_t start_ms = Timebase_NowMs();

    if (!Zdt_Enable(Timebase_NowMs())) {
        return;
    }

    while ((Timebase_NowMs() - start_ms) < 300U) {
        Zdt_Poll(Timebase_NowMs());
        if ((Zdt_GetState() == ZDT_STATE_READY) && !Zdt_IsCommandPending()) {
            break;
        }
        __WFI();
    }

    if ((Zdt_GetState() == ZDT_STATE_READY) && !Zdt_IsCommandPending()) {
        (void)Zdt_MoveAbsolute(3200, 60U, 20U, Timebase_NowMs());
    }
}
#endif

void CTRL_TIMER_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST) == DL_TIMER_IIDX_LOAD) {
        /*
         * 5ms 系统节拍：
         * - Timebase_Advance5ms() 推进全局毫秒时基（所有模块的时间戳都用它）；
         * - JY61P_UpdateTick() 让 IMU 在线检测有时间基准；
         * - Key_Scan5ms() 做按键消抖；
         * - g_control_ticks_pending 是“待处理控制节拍邮箱”，上限 4。
         */
        Timebase_Advance5ms();
        JY61P_UpdateTick();
        Key_Scan5ms();
        if (g_control_ticks_pending < 4U) {
            g_control_ticks_pending++;
        }
    }
}

void GROUP1_IRQHandler(void)
{
    Encoder_ISR();
    Key_GPIO_IRQHandler();
}

void UART0_IRQHandler(void) { Vision_UART_IRQHandler(); }
void UART1_IRQHandler(void) { JY61P_UART_IRQHandler(); }
void UART2_IRQHandler(void) { Zdt_UART_IRQHandler(); }

int main(void)
{
    SYSCFG_DL_init();

    /*
     * 时基必须在使能定时器中断之前清零，否则第一个节拍可能带着旧值。
     */
    Timebase_Reset();
    g_control_ticks_pending = 0U;

    Key_Init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "BALL CTRL", 16);
    OLED_ShowString(0, 24, "WAIT LINKS", 12);
    OLED_Refresh();

    /*
     * 通信与执行机构初始化顺序：
     * 先启动数据链路和驱动器协议层，再初始化控制器状态机。
     * 控制器上电后处于 WAIT_SENSORS，不会立即运动。
     */
    Vision_Init();
    JY61P_Init();
    Zdt_Init();
    BallControl_Init(0U);

    NVIC_ClearPendingIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

#if ENABLE_ZDT_BOOT_DEMO
    zdt_start_position_demo();
#endif

    while (1) {
        uint32_t now_ms = Timebase_NowMs();

        /*
         * 视觉 DMA 完成块在这里解析，避免在 UART/DMA 中断里做 CRC 和拆包。
         * 时间戳已经在中断里打好，所以这里不需要再传时间。
         */
        Vision_Poll();

        /*
         * 取出一个待处理控制 tick。这里一次只消费一个，若主循环偶尔慢于 5ms，
         * pending 会短暂累积，但上限为 4，避免系统长时间补跑旧控制周期。
         */
        __disable_irq();
        bool control_due = (g_control_ticks_pending > 0U);
        if (control_due) g_control_ticks_pending--;
        __enable_irq();

        if (control_due) {
            /* 小球/ZDT 闭环控制的唯一周期入口。 */
            BallControl_Update5ms(now_ms);
        }

        /*
         * 四个按键的语义：
         * KEY1：启动控制；KEY2：IMU 航向归零；
         * KEY3：故障复位；KEY4：主动停止。
         */
        if (Key_GetPressEvent(KEY_ID_1)) {
            (void)BallControl_Start(now_ms);
        }
        if (Key_GetPressEvent(KEY_ID_2)) {
            JY61P_ZeroYaw();
        }
        if (Key_GetPressEvent(KEY_ID_3)) {
            BallControl_ResetFault(now_ms);
        }
        if (Key_GetPressEvent(KEY_ID_4)) {
            BallControl_Stop(now_ms);
        }

        update_oled(now_ms);
        /*
         * Wait For Interrupt：当前台没有工作时让 CPU 休眠，下一次定时器/UART/GPIO
         * 中断会唤醒。裸机低功耗和空闲等待常用这个指令。
         */
        __WFI();
    }
}
