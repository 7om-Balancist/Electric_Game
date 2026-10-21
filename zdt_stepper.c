#include "zdt_stepper.h"

#include "ti_msp_dl_config.h"

/*
 * zdt_stepper.c - ZDT 闭环步进驱动器通信层。
 *
 * 学习重点：
 * 1. 发送命令前用 g_pending 标记“正在等待 ACK”，防止连续命令互相覆盖。
 * 2. 接收中断只把字节放入环形缓冲区，Zdt_Poll() 在主循环中解析。
 * 3. 解析时根据功能码决定回复帧长度，收到尾字节 0x6B 后才认为一帧完整。
 * 4. 读取编码器返回的是 65536 counts/rev，本模块换算成控制命令使用的
 *    3200 pulses/rev，供上层统一使用“命令脉冲”坐标系。
 */

#define ZDT_ADDR                    0x01U
#define ZDT_TAIL                    0x6BU
#define ZDT_ACK_OK                  0x02U
#define ZDT_ACK_CONDITION_FAILED    0xE2U
#define ZDT_ACK_COMMAND_ERROR       0xEEU

#define ZDT_CMD_ENABLE              0xF3U
#define ZDT_CMD_STOP                0xFEU
#define ZDT_CMD_POSITION            0xFDU
#define ZDT_CMD_READ_POSITION       0x36U

#define ZDT_POSITION_ABSOLUTE       0x01U
#define ZDT_ENCODER_COUNTS_PER_REV  65536LL
#define ZDT_COMMAND_PULSES_PER_REV  3200LL
#define ZDT_ACK_TIMEOUT_MS          160U
#define ZDT_MAX_TIMEOUTS            3U
#define ZDT_RX_RING_SIZE            32U

static volatile uint8_t g_rx_ring[ZDT_RX_RING_SIZE];
static volatile uint8_t g_rx_write;
static volatile uint8_t g_rx_read;
static volatile uint32_t g_rx_overflow;

static uint8_t g_frame[12];
static uint8_t g_frame_index;
static uint8_t g_frame_expected;
static bool g_pending;
static uint8_t g_pending_command;
static bool g_pending_disable;
static uint32_t g_pending_since_ms;
static uint32_t g_timeout_count;
static uint32_t g_error_count;
static uint8_t g_consecutive_timeouts;
static bool g_has_position;
static int32_t g_position_pulse;
static uint32_t g_position_ms;
static ZdtState_t g_state;

static void send_byte(uint8_t byte)
{
    uint32_t guard = 20000U;
    while ((guard-- > 0U) &&
           !DL_UART_Main_transmitDataCheck(UART_ZDT_INST, byte)) {
    }
}

static void send_frame(const uint8_t *frame, uint8_t length)
{
    for (uint8_t i = 0U; i < length; i++) {
        send_byte(frame[i]);
    }
}

static bool begin_command(uint8_t command, uint32_t now_ms, bool emergency)
{
    /*
     * 普通命令必须等待上一条 ACK；emergency 命令（STOP/DISABLE）允许覆盖，
     * 因为故障停机的优先级高于普通位置控制。
     */
    if (g_pending && !emergency) {
        return false;
    }
    g_pending = true;
    g_pending_command = command;
    g_pending_since_ms = now_ms;
    return true;
}

static uint8_t expected_length(uint8_t function)
{
    switch (function) {
    case ZDT_CMD_ENABLE:
    case ZDT_CMD_STOP:
    case ZDT_CMD_POSITION:
        return 4U;
    case ZDT_CMD_READ_POSITION:
        return 8U;
    default:
        return 4U;
    }
}

static void finish_frame(uint32_t now_ms)
{
    uint8_t function = g_frame[1];

    if (function == ZDT_CMD_READ_POSITION && g_frame_expected == 8U) {
        /*
         * 位置回复帧中的 magnitude 是驱动器编码器计数。
         * 这里换算到 MoveAbsolute 使用的 3200 pulse/rev 坐标系，
         * 这样上层不需要知道驱动器内部编码器分辨率。
         */
        uint32_t magnitude = ((uint32_t)g_frame[3] << 24) |
                             ((uint32_t)g_frame[4] << 16) |
                             ((uint32_t)g_frame[5] << 8) |
                             (uint32_t)g_frame[6];
        int64_t encoder = (g_frame[2] == 0x01U) ? -(int64_t)magnitude
                                                : (int64_t)magnitude;
        int64_t scaled = encoder * ZDT_COMMAND_PULSES_PER_REV;
        int64_t rounded = (scaled >= 0) ?
            (scaled + ZDT_ENCODER_COUNTS_PER_REV / 2) :
            (scaled - ZDT_ENCODER_COUNTS_PER_REV / 2);
        g_position_pulse = (int32_t)(rounded / ZDT_ENCODER_COUNTS_PER_REV);
        g_position_ms = now_ms;
        g_has_position = true;
        g_pending = false;
        g_consecutive_timeouts = 0U;
        if (g_state != ZDT_STATE_DISABLED) {
            g_state = ZDT_STATE_READY;
        }
        return;
    }

    if (g_frame_expected == 4U) {
        uint8_t status = g_frame[2];
        if (status == ZDT_ACK_OK) {
            g_pending = false;
            g_consecutive_timeouts = 0U;
            if (function == ZDT_CMD_ENABLE) {
                g_state = g_pending_disable ? ZDT_STATE_DISABLED
                                            : ZDT_STATE_READY;
            } else if (function == ZDT_CMD_STOP) {
                g_state = ZDT_STATE_READY;
            } else if (function == ZDT_CMD_POSITION) {
                g_state = ZDT_STATE_READY;
            }
        } else if ((status == ZDT_ACK_CONDITION_FAILED) ||
                   (status == ZDT_ACK_COMMAND_ERROR)) {
            g_pending = false;
            g_error_count++;
            g_state = ZDT_STATE_FAULT;
        }
    }
}

static void parse_byte(uint8_t byte, uint32_t now_ms)
{
    /* 简单定长帧解析器：先等地址，再收功能码，随后按功能码预期长度收帧。 */
    if (g_frame_index == 0U) {
        if (byte == ZDT_ADDR) {
            g_frame[g_frame_index++] = byte;
        }
        return;
    }

    if (g_frame_index == 1U) {
        g_frame[g_frame_index++] = byte;
        g_frame_expected = expected_length(byte);
        return;
    }

    g_frame[g_frame_index++] = byte;
    if (g_frame_index < g_frame_expected) {
        return;
    }

    if (g_frame[g_frame_expected - 1U] == ZDT_TAIL) {
        finish_frame(now_ms);
    } else {
        g_error_count++;
    }
    g_frame_index = 0U;
    g_frame_expected = 0U;
}

void Zdt_Init(void)
{
    g_rx_write = 0U;
    g_rx_read = 0U;
    g_rx_overflow = 0U;
    g_frame_index = 0U;
    g_frame_expected = 0U;
    g_pending = false;
    g_pending_command = 0U;
    g_pending_disable = false;
    g_pending_since_ms = 0U;
    g_timeout_count = 0U;
    g_error_count = 0U;
    g_consecutive_timeouts = 0U;
    g_has_position = false;
    g_position_pulse = 0;
    g_position_ms = 0U;
    g_state = ZDT_STATE_DISABLED;

    NVIC_ClearPendingIRQ(UART_ZDT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_ZDT_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_ZDT_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void Zdt_UART_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_ZDT_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte = DL_UART_Main_receiveData(UART_ZDT_INST);
        /*
         * ISR 内只入环形缓冲区，不做协议解析。
         * 环形缓冲区满时记录溢出计数，便于后续诊断 UART 负载是否过高。
         */
        uint8_t next = (uint8_t)((g_rx_write + 1U) % ZDT_RX_RING_SIZE);
        if (next == g_rx_read) {
            g_rx_overflow++;
        } else {
            g_rx_ring[g_rx_write] = byte;
            g_rx_write = next;
        }
        break;
    }
    default:
        break;
    }
}

void Zdt_Poll(uint32_t now_ms)
{
    /* 主循环周期调用：先解析所有已收到字节，再检查 ACK 超时。 */
    while (g_rx_read != g_rx_write) {
        uint8_t byte = g_rx_ring[g_rx_read];
        g_rx_read = (uint8_t)((g_rx_read + 1U) % ZDT_RX_RING_SIZE);
        parse_byte(byte, now_ms);
    }

    if (g_pending && ((now_ms - g_pending_since_ms) > ZDT_ACK_TIMEOUT_MS)) {
        g_pending = false;
        g_timeout_count++;
        g_consecutive_timeouts++;
        if (g_consecutive_timeouts >= ZDT_MAX_TIMEOUTS) {
            g_state = ZDT_STATE_FAULT;
        }
    }
}

bool Zdt_Enable(uint32_t now_ms)
{
    static const uint8_t frame[] = {ZDT_ADDR, ZDT_CMD_ENABLE,
                                    0xABU, 0x01U, 0x00U, ZDT_TAIL};
    if (!begin_command(ZDT_CMD_ENABLE, now_ms, false)) return false;
    g_pending_disable = false;
    send_frame(frame, sizeof(frame));
    return true;
}

bool Zdt_Disable(uint32_t now_ms)
{
    static const uint8_t frame[] = {ZDT_ADDR, ZDT_CMD_ENABLE,
                                    0xABU, 0x00U, 0x00U, ZDT_TAIL};
    if (!begin_command(ZDT_CMD_ENABLE, now_ms, true)) return false;
    g_pending_disable = true;
    send_frame(frame, sizeof(frame));
    g_state = ZDT_STATE_DISABLED;
    return true;
}

bool Zdt_Stop(uint32_t now_ms)
{
    static const uint8_t frame[] = {ZDT_ADDR, ZDT_CMD_STOP,
                                    0x98U, 0x00U, ZDT_TAIL};
    if (!begin_command(ZDT_CMD_STOP, now_ms, true)) return false;
    send_frame(frame, sizeof(frame));
    return true;
}

bool Zdt_MoveAbsolute(int32_t target_pulse, uint16_t rpm,
                      uint8_t acceleration, uint32_t now_ms)
{
    /*
     * 绝对位置帧：
     * target_pulse 是驱动器绝对坐标，不是相对位移。
     * 上层 ball_stepper_control 会先把“相对软件原点角度”换算成绝对脉冲。
     */
    uint32_t magnitude = (target_pulse < 0) ? (uint32_t)(-target_pulse)
                                             : (uint32_t)target_pulse;
    uint8_t frame[13];

    if (!begin_command(ZDT_CMD_POSITION, now_ms, false)) return false;

    frame[0] = ZDT_ADDR;
    frame[1] = ZDT_CMD_POSITION;
    frame[2] = (target_pulse < 0) ? 0x01U : 0x00U;
    frame[3] = (uint8_t)(rpm >> 8);
    frame[4] = (uint8_t)rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(magnitude >> 24);
    frame[7] = (uint8_t)(magnitude >> 16);
    frame[8] = (uint8_t)(magnitude >> 8);
    frame[9] = (uint8_t)magnitude;
    frame[10] = ZDT_POSITION_ABSOLUTE;
    frame[11] = 0x00U;
    frame[12] = ZDT_TAIL;
    send_frame(frame, sizeof(frame));
    g_state = ZDT_STATE_MOVING;
    return true;
}

bool Zdt_RequestPosition(uint32_t now_ms)
{
    static const uint8_t frame[] = {ZDT_ADDR, ZDT_CMD_READ_POSITION, ZDT_TAIL};
    if (!begin_command(ZDT_CMD_READ_POSITION, now_ms, false)) return false;
    send_frame(frame, sizeof(frame));
    return true;
}

bool Zdt_IsCommandPending(void) { return g_pending; }
bool Zdt_HasPosition(void) { return g_has_position; }
bool Zdt_PositionIsFresh(uint32_t now_ms, uint32_t max_age_ms)
{
    return g_has_position && ((now_ms - g_position_ms) <= max_age_ms);
}
int32_t Zdt_GetPositionPulse(void) { return g_position_pulse; }
uint32_t Zdt_GetPositionAgeMs(uint32_t now_ms)
{
    return g_has_position ? (now_ms - g_position_ms) : UINT32_MAX;
}
ZdtState_t Zdt_GetState(void) { return g_state; }
uint32_t Zdt_GetTimeoutCount(void) { return g_timeout_count; }
uint32_t Zdt_GetErrorCount(void) { return g_error_count + g_rx_overflow; }
