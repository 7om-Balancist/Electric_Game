#include "jy61p.h"

#include "ti_msp_dl_config.h"

/*
 * jy61p.c - JY61P/WIT IMU 串口帧解析。
 *
 * 学习重点：
 * 1. JY61P 持续输出 0x55 开头的 11 字节帧。
 * 2. 第 2 字节表示帧类型：0x51=加速度，0x52=角速度，0x53=欧拉角。
 * 3. 数据为 little-endian int16 原始值，需要乘以比例系数换算成物理量。
 * 4. g_tick_5ms 由控制定时器推进，用于判断 IMU 数据是否超时离线。
 */

#define WIT_HEADER             0x55U
#define WIT_TYPE_ACCEL         0x51U
#define WIT_TYPE_GYRO          0x52U
#define WIT_TYPE_ANGLE         0x53U
#define WIT_FRAME_LENGTH       11U
#define ANGLE_SCALE            (180.0f / 32768.0f)
#define GYRO_SCALE             (2000.0f / 32768.0f)
#define ACCEL_SCALE_MPS2       (16.0f * 9.80665f / 32768.0f)
#define ONLINE_TIMEOUT_TICKS   100U

static volatile uint8_t g_frame[WIT_FRAME_LENGTH];
static volatile uint8_t g_index;
static volatile int16_t g_roll_raw;
static volatile int16_t g_pitch_raw;
static volatile int16_t g_yaw_raw;
static volatile int16_t g_gyro_x_raw;
static volatile int16_t g_gyro_y_raw;
static volatile int16_t g_gyro_z_raw;
static volatile int16_t g_accel_x_raw;
static volatile int16_t g_accel_y_raw;
static volatile int16_t g_accel_z_raw;
static volatile uint32_t g_accel_frames;
static volatile uint32_t g_angle_frames;
static volatile uint32_t g_gyro_frames;
static volatile uint32_t g_last_angle_tick;
static volatile uint32_t g_last_gyro_tick;
static volatile uint32_t g_last_accel_tick;
static volatile uint32_t g_tick_5ms;
static float g_yaw_offset;

static int16_t decode_i16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static void process_frame(void)
{
    /*
     * WIT 协议校验：前 10 字节累加的低 8 位应等于第 11 字节。
     * 校验失败直接丢弃，不更新任何传感器数据。
     */
    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < WIT_FRAME_LENGTH - 1U; i++) {
        checksum = (uint8_t)(checksum + g_frame[i]);
    }
    if (checksum != g_frame[WIT_FRAME_LENGTH - 1U]) {
        return;
    }

    if (g_frame[1] == WIT_TYPE_ACCEL) {
        /* 加速度原始量后续按 16g 满量程换算为 m/s^2。 */
        g_accel_x_raw = decode_i16(g_frame[2], g_frame[3]);
        g_accel_y_raw = decode_i16(g_frame[4], g_frame[5]);
        g_accel_z_raw = decode_i16(g_frame[6], g_frame[7]);
        g_accel_frames++;
        g_last_accel_tick = g_tick_5ms;
    } else if (g_frame[1] == WIT_TYPE_GYRO) {
        g_gyro_x_raw = decode_i16(g_frame[2], g_frame[3]);
        g_gyro_y_raw = decode_i16(g_frame[4], g_frame[5]);
        g_gyro_z_raw = decode_i16(g_frame[6], g_frame[7]);
        g_gyro_frames++;
        g_last_gyro_tick = g_tick_5ms;
    } else if (g_frame[1] == WIT_TYPE_ANGLE) {
        g_roll_raw = decode_i16(g_frame[2], g_frame[3]);
        g_pitch_raw = decode_i16(g_frame[4], g_frame[5]);
        g_yaw_raw = decode_i16(g_frame[6], g_frame[7]);
        g_angle_frames++;
        g_last_angle_tick = g_tick_5ms;
    }
}

void JY61P_Init(void)
{
    g_index = 0U;
    g_roll_raw = 0;
    g_pitch_raw = 0;
    g_yaw_raw = 0;
    g_gyro_x_raw = 0;
    g_gyro_y_raw = 0;
    g_gyro_z_raw = 0;
    g_accel_x_raw = 0;
    g_accel_y_raw = 0;
    g_accel_z_raw = 0;
    g_accel_frames = 0U;
    g_angle_frames = 0U;
    g_gyro_frames = 0U;
    g_last_angle_tick = 0U;
    g_last_gyro_tick = 0U;
    g_last_accel_tick = 0U;
    g_tick_5ms = 0U;
    g_yaw_offset = 0.0f;

    NVIC_ClearPendingIRQ(JY61P_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(JY61P_UART_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(JY61P_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void JY61P_UART_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(JY61P_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte = DL_UART_Main_receiveData(JY61P_UART_INST);

        /*
         * 帧同步：只有 index=0 时遇到 0x55 才开始收帧。
         * 如果中途丢字节，下一次遇到帧头可重新同步。
         */
        if (g_index == 0U) {
            if (byte == WIT_HEADER) {
                g_frame[g_index++] = byte;
            }
            break;
        }

        if (g_index == 1U) {
            g_frame[g_index++] = byte;
            break;
        }

        g_frame[g_index++] = byte;
        if (g_index >= WIT_FRAME_LENGTH) {
            process_frame();
            g_index = 0U;
        }
        break;
    }
    default:
        break;
    }
}

void JY61P_UpdateTick(void)
{
    g_tick_5ms++;
}

float JY61P_GetRoll(void) { return (float)g_roll_raw * ANGLE_SCALE; }
float JY61P_GetPitch(void) { return (float)g_pitch_raw * ANGLE_SCALE; }

float JY61P_GetYaw(void)
{
    float yaw = (float)g_yaw_raw * ANGLE_SCALE - g_yaw_offset;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

float JY61P_GetGyroX(void) { return (float)g_gyro_x_raw * GYRO_SCALE; }
float JY61P_GetGyroY(void) { return (float)g_gyro_y_raw * GYRO_SCALE; }
float JY61P_GetGyroZ(void) { return (float)g_gyro_z_raw * GYRO_SCALE; }
float JY61P_GetAccelX(void) { return (float)g_accel_x_raw * ACCEL_SCALE_MPS2; }
float JY61P_GetAccelY(void) { return (float)g_accel_y_raw * ACCEL_SCALE_MPS2; }
float JY61P_GetAccelZ(void) { return (float)g_accel_z_raw * ACCEL_SCALE_MPS2; }
int16_t JY61P_GetYawRaw(void) { return g_yaw_raw; }

void JY61P_ZeroYaw(void)
{
    /* 将当前航向作为 0 度参考，后续 GetYaw() 返回相对航向。 */
    g_yaw_offset = (float)g_yaw_raw * ANGLE_SCALE;
}

bool JY61P_IsOnline(void)
{
    /*
     * 完整在线要求加速度、角速度、角度三类帧都收到过，并且最近仍在更新。
     * 对只需要前馈的模块，可使用 JY61P_AccelIsOnline()。
     */
    return (g_accel_frames > 0U) && (g_angle_frames > 0U) &&
           (g_gyro_frames > 0U) &&
           ((g_tick_5ms - g_last_accel_tick) < ONLINE_TIMEOUT_TICKS) &&
           ((g_tick_5ms - g_last_angle_tick) < ONLINE_TIMEOUT_TICKS) &&
           ((g_tick_5ms - g_last_gyro_tick) < ONLINE_TIMEOUT_TICKS);
}

bool JY61P_AccelIsOnline(void)
{
    return (g_accel_frames > 0U) &&
           ((g_tick_5ms - g_last_accel_tick) < ONLINE_TIMEOUT_TICKS);
}

uint32_t JY61P_GetFrameCount(void)
{
    return g_accel_frames + g_angle_frames + g_gyro_frames;
}
uint32_t JY61P_GetAccelFrameCount(void) { return g_accel_frames; }
uint32_t JY61P_GetAngleFrameCount(void) { return g_angle_frames; }
uint32_t JY61P_GetGyroFrameCount(void) { return g_gyro_frames; }
