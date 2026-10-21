#include "vision_link.h"

#include "ti_msp_dl_config.h"
#include "timebase.h"

#include <string.h>

/*
 * vision_link.c - UART + DMA + CRC 的视觉数据接收层。
 *
 * 学习重点：
 * 1. UART RX 使用 DMA 每次搬运固定 12 字节，把中断频率从 720/s 降到 60/s。
 * 2. DMA 完成中断只做三件事：切换缓冲、把数据块搬进 pending 槽、打到达时间戳。
 *    真正的解析放在 Vision_Poll() 前台执行。
 * 3. 解析器按字节寻找 A5 5A 帧头，即使中途丢字节也能重新同步。
 * 4. CRC16/CCITT-FALSE 校验保证控制器只使用可信的位置数据。
 *
 * 关于两级在线判据的差别（这是本项目一个真实缺陷的来源）：
 *   Vision_IsOnline()        只看 received_ms，即"最近收到过 CRC 正确的帧"。
 *   Vision_PayloadIsOnline() 看"最近收到过 flags 有效的帧"。
 * 相机丢球时会持续发送 CRC 正确但 flags=0 的帧，此时传输层一直在线，
 * 而控制律一帧都不会被消费。所以安全停机判据必须建立在载荷层以及
 * 控制器自己的消费时刻（g_last_sample_ms）之上，不能只看传输层。
 *
 * 另外注意：本文件不关心"哪些帧被控制律消费了"。消费级的新鲜度由
 * ball_stepper_control.c 自己维护，因为只有它知道令牌门（sequence 去重）。
 */

static uint8_t g_dma_buffer[2][VISION_FRAME_SIZE];
static volatile uint8_t g_dma_active;
static volatile uint8_t g_pending_data[VISION_FRAME_SIZE];
static volatile bool g_pending;
static volatile uint32_t g_pending_ms;

static uint8_t g_parser_frame[VISION_FRAME_SIZE];
static uint8_t g_parser_index;

static VisionSample_t g_latest;
static bool g_latest_valid;

/* 载荷层时间戳：只有 flags 有效时才刷新。 */
static uint32_t g_payload_ms;
static bool g_payload_seen;
/* 用于统计"相机端是否卡在同一个 sequence 上"。 */
static uint8_t g_last_sequence;
static bool g_have_last_sequence;

static volatile VisionStats_t g_stats;

static uint16_t crc16_ccitt(const uint8_t *data, uint8_t length)
{
    /*
     * CRC-16/CCITT-FALSE:
     * poly=0x1021, init=0xFFFF, no reflection, xorout=0。
     * 相机端和 MCU 端必须使用完全相同参数，否则所有帧都会校验失败。
     */
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void arm_dma(uint8_t buffer_index)
{
    /*
     * 重新装填 DMA：源地址是 UART RXDATA，目的地址是双缓冲之一。
     * 双缓冲减少"上一帧还没处理完，下一帧已经到来"的覆盖风险。
     *
     * 隐含假设：重新装填窗口（约 1us）内到达的字节由 UART 的 RX FIFO 接住。
     * 115200 下一个字节要 87us，余量约两个数量级，因此这个假设是安全的。
     */
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                      (uint32_t)&UART_MaixCAM_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
                       (uint32_t)&g_dma_buffer[buffer_index][0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, VISION_FRAME_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}

/* 记录一帧"内容可用"的样本，并区分它是不是重复序号。 */
static void accept_payload(uint8_t sequence, uint32_t now_ms)
{
    if (g_have_last_sequence && (sequence == g_last_sequence)) {
        g_stats.duplicate_frames++;
    } else {
        g_stats.payload_ok_frames++;
        g_last_sequence = sequence;
        g_have_last_sequence = true;
    }
    g_payload_ms = now_ms;
    g_payload_seen = true;
}

static void consume_byte(uint8_t byte, uint32_t now_ms)
{
    /* 帧头同步状态机：先找 A5，再找 5A，之后收满固定长度。 */
    if (g_parser_index == 0U) {
        if (byte == VISION_FRAME_HEADER_0) {
            g_parser_frame[g_parser_index++] = byte;
        }
        return;
    }

    if (g_parser_index == 1U) {
        if (byte == VISION_FRAME_HEADER_1) {
            g_parser_frame[g_parser_index++] = byte;
        } else if (byte != VISION_FRAME_HEADER_0) {
            g_parser_index = 0U;
        }
        /*
         * 若 byte == A5，则保持在 index==1：这样 "A5 A5 5A" 也能正确同步。
         * 这是字节流解析器最容易写错的一处细节。
         */
        return;
    }

    g_parser_frame[g_parser_index++] = byte;
    if (g_parser_index < VISION_FRAME_SIZE) {
        return;
    }

    uint16_t received_crc = (uint16_t)g_parser_frame[10] |
                            ((uint16_t)g_parser_frame[11] << 8);
    uint16_t calculated_crc = crc16_ccitt(g_parser_frame, 10U);

    if (received_crc == calculated_crc) {
        uint8_t sequence = g_parser_frame[2];
        uint8_t flags = g_parser_frame[3];

        /* CRC 正确才更新 latest，控制器永远读取最后一帧有效样本。 */
        g_latest.sequence = sequence;
        g_latest.flags = flags;
        g_latest.x_px = (int16_t)((uint16_t)g_parser_frame[4] |
                                  ((uint16_t)g_parser_frame[5] << 8));
        g_latest.camera_time_ms = (uint16_t)g_parser_frame[6] |
                                  ((uint16_t)g_parser_frame[7] << 8);
        g_latest.confidence = g_parser_frame[8];
        g_latest.received_ms = now_ms;
        g_latest_valid = true;
        g_stats.valid_frames++;

        if ((flags & VISION_FLAG_VALID) != 0U) {
            accept_payload(sequence, now_ms);
        } else {
            /* 相机在发帧，但这一帧没有可用位置（丢球、遮挡、目标丢失）。 */
            g_stats.invalid_flag_frames++;
        }
    } else {
        g_stats.crc_errors++;
    }

    g_parser_index = 0U;
}

void Vision_Init(void)
{
    memset(g_dma_buffer, 0, sizeof(g_dma_buffer));
    memset(g_parser_frame, 0, sizeof(g_parser_frame));
    memset(&g_latest, 0, sizeof(g_latest));
    memset((void *)&g_stats, 0, sizeof(g_stats));

    g_dma_active = 0U;
    g_pending = false;
    g_pending_ms = 0U;
    g_parser_index = 0U;
    g_latest_valid = false;
    g_payload_ms = 0U;
    g_payload_seen = false;
    g_last_sequence = 0U;
    g_have_last_sequence = false;

    arm_dma(g_dma_active);
    NVIC_ClearPendingIRQ(UART_MaixCAM_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MaixCAM_INST_INT_IRQN);
}

void Vision_UART_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_MaixCAM_INST)) {
    case DL_UART_MAIN_IIDX_DMA_DONE_RX: {
        /*
         * DMA 完成说明收到了一个 12 字节块。
         * 顺序很重要：先切换并重装 DMA，再拷贝刚填满的那一半，
         * 这样拷贝期间新到的数据不会破坏被拷贝的缓冲。
         */
        uint8_t completed = g_dma_active;
        g_dma_active ^= 1U;
        arm_dma(g_dma_active);

        if (g_pending) {
            /*
             * 上一块还没被前台取走。这里选择丢弃旧块，而不是等待或排队：
             * 相机数据是状态量，过时的位置没有价值。丢块只影响统计，
             * 不会造成解析器失步（解析器是字节流状态机，靠帧头重同步）。
             */
            g_stats.dropped_blocks++;
        }
        for (uint8_t i = 0U; i < VISION_FRAME_SIZE; i++) {
            g_pending_data[i] = g_dma_buffer[completed][i];
        }
        /*
         * 在中断里打"到达时刻"。这样时间戳描述的是数据真正到达本机的时刻，
         * 而不是主循环恰好取走它的时刻。用统一时基，控制器可以直接比较。
         */
        g_pending_ms = Timebase_NowMs();
        g_pending = true;
        break;
    }
    default:
        break;
    }
}

void Vision_Poll(void)
{
    uint8_t local[VISION_FRAME_SIZE];
    uint32_t local_ms = 0U;
    bool have_block = false;

    /*
     * pending_data 由中断写、主循环读，复制时关中断形成一个很短的临界区。
     * 复制完成后再逐字节解析，解析过程不需要关中断。
     */
    __disable_irq();
    if (g_pending) {
        for (uint8_t i = 0U; i < VISION_FRAME_SIZE; i++) {
            local[i] = g_pending_data[i];
        }
        local_ms = g_pending_ms;
        g_pending = false;
        have_block = true;
    }
    __enable_irq();

    if (!have_block) {
        return;
    }

    for (uint8_t i = 0U; i < VISION_FRAME_SIZE; i++) {
        consume_byte(local[i], local_ms);
    }
}

bool Vision_GetLatest(VisionSample_t *sample)
{
    if ((sample == NULL) || !g_latest_valid) {
        return false;
    }
    *sample = g_latest;
    return true;
}

bool Vision_IsOnline(uint32_t timeout_ms)
{
    /*
     * 传输层判据：最近收到过 CRC 正确的帧。
     * 它不能用来判断"控制律还有没有可用输入"，见文件头的说明。
     */
    return g_latest_valid &&
           ((uint32_t)(Timebase_NowMs() - g_latest.received_ms) <= timeout_ms);
}

bool Vision_PayloadIsOnline(uint32_t timeout_ms)
{
    return g_payload_seen &&
           ((uint32_t)(Timebase_NowMs() - g_payload_ms) <= timeout_ms);
}

void Vision_GetStats(VisionStats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    /*
     * 这些是诊断计数器，每个字段都是 32 位对齐自增，读取本身是原子的。
     * 前后两次读之间可能有新帧进来，因此拿到的是一份略微不同步的快照，
     * 这对显示和诊断完全够用，不必为它关中断。
     */
    *stats = g_stats;
}
