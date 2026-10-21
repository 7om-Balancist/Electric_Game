/*
 * vision_link.h - MaixCAM 到 MSPM0 的位置数据链路。
 *
 * 注意：本模块不做视觉识别，只负责接收、校验并保存相机发来的结构化数据。
 * 帧格式为 12 字节：A5 5A + sequence + flags + x + camera_time +
 * confidence + reserved + CRC16。
 *
 * 学习重点：
 * 控制器只依赖这个模块输出的 VisionSample_t，不关心相机端如何识别小球。
 * 本模块对外提供两级"在线"判据，名字必须区分清楚：
 *   Vision_IsOnline()        - 传输层：最近有 CRC 正确的帧到达。
 *   Vision_PayloadIsOnline() - 载荷层：最近有 flags 有效的帧到达。
 * 控制律要用载荷层。只查传输层会被"相机一直在发但一直没看到球"骗过去。
 */

#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_FRAME_SIZE       12U
#define VISION_FRAME_HEADER_0   0xA5U
#define VISION_FRAME_HEADER_1   0x5AU
#define VISION_FLAG_VALID       0x01U

typedef struct {
    /* 小球横向像素坐标，通常 0~319，中心约为 160。 */
    int16_t x_px;
    /* 相机端时间戳，用于诊断链路延迟；控制器主要使用 received_ms。 */
    uint16_t camera_time_ms;
    /* 相机每帧递增的序号，用于判断是否为新样本。 */
    uint8_t sequence;
    uint8_t flags;
    uint8_t confidence;
    /*
     * 本机时间戳，在 DMA 完成中断里打点，单位毫秒。
     * 它表示"这个数据块到达本机"的时刻，而不是"被主循环取走解析"的时刻。
     */
    uint32_t received_ms;
} VisionSample_t;

/*
 * 链路统计。
 *
 * 这些计数器的用途是把"链路健康"从感觉变成数字：
 *   valid_frames        - CRC 通过的帧（含内容不可用的帧）
 *   payload_ok_frames   - CRC 通过且 flags 有效（相机确实看到球）
 *   invalid_flag_frames - CRC 通过但 flags 无效（通常是丢球/遮挡）
 *   duplicate_frames    - 载荷有效但 sequence 与上一帧相同（相机端卡住）
 *   crc_errors          - CRC 失败的帧
 *   dropped_blocks      - DMA 块被覆盖而丢弃（上一块还没被前台取走）
 *
 * 判断"控制器实际吃到了多少帧"，要拿 payload_ok_frames 和控制器自己的
 * consumed_frames 对比：
 *   payload_ok 约等于 consumed      -> 控制律跑在相机帧率上
 *   payload_ok 约为 2 倍 consumed   -> 一半的帧被单槽邮箱合并（更新率不足）
 */
typedef struct {
    uint32_t valid_frames;
    uint32_t payload_ok_frames;
    uint32_t invalid_flag_frames;
    uint32_t duplicate_frames;
    uint32_t crc_errors;
    uint32_t dropped_blocks;
} VisionStats_t;

void Vision_Init(void);
void Vision_UART_IRQHandler(void);
/* 主循环前台调用：取出 pending 块并逐字节解析。时间戳由模块自己在中断里打。 */
void Vision_Poll(void);
bool Vision_GetLatest(VisionSample_t *sample);
/* 传输层在线：最近有 CRC 正确的帧。 */
bool Vision_IsOnline(uint32_t timeout_ms);
/* 载荷层在线：最近有 flags 有效的帧（相机确实看到了球）。 */
bool Vision_PayloadIsOnline(uint32_t timeout_ms);
void Vision_GetStats(VisionStats_t *stats);

#endif
