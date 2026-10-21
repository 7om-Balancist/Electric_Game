#ifndef BALL_STEPPER_CONTROL_H
#define BALL_STEPPER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 小球控制器对外接口。
 *
 * 推荐调用关系：
 *   1. 完成SysConfig、Vision、JY61P、ZDT初始化；
 *   2. 调用BallControl_Init()初始化控制器软件状态；
 *   3. 每个5ms定时节拍后，在主循环而不是ISR中调用Update5ms()；
 *   4. 用户确认机构静止且OLED显示ARMED后，调用Start()；
 *   5. Stop()可主动停止，FAULT则必须排除原因后调用ResetFault()。
 *
 * 故障采用锁存方式：传感器恢复并不会自动重新运行，防止设备在无人确认时
 * 突然恢复动作。
 *
 * 学习重点：
 * 这个模块是“上层控制器”，它不直接读 UART 寄存器，也不直接操作 GPIO。
 * 它只消费 Vision/JY61P/ZDT 三个模块提供的抽象接口，然后输出目标位置。
 *
 * 关于“视觉丢失”的两个层次（重要）：
 *   链路还在收帧，但没有可用的新样本 -> BALL_FAULT_BALL_LOST（通常是丢球）
 *   链路本身不再来帧                   -> BALL_FAULT_VISION_LOST（相机/接线问题）
 * 两者必须分开判断。只看链路层会让“相机一直在发无效帧”这种故障被漏掉，
 * 控制律会被静默冻结，同时边缘保护和超速保护一起失效。
 */

typedef enum {
    /* 等待相机和JY61P链路。 */
    BALL_CTRL_WAIT_SENSORS = 0,
    /* 已发送ZDT使能帧，等待ACK。 */
    BALL_CTRL_ENABLING,
    /* 已发送编码器位置查询，等待捕获ARM软件原点。 */
    BALL_CTRL_READ_ORIGIN,
    /* 原点和通信已准备好，等待用户启动。 */
    BALL_CTRL_ARMED,
    /* 视觉PD、IMU前馈和ZDT目标调度正在运行。 */
    BALL_CTRL_RUNNING,
    /* 已发送STOP，等待应答。 */
    BALL_CTRL_STOPPING,
    /* 锁存故障，执行STOP和DISABLE。 */
    BALL_CTRL_FAULT
} BallControlState_t;

typedef enum {
    BALL_FAULT_NONE = 0,
    /* 传输层：相机链路超过规定时间没有新的帧到达。 */
    BALL_FAULT_VISION_LOST,
    /* JY61P 0x51加速度帧超时。 */
    BALL_FAULT_IMU_LOST,
    /* ZDT返回NACK或连续通信超时。 */
    BALL_FAULT_ZDT,
    /* 启动阶段未能及时读取编码器原点。 */
    BALL_FAULT_ORIGIN_TIMEOUT,
    /* 小球已经进入图像边缘危险区。 */
    BALL_FAULT_BALL_EDGE,
    /* 连续视觉帧确认小球速度过高。 */
    BALL_FAULT_BALL_OVERSPEED,
    /* ZDT编码器位置反馈长期没有刷新。 */
    BALL_FAULT_MOTOR_FEEDBACK,
    /*
     * 链路在收帧，但一直没有可用的新样本（flags 无效或序号不更新）。
     * 相机端认为球丢了；控制律不能继续用冻结的旧位置驱动电机。
     * 新增项一律追加在末尾，避免改变已有故障码的数值（OLED 会显示 F+数字）。
     */
    BALL_FAULT_BALL_LOST
} BallControlFault_t;

void BallControl_Init(uint32_t now_ms);
void BallControl_Update5ms(uint32_t now_ms);
bool BallControl_Start(uint32_t now_ms);
void BallControl_Stop(uint32_t now_ms);
void BallControl_ResetFault(uint32_t now_ms);

BallControlState_t BallControl_GetState(void);
BallControlFault_t BallControl_GetFault(void);
const char *BallControl_GetStateName(void);
const char *BallControl_GetProfileName(void);
int16_t BallControl_GetBallX(void);
float BallControl_GetBallVelocity(void);
float BallControl_GetBallAcceleration(void);
float BallControl_GetImuAcceleration(void);
float BallControl_GetFeedbackDeg(void);
float BallControl_GetFeedforwardDeg(void);
float BallControl_GetCommandDeg(void);
int32_t BallControl_GetCommandPulse(void);
bool BallControl_HasOrigin(void);

/*
 * 诊断：控制器实际消费掉的新样本数，以及距上一次成功消费的毫秒数。
 * 与 Vision_GetStats() 的 payload_ok_frames 对比即可判断单槽邮箱有没有
 * 把帧合并掉（合并过多说明控制节拍跟不上相机帧率）。
 */
uint32_t BallControl_GetConsumedFrames(void);
uint32_t BallControl_GetSampleAgeMs(void);

#endif
