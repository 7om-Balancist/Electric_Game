/**
 * motor.c — TB6612 双电机驱动 + 编码器 + PID (MSPM0 DriverLib)
 *
 * 学习重点：
 * 1. 直流电机控制被拆成“方向 GPIO + PWM 占空比”两部分。
 * 2. 编码器使用外部中断计数，A 相触发、读取 B 相判断方向。
 * 3. 距离计算不直接用总脉冲，而是先记录零点，再计算相对位移。
 * 4. PID_t 是通用控制器结构，当前巡线主要使用 PD 差速，保留 PID
 *    方便以后扩展为左右轮速度闭环。
 */

#include "motor.h"

/* 编码器脉冲计数 (ISR 中累加) */
volatile int32_t g_enc_left  = 0;
volatile int32_t g_enc_right = 0;

static int32_t g_enc_left_zero;
static int32_t g_enc_right_zero;

/* ================================================================
 *  电机初始化
 * ================================================================ */
void Motor_Init(void)
{
    /* 上电先停车，防止 PWM/GPIO 初始化过程中的不确定电平让电机误动。 */
    Motor_Stop();
    Encoder_ResetCounts();
}

/* ================================================================
 *  TB6612 方向控制 (带缓存, 方向不变不重写 GPIO)
 * ================================================================ */

static int8_t g_last_left_dir  = -1;   /* -1=未知, 0=停, 1=正, 2=反 */
static int8_t g_last_right_dir = -1;

static void Motor_LeftDir(bool in1, bool in2)
{
    int8_t st = in1 ? (in2 ? 0 : 1) : (in2 ? 2 : 0);
    /*
     * 方向缓存：如果方向没有变化，就不重复写 GPIO。
     * 这能减少控制周期内的无意义寄存器访问，也降低毛刺风险。
     */
    if (st == g_last_left_dir) return;
    g_last_left_dir = st;

    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
}

static void Motor_RightDir(bool in1, bool in2)
{
    int8_t st = in1 ? (in2 ? 0 : 1) : (in2 ? 2 : 0);
    if (st == g_last_right_dir) return;
    g_last_right_dir = st;

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);

    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
}

/* ================================================================
 *  电机调速
 * ================================================================ */

void Motor_SetLeftSpeed(int16_t speed)
{
    uint16_t duty;
    /*
     * speed 的符号表示方向，绝对值表示 PWM 占空比。
     * 上层只需要传入 -4000~4000，不需要关心 TB6612 的 IN1/IN2 真值表。
     */
    if (speed > 0) {
        Motor_LeftDir(true, false);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        Motor_LeftDir(false, true);
        duty = (uint16_t)(-speed);
    } else {
        Motor_LeftDir(false, false);
        duty = 0;
    }
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        duty, DL_TIMER_CC_0_INDEX);
}

void Motor_SetRightSpeed(int16_t speed)
{
    uint16_t duty;
    if (speed > 0) {
        Motor_RightDir(true, false);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        Motor_RightDir(false, true);
        duty = (uint16_t)(-speed);
    } else {
        Motor_RightDir(false, false);
        duty = 0;
    }
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        duty, DL_TIMER_CC_1_INDEX);
}

void Motor_Stop(void)
{
    /* 滑行停车：方向脚都置为无驱动状态，PWM 清零。 */
    Motor_LeftDir(false, false);
    Motor_RightDir(false, false);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
}

void Motor_Brake(void)
{
    /*
     * 短刹/电刹：两个方向输入同时有效，并给最大 PWM。
     * 该模式用于终点停车阶段，比简单 Stop 更快消耗动能。
     */
    Motor_LeftDir(true, true);
    Motor_RightDir(true, true);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        MOTOR_PWM_MAX, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        MOTOR_PWM_MAX, DL_TIMER_CC_1_INDEX);
}

/* ================================================================
 *  编码器读取
 * ================================================================ */

int32_t Encoder_GetLeftCount(void)  { return g_enc_left; }
int32_t Encoder_GetRightCount(void) { return g_enc_right; }

void Encoder_ResetCounts(void)
{
    g_enc_left  = 0;
    g_enc_right = 0;
    Encoder_ResetDistance();
}

void Encoder_ResetDistance(void)
{
    g_enc_left_zero = g_enc_left;
    g_enc_right_zero = g_enc_right;
}

static float Encoder_CountToCm(int32_t count)
{
    /*
     * 编码器脉冲 -> 轮子转数 -> 距离。
     * wheel_circumference_cm 需要用真实轮胎周长标定；轮胎打滑也会引入误差。
     */
    const float wheel_circumference_cm = 21.05f;
    return ((float)count * wheel_circumference_cm) / (float)PULSES_PER_REV;
}

float Encoder_GetLeftDistanceCm(void)
{
    return Encoder_CountToCm(g_enc_left - g_enc_left_zero);
}

float Encoder_GetRightDistanceCm(void)
{
    return Encoder_CountToCm(g_enc_right - g_enc_right_zero);
}

float Encoder_GetAverageDistanceCm(void)
{
    float left = Encoder_GetLeftDistanceCm();
    float right = Encoder_GetRightDistanceCm();

    if (left < 0.0f) left = -left;
    if (right < 0.0f) right = -right;
    return (left + right) * 0.5f;
}

/**
 * @brief 编码器 GPIO 中断处理 (GPIOA)
 *
 * 在 GPIOA 中断服务中调用:
 *   void GROUP1_IRQHandler(void) { Encoder_ISR(); }
 *
 * 编码器 A 相上升沿触发, 读 B 相电平判方向:
 *   B=高 → 正转 (++),  B=低 → 反转 (--)
 */
void Encoder_ISR(void)
{
    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(ENCODER_PORT);
    uint16_t pin;

    /* 编码器 A (左电机): PA27=A相, PA26=B相 */
    if (iidx == ENCODER_ENC_A1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_A1_IIDX);
        /*
         * A 相上升沿触发时读取 B 相：
         * B 相高低代表 A/B 相位关系，从而判断正转或反转。
         */
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_A2_PIN))
            g_enc_left++;
        else
            g_enc_left--;
    }

    /* 编码器 B (右电机): PA14=A相, PA25=B相 */
    else if (iidx == ENCODER_ENC_B1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_B1_IIDX);
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_B2_PIN))
            g_enc_right++;
        else
            g_enc_right--;
    }
}

/* ================================================================
 *  PID 控制器
 * ================================================================ */

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, int16_t out_limit)
{
    /* integral_limit 取输出限幅的一半，防止积分项长期累积后让输出饱和。 */
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit  = (float)out_limit * 0.5f;
    pid->output_limit    = out_limit;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

int16_t PID_Compute(PID_t *pid, int16_t setpoint, int16_t measurement, float dt)
{
    /*
     * 标准位置式 PID：
     * error = 目标值 - 测量值
     * output = Kp*error + Ki*∫error + Kd*d(error)/dt
     */
    float error  = (float)(setpoint - measurement);
    float output = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    output += pid->Ki * pid->integral;

    /* dt 太小时不计算 D 项，避免除以极小数导致输出异常放大。 */
    if (dt > 0.001f)
        output += pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return (int16_t)output;
}
