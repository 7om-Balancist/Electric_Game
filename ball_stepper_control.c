#include "ball_stepper_control.h"

#include "jy61p.h"
#include "timebase.h"
#include "vision_link.h"
#include "zdt_stepper.h"

/*
 * ==========================================================================
 *  小球位置控制器：MaixCAM v6.12.37 Q5 直道快速控制算法的 MSPM0 移植版
 * ==========================================================================
 *
 * 学习重点：
 * 本文件展示了一个较完整的嵌入式闭环控制模块：状态机负责生命周期，
 * 视觉/IMU 负责测量，ZDT 负责执行，故障锁存负责安全边界。
 *
 * 一、模块的输入和输出
 * --------------------------------------------------------------------------
 * 输入 1：MaixCAM 通过 UART + DMA 发送的小球横向像素坐标 x。
 *         图像宽度为 320 px，标定中心 O 点为 x=160 px。
 *
 * 输入 2：JY61P 的 0x51 加速度帧。本模块默认读取 AccX，并在按下 KEY1
 *         时记录静止零偏，之后把“当前加速度 - 静止零偏”作为车体纵向加速度。
 *
 * 输出：  相对于 ARM 软件原点的电机目标角度，经 3200 pulse/rev 换算后，
 *         加到 ZDT 编码器启动位置上，最终通过 raF=1 绝对位置帧发送给 ZDT。
 *
 * 二、完整数据通路
 * --------------------------------------------------------------------------
 *   相机位置 x
 *       -> 快速速度（一帧差分）+ 慢速速度（约 91 ms 窗口）
 *       -> 位置/速度 PD + 二次边缘项 + 静摩擦补偿
 *
 *   JY61P 纵向加速度
 *       -> 静止零偏扣除 -> 低通滤波 -> 限幅/方向保护
 *       -> 加速度前馈角
 *
 *   视觉反馈角 + IMU 前馈角
 *       -> +/-13 deg 最终机械限位
 *       -> 电机方向符号修正
 *       -> ZDT 绝对位置脉冲
 *
 * 三、核心控制公式
 * --------------------------------------------------------------------------
 * 位置误差：
 *     error = target_x - ball_x
 *
 * 视觉反馈（尚未加入静摩擦地板和最终限幅）：
 *     feedback = Kp * p_error - Kd * ball_velocity + edge_rescue
 *
 * IMU 加速度前馈：
 *     accel = imu_axis - stationary_zero
 *     ff = ff_sign * ff_gain * accel
 *
 * 最终命令：
 *     motor_deg = direction_sign * clamp(feedback + ff, -13, +13)
 *
 * 注意：MaixCAM Q5 原算法是“位置 P + 速度 D”，积分 I 明确为 0。
 * 这里没有为了凑成传统 PID 而额外加入积分，否则在摩擦、延迟和饱和存在时
 * 容易产生积分累积，导致小球越过中心后仍保持较大倾角。
 *
 * 四、采样与调度原则
 * --------------------------------------------------------------------------
 * 1. 每个新的相机 sequence 只计算一次视觉 PD，重复帧不会重复计算。
 * 2. 每个新的 JY61P 0x51 帧只更新一次前馈，旧加速度不会重复滤波。
 * 3. 5 ms 主循环只负责调度；ZDT 正在等待 ACK 时，新目标覆盖旧目标，形成
 *    “最新目标邮箱”，不会排队执行已经过时的位置命令。
 * 4. ARM 时读取 ZDT 编码器当前位置作为软件原点，不发送 SetZero，因此不会
 *    改写驱动器内部保存的机械零点。
 * 5. 视觉链路上有三个互相独立的时刻，安全判据必须用对的那一个：
 *    a. 帧到达       -> vision_link 的 g_latest.received_ms（传输层，CRC 通过就刷新）
 *    b. 帧被消费     -> 本文件的 g_last_sample_ms（令牌通过 + flags 有效才刷新）
 *    c. 命令下发     -> 本文件的 g_last_command_ms（Zdt_MoveAbsolute 成功才刷新）
 *    RUNNING 的视觉超时必须用 b，不能用 a；用 a 会在相机持续发无效帧时
 *    永远不触发故障。IMU 通道用的是"新帧计数 + g_last_accel_ms"，与 b 等价，
 *    所以它本来就没有这个问题。
 * 6. 投递口有角度速率限制（CTRL_COMMAND_SLEW_DEG_PER_S），只用于拦单帧跳变。
 *
 * 五、三个容易混淆的“加速度”
 * --------------------------------------------------------------------------
 * g_ball_acceleration：由相机位置二次差分得到，单位 px/s^2，目前仅用于诊断。
 * g_imu_accel_filtered：JY61P 测得的车体纵向加速度，单位 m/s^2，用于前馈。
 * CTRL_ZDT_ACCELERATION：ZDT 位置指令中的速度曲线参数，不是物理 m/s^2。
 */

/* ========================= 1. 坐标与机械边界 ============================= */
/* 相机中心；若重新标定 O 点，只修改该值，不要用电机零点补偿像素误差。 */
#define CTRL_TARGET_X_PX                 160.0f
/* 控制量到电机方向的映射。若小球越控越远，应优先检查该符号和机械接线。 */
#define CTRL_DIRECTION_SIGN              (-1.0f)
/* 相对 ARM 软件原点的最终机械限位。任何反馈、前馈都不能越过此限制。 */
#define CTRL_HARD_LIMIT_DEG               13.0f
#define CTRL_PULSES_PER_REV               3200.0f

/* ========================= 2. 视觉 PD 参数 =============================== */
/*
 * HOLD：中心附近的小增益区，降低抖动。
 * CAPTURE：偏离中心后的大增益区，提高拉回能力。
 * 5~11 px 之间使用线性插值，避免 Kp/Kd 突然跳变。
 */
#define CTRL_HOLD_KP                      0.100f
#define CTRL_HOLD_KD                      0.055f
#define CTRL_CAPTURE_KP                   0.165f
#define CTRL_CAPTURE_KD                   0.070f
#define CTRL_HOLD_ENTRY_PX                5.0f
#define CTRL_HOLD_EXIT_PX                 8.0f
#define CTRL_CAPTURE_FULL_PX              11.0f
/* 最终速度 = 0.65*快速速度 + 0.35*慢速速度。 */
#define CTRL_FAST_VELOCITY_WEIGHT         0.65f
#define CTRL_VELOCITY_CLIP_PX_S           240.0f
#define CTRL_SLOW_WINDOW_MS               140U
#define CTRL_SLOW_EMA_ALPHA               0.30f
#define CTRL_SLOW_DEADBAND_PX_S           6.0f

/* ========================= 3. 制动、预测与边缘救援 ====================== */
/*
 * error*velocity < 0 表示小球正在远离目标，此时增大 D 和边缘救援项。
 * 小球正在返回中心且速度较大时，用 0.20 s 预测误差提前撤掉旧方向 P 项，
 * 让 D 项先进行反向制动，减少穿过中心后的过冲。
 */
#define CTRL_OUTWARD_SPEED_PX_S           4.0f
#define CTRL_OUTWARD_D_MULTIPLIER          1.80f
#define CTRL_EDGE_START_PX                 6.0f
#define CTRL_EDGE_QUADRATIC                0.035f
#define CTRL_OUTWARD_EDGE_MULTIPLIER       1.50f
#define CTRL_TOWARD_TAPER_START_PX_S       8.0f
#define CTRL_TOWARD_EDGE_ZERO_PX_S         22.0f
#define CTRL_PREDICT_DELAY_S               0.20f

/* ========================= 4. 静摩擦补偿 ================================ */
/*
 * 当误差已经存在，但朝目标方向的速度仍很低时，给控制量一个最小角度地板。
 * 左右值不同是为了补偿连杆、重心和摩擦不对称；它不是一次性“猛推”。
 */
#define CTRL_CREEP_ENTRY_PX                1.5f
#define CTRL_CREEP_RELEASE_PX_S            14.0f
#define CTRL_CREEP_LEFT_DEG                1.20f
#define CTRL_CREEP_RIGHT_DEG               1.50f

/* ========================= 5. JY61P 加速度前馈 ========================== */
/*
 * 常规前馈增益为 2 deg/(m/s^2)，最大只允许贡献 3 deg。
 * 小球离中心越远，视觉反馈应占主导，因此前馈在 3~12 px 间逐渐衰减。
 * 若前馈与视觉拉回方向冲突，且误差已经大于 4 px，则直接屏蔽该次前馈。
 */
#define CTRL_FF_GAIN_DEG_PER_MPS2          2.0f
#define CTRL_FF_SIGN                       (-1.0f)
#define CTRL_FF_MAX_DEG                    3.0f
#define CTRL_FF_ALPHA                      0.35f
#define CTRL_FF_SLEW_DEG                   0.80f
#define CTRL_FF_FULL_ERROR_PX              3.0f
#define CTRL_FF_ZERO_ERROR_PX              12.0f
#define CTRL_FF_GUARD_ERROR_PX             4.0f
#define CTRL_FF_TIMEOUT_MS                 220U
#define CTRL_FF_WARMUP_SAMPLES             3U

/* ========================= 6. 启动阶段前馈增强 ========================== */
/*
 * KEY1 后先等待实际加速度超过 0.15 m/s^2，再启动 1.5 s 的增强窗口。
 * 这样补偿窗口对齐真实车辆起步，而不是对齐按键时刻。窗口内前馈更快、更强，
 * 同时延后边缘项和静摩擦项，避免起步瞬间多个补偿一起满幅叠加。
 */
#define CTRL_STARTUP_WAIT_MS               5000U
#define CTRL_STARTUP_TRIGGER_MPS2          0.15f
#define CTRL_STARTUP_WINDOW_MS             1500U
#define CTRL_STARTUP_RAMP_MS               900U
#define CTRL_STARTUP_P_SCALE               0.85f
#define CTRL_STARTUP_D_SCALE               1.10f
#define CTRL_STARTUP_EDGE_DELAY_MS         250U
#define CTRL_STARTUP_CREEP_DELAY_MS        650U
#define CTRL_STARTUP_EMERGENCY_ERROR_PX    7.0f
#define CTRL_STARTUP_FF_GAIN_MULTIPLIER    1.45f
#define CTRL_STARTUP_FF_MAX_DEG            3.20f
#define CTRL_STARTUP_FF_ALPHA              0.78f
#define CTRL_STARTUP_FF_SLEW_DEG           0.75f
#define CTRL_STARTUP_ACCEL_DEADBAND_MPS2   0.08f
#define CTRL_STARTUP_FF_FULL_ERROR_PX      6.0f
#define CTRL_STARTUP_FF_ZERO_ERROR_PX      16.0f

/* ========================= 7. IMU 轴向与滤波 ============================ */
/* 0=X、1=Y、2=Z；必须根据 JY61P 实际安装方向选择车辆纵向轴。 */
#define CTRL_IMU_ACCEL_AXIS                0U /* 0=X, 1=Y, 2=Z */
/* 若车辆正向加速时数据显示为负，可改为 -1.0f。 */
#define CTRL_IMU_ACCEL_SIGN                1.0f
/* 一阶低通：new = old + alpha*(measurement-old)，越小越平滑但延迟越大。 */
#define CTRL_IMU_ACCEL_ALPHA               0.25f

/* ========================= 8. UART 调度与安全超时 ====================== */
/*
 * ZDT 的 15 ms 是最短发命令间隔；编码器查询为低优先级，只占空闲 UART 槽。
 * 视觉超过 100 ms、IMU 前馈超过 220 ms、位置反馈超过 1200 ms 均会停机。
 *
 * 注意"视觉 100 ms"是两级判据，缺一不可（见 Update5ms 的 RUNNING 分支）：
 *   1) 链路层：Vision_IsOnline()，最近收到过 CRC 正确的帧；
 *   2) 消费层：g_last_sample_ms，最近真正消费过一帧"新且有效"的样本。
 * 只查第 1 条时，相机持续发送 flags 无效的帧会让故障判据失效：链路看起来
 * 一直在线，但 PD 一帧都不再重算，控制量被冻结，而边缘保护和超速保护都
 * 只在 compute_feedback() 里调用，于是同时失效。
 */

/*
 * 投递口角度速率限制（deg/s）。
 *
 * 视觉侧没有像前馈那样的 slew 限制，一次误识别可能让目标角出现接近满幅
 * （-13 到 +13）的阶跃。这里给出与执行器能力同量级的速率上限：
 * ZDT 的 CTRL_ZDT_RPM=80 约等于 480 deg/s，把它设成 900 deg/s 意味着
 * 正常工况下这道限制永远不会生效（15 ms 内允许约 13.5 deg 变化，
 * 而满量程只有 26 deg），只在"相邻两条命令方向接近相反且间隔很短"
 * 这种明显的跳变场景下才起作用。这样既拦住毛刺，又不改变已调好的行为。
 */
#define CTRL_COMMAND_SLEW_DEG_PER_S        900.0f

#define CTRL_COMMAND_PERIOD_MS             15U
#define CTRL_QUERY_PERIOD_MS               200U
#define CTRL_QUERY_AFTER_COMMAND_MS        10U
#define CTRL_VISION_TIMEOUT_MS             100U
#define CTRL_ORIGIN_TIMEOUT_MS             1600U
#define CTRL_POSITION_LOST_MS              1200U
#define CTRL_ZDT_RPM                       80U
#define CTRL_ZDT_ACCELERATION              24U
#define CTRL_EDGE_STOP_PX                  12.0f
#define CTRL_OVERSPEED_PX_S                420.0f
#define CTRL_HISTORY_SIZE                  32U

typedef struct {
    /* 一个历史样本：MSPM0接收时刻和对应的小球像素坐标。 */
    uint32_t ms;
    float x;
} PositionHistory_t;

/* ========================= 9. 控制器内部状态 ============================ */
/* 状态机、故障及ZDT软件原点/最后命令。 */
static BallControlState_t g_state;
static BallControlFault_t g_fault;
static uint32_t g_state_since_ms;
static bool g_origin_valid;
static int32_t g_origin_pulse;
static int32_t g_command_pulse;
static float g_command_deg;
static uint32_t g_last_command_ms;
/* 是否已经成功下发过至少一条位置命令（用于投递口速率限制的第一次放行）。 */
static bool g_have_command;
static uint32_t g_last_query_ms;
static bool g_target_dirty;

/* 视觉测量及其派生量，速度单位 px/s，加速度单位 px/s^2。 */
static int16_t g_ball_x;
static float g_error_px;
static float g_fast_velocity;
static float g_slow_velocity;
static float g_velocity_used;
static float g_ball_acceleration;
static float g_previous_fast_velocity;
static uint32_t g_previous_velocity_ms;
static uint8_t g_last_vision_sequence;
static bool g_have_vision_sequence;
/*
 * 消费级时间戳：只有"令牌通过且 flags 有效"的样本被真正消费时才刷新。
 * 它表达的是"控制律最后一次拿到可用输入"的时刻，安全停机判据用它。
 * 不要和 Vision 模块的 received_ms（传输层到达时刻）混为一谈。
 */
static uint32_t g_last_sample_ms;
static uint32_t g_consumed_frames;
static PositionHistory_t g_history[CTRL_HISTORY_SIZE];
static uint8_t g_history_count;

/* 控制量分解，全部使用“电机轴角度 deg”作为统一单位。 */
static float g_feedback_deg;
static float g_p_term_deg;
static float g_d_term_deg;
static float g_edge_term_deg;
static float g_feedforward_deg;
static float g_imu_accel_zero;
static float g_imu_accel_filtered;
static uint32_t g_last_accel_frame_count;
static uint32_t g_last_accel_ms;
static uint8_t g_ff_warmup_hits;

/* 非线性控制区、启动补偿和故障停机的辅助状态。 */
static bool g_hold_zone;
static bool g_moving_outward;
static bool g_creep_active;
static bool g_startup_waiting;
static bool g_startup_active;
static uint32_t g_startup_wait_ms;
static uint32_t g_startup_trigger_ms;
static uint8_t g_overspeed_hits;
static const char *g_profile;
static bool g_shutdown_sent;
static bool g_disable_sent;

/* 以下三个小函数避免依赖完整数学库，也使符号/限幅意图更直观。 */
static float absf_local(float value) { return value < 0.0f ? -value : value; }

static float signf_local(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static float clampf_local(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int32_t deg_to_pulse(float degree)
{
    /*
     * ZDT命令脉冲换算：pulse = degree / 360 * 3200。
     * 正负数分别加/减0.5再转整数，得到对称的四舍五入，避免负数长期偏零。
     */
    float pulse = degree * CTRL_PULSES_PER_REV / 360.0f;
    return (int32_t)(pulse + ((pulse >= 0.0f) ? 0.5f : -0.5f));
}

static float selected_acceleration(void)
{
    /* 编译期选择JY61P纵向轴，不在实时循环里做坐标矩阵运算。 */
    if (CTRL_IMU_ACCEL_AXIS == 1U) return JY61P_GetAccelY();
    if (CTRL_IMU_ACCEL_AXIS == 2U) return JY61P_GetAccelZ();
    return JY61P_GetAccelX();
}

static void enter_state(BallControlState_t state, uint32_t now_ms)
{
    /* 所有状态切换都同时记录进入时刻，供超时和启动渐变使用。 */
    g_state = state;
    g_state_since_ms = now_ms;
}

static void enter_fault(BallControlFault_t fault, uint32_t now_ms)
{
    /*
     * 故障是锁存的：第一次故障原因最有诊断价值，后续连锁错误不覆盖它。
     * 真正的STOP/DISABLE由5ms状态机分两步执行，避免在检测函数中阻塞UART。
     */
    if (g_state == BALL_CTRL_FAULT) return;
    g_fault = fault;
    g_shutdown_sent = false;
    g_disable_sent = false;
    g_profile = "FAULT";
    enter_state(BALL_CTRL_FAULT, now_ms);
}

static void history_reset(void)
{
    g_history_count = 0U;
}

static void history_push(float x, uint32_t now_ms)
{
    /*
     * 保存短时间位置历史，用于计算抗单帧反光跳变的慢速速度。
     * 缓冲区满时移除最旧样本；同时删除500ms以前的无关数据。
     * 32个样本在60Hz下约覆盖533ms，足够容纳当前140ms窗口。
     */
    if (g_history_count >= CTRL_HISTORY_SIZE) {
        for (uint8_t i = 1U; i < CTRL_HISTORY_SIZE; i++) {
            g_history[i - 1U] = g_history[i];
        }
        g_history_count = CTRL_HISTORY_SIZE - 1U;
    }
    g_history[g_history_count].ms = now_ms;
    g_history[g_history_count].x = x;
    g_history_count++;

    while ((g_history_count > 2U) &&
           ((now_ms - g_history[0].ms) > 500U)) {
        for (uint8_t i = 1U; i < g_history_count; i++) {
            g_history[i - 1U] = g_history[i];
        }
        g_history_count--;
    }
}

static void update_velocity(float x, uint32_t sample_ms)
{
    /*
     * 快速速度在process_visual_sample()中由相邻两帧计算，响应最快但噪声大。
     * 本函数计算慢速速度：寻找约91ms前的锚点做长窗口差分，再经过EMA。
     *
     * 为什么不是简单平均两种速度：
     * - 快速通道让控制器尽早看到小球开始逃逸或开始返回；
     * - 慢速通道抑制识别框在反光、边缘处发生的一帧跳动；
     * - 最终控制速度由compute_feedback()按65%/35%融合。
     */
    if (!g_have_vision_sequence) return;

    history_push(x, sample_ms);
    uint32_t target_age = (CTRL_SLOW_WINDOW_MS * 65U) / 100U;
    int anchor_index = -1;
    for (int i = (int)g_history_count - 2; i >= 0; i--) {
        if ((sample_ms - g_history[i].ms) >= target_age) {
            anchor_index = i;
            break;
        }
    }

    /* 找不到足够老的锚点时，临时用快速速度作为慢速通道输入。 */
    float raw_slow = g_fast_velocity;
    if (anchor_index >= 0) {
        uint32_t dt_ms = sample_ms - g_history[anchor_index].ms;
        if (dt_ms >= 40U) {
            raw_slow = (x - g_history[anchor_index].x) * 1000.0f /
                       (float)dt_ms;
        }
    }
    g_slow_velocity += CTRL_SLOW_EMA_ALPHA * (raw_slow - g_slow_velocity);
    if (absf_local(g_slow_velocity) < CTRL_SLOW_DEADBAND_PX_S) {
        g_slow_velocity = 0.0f;
    }

    if (g_previous_velocity_ms > 0U) {
        /*
         * 这是“小球图像加速度”，单位px/s^2，仅保留作诊断和未来扩展。
         * 它不是JY61P车体加速度，也不参与当前Q5前馈。
         */
        uint32_t dt_ms = sample_ms - g_previous_velocity_ms;
        if ((dt_ms >= 8U) && (dt_ms <= 250U)) {
            float raw_a = (g_fast_velocity - g_previous_fast_velocity) *
                          1000.0f / (float)dt_ms;
            raw_a = clampf_local(raw_a, -1800.0f, 1800.0f);
            g_ball_acceleration += 0.20f * (raw_a - g_ball_acceleration);
        }
    }
    g_previous_fast_velocity = g_fast_velocity;
    g_previous_velocity_ms = sample_ms;
}

static bool motion_is_safe(float x, float velocity, uint32_t now_ms)
{
    /*
     * 两层视觉安全保护：
     * 1. 小球进入画面左右12px危险区，立即锁存BALL_EDGE故障；
     * 2. 连续两帧速度超过420px/s，锁存BALL_OVERSPEED故障。
     * 连续确认可避免单帧识别跳点直接停机。
     */
    if ((x <= CTRL_EDGE_STOP_PX) || (x >= (319.0f - CTRL_EDGE_STOP_PX))) {
        enter_fault(BALL_FAULT_BALL_EDGE, now_ms);
        return false;
    }
    if (absf_local(velocity) >= CTRL_OVERSPEED_PX_S) {
        if (g_overspeed_hits < 255U) g_overspeed_hits++;
    } else {
        g_overspeed_hits = 0U;
    }
    if (g_overspeed_hits >= 2U) {
        enter_fault(BALL_FAULT_BALL_OVERSPEED, now_ms);
        return false;
    }
    return true;
}

static float startup_mix(uint32_t now_ms)
{
    /*
     * 启动增强不是突然切回常规参数，而是在900ms内线性混合。
     * 返回0表示刚触发，返回1表示已经完全恢复常规控制器。
     */
    if (!g_startup_active) return 1.0f;
    uint32_t age = now_ms - g_startup_trigger_ms;
    if (age >= CTRL_STARTUP_WINDOW_MS) {
        g_startup_active = false;
        return 1.0f;
    }
    return clampf_local((float)age / (float)CTRL_STARTUP_RAMP_MS, 0.0f, 1.0f);
}

static void compute_feedback(uint32_t now_ms)
{
    /*
     * 对“一帧新的相机位置”执行一次完整视觉控制计算。
     * 本函数只生成g_feedback_deg，不直接操作UART；发送由dispatch_target()统一调度。
     */
    float error = CTRL_TARGET_X_PX - (float)g_ball_x;
    float distance = absf_local(error);
    float velocity = (1.0f - CTRL_FAST_VELOCITY_WEIGHT) * g_slow_velocity +
                     CTRL_FAST_VELOCITY_WEIGHT * g_fast_velocity;
    velocity = clampf_local(velocity, -CTRL_VELOCITY_CLIP_PX_S,
                            CTRL_VELOCITY_CLIP_PX_S);
    g_velocity_used = velocity;
    g_error_px = error;

    if (!motion_is_safe((float)g_ball_x, velocity, now_ms)) return;

    /*
     * HOLD/CAPTURE使用5px进入、8px退出的迟滞区。
     * 没有迟滞时，小球在边界附近抖动会导致Kp/Kd每帧来回切换。
     */
    if (g_hold_zone) {
        if (distance >= CTRL_HOLD_EXIT_PX) g_hold_zone = false;
    } else if (distance <= CTRL_HOLD_ENTRY_PX) {
        g_hold_zone = true;
    }

    float blend;
    if (distance <= CTRL_HOLD_ENTRY_PX) blend = 0.0f;
    else if (distance >= CTRL_CAPTURE_FULL_PX) blend = 1.0f;
    else blend = (distance - CTRL_HOLD_ENTRY_PX) /
                 (CTRL_CAPTURE_FULL_PX - CTRL_HOLD_ENTRY_PX);

    /* 在5~11px之间连续插值，11px以外使用完整CAPTURE增益。 */
    float kp = CTRL_HOLD_KP + blend * (CTRL_CAPTURE_KP - CTRL_HOLD_KP);
    float kd = CTRL_HOLD_KD + blend * (CTRL_CAPTURE_KD - CTRL_HOLD_KD);
    float mix = startup_mix(now_ms);
    uint32_t startup_age = g_startup_active ?
        (now_ms - g_startup_trigger_ms) : UINT32_MAX;
    if (g_startup_active) {
        /* 起步早期略减P、增强D，让前馈先抵消车体加速，避免视觉项过早猛拉。 */
        kp *= CTRL_STARTUP_P_SCALE + mix * (1.0f - CTRL_STARTUP_P_SCALE);
        kd *= CTRL_STARTUP_D_SCALE + mix * (1.0f - CTRL_STARTUP_D_SCALE);
    }

    /*
     * error与velocity反号，表示速度方向在扩大误差，即小球正在远离O点。
     * 此时D增益乘1.8，优先把速度刹住，而不是只依赖位置误差慢慢增大。
     */
    g_moving_outward = (error * velocity < 0.0f) &&
                       (absf_local(velocity) >= CTRL_OUTWARD_SPEED_PX_S);
    if (g_moving_outward) kd *= CTRL_OUTWARD_D_MULTIPLIER;

    float toward_speed = signf_local(error) * velocity;
    bool taper = !g_moving_outward &&
                 (distance > CTRL_HOLD_ENTRY_PX) &&
                 (toward_speed >= CTRL_TOWARD_TAPER_START_PX_S);
    float p_error = error;
    /*
     * 小球已朝O点快速返回时预测0.2s后的误差。
     * 如果预测位置已经越过O点，则把P项临时降为0，只保留D制动；
     * 否则P项使用当前误差和预测误差中较小者，逐步撤掉旧方向驱动力。
     */
    if (taper) {
        float predicted_error = error - velocity * CTRL_PREDICT_DELAY_S;
        if (predicted_error * error <= 0.0f) p_error = 0.0f;
        else p_error = signf_local(error) *
                       ((distance < absf_local(predicted_error)) ?
                        distance : absf_local(predicted_error));
    }

    g_p_term_deg = kp * p_error;
    g_d_term_deg = -kd * velocity;
    float edge_excess = distance - CTRL_EDGE_START_PX;
    if (edge_excess < 0.0f) edge_excess = 0.0f;
    /*
     * 超出6px后加入二次项：edge = sign(error)*0.035*(distance-6)^2。
     * 二次项在中心附近为0，误差继续扩大时快速增强，专门用于边缘救援。
     */
    g_edge_term_deg = signf_local(error) * CTRL_EDGE_QUADRATIC *
                      edge_excess * edge_excess;
    if (g_moving_outward) {
        g_edge_term_deg *= CTRL_OUTWARD_EDGE_MULTIPLIER;
    } else if (taper) {
        float scale = (CTRL_TOWARD_EDGE_ZERO_PX_S - toward_speed) /
                      (CTRL_TOWARD_EDGE_ZERO_PX_S -
                       CTRL_TOWARD_TAPER_START_PX_S);
        g_edge_term_deg *= clampf_local(scale, 0.0f, 1.0f);
    }

    if (g_startup_active) {
        /*
         * 起步后前250ms关闭普通边缘项，随后渐进恢复；若小球已经明显向外逃逸，
         * 至少立即开放80%边缘能力。这样兼顾起步平顺和紧急救援。
         */
        float edge_scale = 0.0f;
        if (startup_age > CTRL_STARTUP_EDGE_DELAY_MS) {
            edge_scale = (float)(startup_age - CTRL_STARTUP_EDGE_DELAY_MS) /
                         (float)(CTRL_STARTUP_RAMP_MS -
                                 CTRL_STARTUP_EDGE_DELAY_MS);
        }
        edge_scale = clampf_local(edge_scale, 0.0f, 1.0f);
        if (g_moving_outward && distance >= CTRL_STARTUP_EMERGENCY_ERROR_PX &&
            edge_scale < 0.80f) {
            edge_scale = 0.80f;
        }
        g_edge_term_deg *= edge_scale;
    }

    g_feedback_deg = g_p_term_deg + g_d_term_deg + g_edge_term_deg;

    /*
     * 连续静摩擦地板：只有“有误差但朝目标速度不足”时才生效。
     * 它只保证控制量不低于克服摩擦所需角度，不会额外叠加一个脉冲，
     * 所以小球一旦产生足够速度，补偿会自然退出。
     */
    g_creep_active = false;
    bool creep_allowed = !g_startup_active ||
                         (startup_age >= CTRL_STARTUP_CREEP_DELAY_MS);
    float desired = signf_local(error);
    if (creep_allowed && desired != 0.0f &&
        distance >= CTRL_CREEP_ENTRY_PX &&
        (desired * velocity) < CTRL_CREEP_RELEASE_PX_S) {
        float floor_deg = desired < 0.0f ? CTRL_CREEP_LEFT_DEG
                                         : CTRL_CREEP_RIGHT_DEG;
        if ((g_feedback_deg + g_feedforward_deg) * desired < floor_deg) {
            g_creep_active = true;
        }
    }

    if (g_moving_outward && distance >= CTRL_EDGE_START_PX) g_profile = "EDGE";
    else if (g_moving_outward) g_profile = "BRAKE";
    else if (taper) g_profile = "TAPER";
    else if (g_creep_active) g_profile = "FRICTION";
    else if (g_hold_zone) g_profile = "HOLD_PD";
    else g_profile = "CAPTURE_PD";
    g_target_dirty = true;
}

static void process_visual_sample(uint32_t now_ms)
{
    /*
     * 从Vision模块的“最新样本邮箱”中取数据。
     * sequence相同表示仍是上一帧；直接返回可确保一次相机测量只进入一次PD。
     */
    VisionSample_t sample;
    if (!Vision_GetLatest(&sample) ||
        ((sample.flags & VISION_FLAG_VALID) == 0U)) return;
    /* 只消费新sequence，忽略5ms主循环中重复看到的同一视觉帧。 */
    if (g_have_vision_sequence &&
        (sample.sequence == g_last_vision_sequence)) return;

    /*
     * 走到这里才算"控制律拿到了一个新且可用的样本"。
     * 刷新消费级时间戳，它是视觉丢失判据的唯一依据，因此必须放在令牌门之后：
     * 放在前面（或者用 vision_link 的 received_ms）会让重复帧、无效帧
     * 也把"新鲜度"顶住，故障判据就再也触发不了。
     */
    g_last_sample_ms = now_ms;
    g_consumed_frames++;

    float old_x = (float)g_ball_x;
    uint32_t old_ms = g_history_count > 0U ?
        g_history[g_history_count - 1U].ms : 0U;
    g_ball_x = sample.x_px;
    if (g_have_vision_sequence && old_ms > 0U) {
        /* 快速速度 = 相邻帧位置差 / 实际接收时间差，不能假定永远精确60Hz。 */
        uint32_t dt_ms = sample.received_ms - old_ms;
        if ((dt_ms >= 8U) && (dt_ms <= 100U)) {
            g_fast_velocity = ((float)sample.x_px - old_x) * 1000.0f /
                              (float)dt_ms;
        }
    } else {
        g_fast_velocity = 0.0f;
        history_reset();
    }
    g_have_vision_sequence = true;
    g_last_vision_sequence = sample.sequence;
    update_velocity((float)sample.x_px, sample.received_ms);

    if (g_state == BALL_CTRL_RUNNING) compute_feedback(now_ms);
}

static void process_acceleration(uint32_t now_ms)
{
    /*
     * 使用JY61P有效0x51帧计数判断是否真的来了新加速度。
     * 这样即使5ms循环运行多次，也不会对同一个测量重复执行低通滤波。
     */
    uint32_t count = JY61P_GetAccelFrameCount();
    if (count == g_last_accel_frame_count) return;
    g_last_accel_frame_count = count;
    g_last_accel_ms = now_ms;

    /*
     * KEY1时记录的静止值包含传感器零偏和该安装姿态下的重力投影。
     * 当前值减去它后，才近似得到车辆纵向动态加速度。
     */
    float accel = CTRL_IMU_ACCEL_SIGN *
                  (selected_acceleration() - g_imu_accel_zero);
    g_imu_accel_filtered += CTRL_IMU_ACCEL_ALPHA *
                            (accel - g_imu_accel_filtered);
    if (g_ff_warmup_hits < 255U) g_ff_warmup_hits++;

    /*
     * 用真实加速度触发启动窗口，而不是按键触发。
     * 如果5秒内车辆没有起步，则放弃启动增强，但常规PD仍可继续工作。
     */
    if (g_startup_waiting) {
        if (absf_local(g_imu_accel_filtered) >= CTRL_STARTUP_TRIGGER_MPS2) {
            g_startup_waiting = false;
            g_startup_active = true;
            g_startup_trigger_ms = now_ms;
        } else if ((now_ms - g_startup_wait_ms) > CTRL_STARTUP_WAIT_MS) {
            g_startup_waiting = false;
        }
    }

    if (g_state != BALL_CTRL_RUNNING ||
        g_ff_warmup_hits < CTRL_FF_WARMUP_SAMPLES) return;

    float distance = absf_local(g_error_px);
    float accel_for_ff = g_imu_accel_filtered;
    float gain = CTRL_FF_GAIN_DEG_PER_MPS2;
    float ff_max = CTRL_FF_MAX_DEG;
    float full_error = CTRL_FF_FULL_ERROR_PX;
    float zero_error = CTRL_FF_ZERO_ERROR_PX;
    float alpha = CTRL_FF_ALPHA;
    float slew = CTRL_FF_SLEW_DEG;

    if (g_startup_active) {
        /*
         * 启动窗口：前馈增益提升45%，死区去除零点噪声，最大前馈放宽到3.2deg，
         * 并采用更快滤波系数0.78，使车辆刚起步、视觉尚未出现位移时就能响应。
         */
        gain *= CTRL_STARTUP_FF_GAIN_MULTIPLIER;
        if (absf_local(accel_for_ff) <= CTRL_STARTUP_ACCEL_DEADBAND_MPS2) {
            accel_for_ff = 0.0f;
        } else {
            accel_for_ff -= signf_local(accel_for_ff) *
                            CTRL_STARTUP_ACCEL_DEADBAND_MPS2;
        }
        ff_max = CTRL_STARTUP_FF_MAX_DEG;
        full_error = CTRL_STARTUP_FF_FULL_ERROR_PX;
        zero_error = CTRL_STARTUP_FF_ZERO_ERROR_PX;
        alpha = CTRL_STARTUP_FF_ALPHA;
        slew = CTRL_STARTUP_FF_SLEW_DEG;
    }

    float target = CTRL_FF_SIGN * gain * accel_for_ff;
    target = clampf_local(target, -ff_max, ff_max);
    float weight;
    /*
     * 前馈权限随小球误差衰减：中心附近全开，远离中心后归零。
     * 原因是前馈只负责提前抵消车体扰动，不能压过已经明确的小球位置纠偏。
     */
    if (distance <= full_error) weight = 1.0f;
    else if (distance >= zero_error) weight = 0.0f;
    else weight = (zero_error - distance) / (zero_error - full_error);
    target *= weight;

    /* 误差较大时，若前馈与视觉反馈反向，则视觉安全优先，屏蔽本次前馈。 */
    if (distance >= CTRL_FF_GUARD_ERROR_PX && g_feedback_deg != 0.0f &&
        target * g_feedback_deg < 0.0f) {
        target = 0.0f;
    }

    float next = g_feedforward_deg + alpha * (target - g_feedforward_deg);
    /* 先一阶低通，再限制每次前馈角变化量，抑制IMU尖峰造成的电机突跳。 */
    next = clampf_local(next, g_feedforward_deg - slew,
                        g_feedforward_deg + slew);
    g_feedforward_deg = next;
    g_target_dirty = true;
}

static void dispatch_target(uint32_t now_ms)
{
    /*
     * 将最新控制结果发给ZDT。三个条件任一不满足就暂缓：
     * - 没有新目标；
     * - 上一条ZDT命令仍在等待ACK；
     * - 距离上一条位置命令不足15ms。
     */
    if (!g_target_dirty || Zdt_IsCommandPending() ||
        ((now_ms - g_last_command_ms) < CTRL_COMMAND_PERIOD_MS)) return;

    /* ACK等待期间不排队旧命令，g_target_dirty始终代表当前最新目标。 */
    float combined = g_feedback_deg + g_feedforward_deg;
    float desired = signf_local(g_error_px);
    if (g_creep_active) {
        float floor_deg = desired < 0.0f ? CTRL_CREEP_LEFT_DEG
                                         : CTRL_CREEP_RIGHT_DEG;
        if (combined * desired < floor_deg) combined = desired * floor_deg;
    }
    combined = clampf_local(combined, -CTRL_HARD_LIMIT_DEG,
                            CTRL_HARD_LIMIT_DEG);
    /*
     * motor_deg是相对ARM位置；ZDT需要驱动器绝对坐标：
     * absolute_pulse = arm_origin_pulse + motor_deg/360*3200。
     */
    float motor_deg = CTRL_DIRECTION_SIGN * combined;

    /*
     * 投递口速率限制：把"控制算法算出的目标"和"执行机构跟得上的目标"分开。
     * 按实际经过的时间计算允许的最大变化量；第一次下发不做限制。
     * 这一步只拦单帧误识别造成的阶跃，正常工况下不会生效（见宏定义处说明）。
     */
    if (g_have_command) {
        uint32_t elapsed_ms = now_ms - g_last_command_ms;
        float max_step = CTRL_COMMAND_SLEW_DEG_PER_S * (float)elapsed_ms * 0.001f;
        float delta = motor_deg - g_command_deg;
        if (delta > max_step) {
            motor_deg = g_command_deg + max_step;
        } else if (delta < -max_step) {
            motor_deg = g_command_deg - max_step;
        }
    }

    int32_t absolute_pulse = g_origin_pulse + deg_to_pulse(motor_deg);

    if (absolute_pulse == g_command_pulse) {
        g_target_dirty = false;
        return;
    }
    if (Zdt_MoveAbsolute(absolute_pulse, CTRL_ZDT_RPM,
                         CTRL_ZDT_ACCELERATION, now_ms)) {
        g_command_deg = motor_deg;
        g_command_pulse = absolute_pulse;
        g_last_command_ms = now_ms;
        g_have_command = true;
        g_target_dirty = false;
    }
}

void BallControl_Init(uint32_t now_ms)
{
    /*
     * 只初始化软件变量，不在这里让电机运动。
     * 上电后的第一状态是WAIT_SENSORS；必须先收到相机位置和JY61P加速度，
     * 状态机才会使能ZDT并读取编码器原点。
     */
    g_state = BALL_CTRL_WAIT_SENSORS;
    g_fault = BALL_FAULT_NONE;
    g_state_since_ms = now_ms;
    g_origin_valid = false;
    g_origin_pulse = 0;
    g_command_pulse = 0;
    g_command_deg = 0.0f;
    g_last_command_ms = now_ms;
    g_last_query_ms = now_ms;
    g_target_dirty = false;
    g_have_command = false;
    g_ball_x = 160;
    g_error_px = 0.0f;
    g_fast_velocity = 0.0f;
    g_slow_velocity = 0.0f;
    g_velocity_used = 0.0f;
    g_ball_acceleration = 0.0f;
    g_previous_fast_velocity = 0.0f;
    g_previous_velocity_ms = 0U;
    g_last_vision_sequence = 0U;
    g_have_vision_sequence = false;
    g_last_sample_ms = now_ms;
    g_consumed_frames = 0U;
    history_reset();
    g_feedback_deg = 0.0f;
    g_p_term_deg = 0.0f;
    g_d_term_deg = 0.0f;
    g_edge_term_deg = 0.0f;
    g_feedforward_deg = 0.0f;
    g_imu_accel_zero = 0.0f;
    g_imu_accel_filtered = 0.0f;
    g_last_accel_frame_count = 0U;
    g_last_accel_ms = 0U;
    g_ff_warmup_hits = 0U;
    g_hold_zone = true;
    g_moving_outward = false;
    g_creep_active = false;
    g_startup_waiting = false;
    g_startup_active = false;
    g_startup_wait_ms = 0U;
    g_startup_trigger_ms = 0U;
    g_overspeed_hits = 0U;
    g_profile = "WAIT";
    g_shutdown_sent = false;
    g_disable_sent = false;
}

void BallControl_Update5ms(uint32_t now_ms)
{
    /*
     * 这是控制器唯一的周期服务入口，由main在5ms节拍后于前台调用。
     * 顺序不能随意交换：
     * 1. 先解析ZDT回复，释放pending命令；
     * 2. 再消费最新视觉和IMU样本，更新目标邮箱；
     * 3. 最后运行状态机，决定使能、发目标、查询位置或故障停机。
     *
     * 浮点计算和UART发送均不放进定时器ISR，避免长中断阻塞DMA/JY61P接收。
     */
    /*
     * 周期入口的三个步骤：
     * 1. 先解析 ZDT 回复，释放上一条命令的 pending 状态；
     * 2. 再消费视觉位置和 IMU 加速度，更新控制目标；
     * 3. 最后按状态机决定是否发新目标、查询位置或进入故障。
     */
    Zdt_Poll(now_ms);
    process_visual_sample(now_ms);
    process_acceleration(now_ms);

    if ((g_state != BALL_CTRL_WAIT_SENSORS) &&
        (g_state != BALL_CTRL_FAULT) &&
        (Zdt_GetState() == ZDT_STATE_FAULT)) {
        enter_fault(BALL_FAULT_ZDT, now_ms);
    }

    switch (g_state) {
    case BALL_CTRL_WAIT_SENSORS:
        /*
         * 等待通信链路：至少有一帧有效小球位置和一帧新鲜0x51加速度。
         * 两者都在线后只发送ZDT使能，不立即发任何位置目标。
         */
        if (JY61P_AccelIsOnline() &&
            Vision_IsOnline(CTRL_VISION_TIMEOUT_MS)) {
            if (Zdt_Enable(now_ms)) {
                g_profile = "ENABLE";
                enter_state(BALL_CTRL_ENABLING, now_ms);
            }
        }
        break;

    case BALL_CTRL_ENABLING:
        /* 等待ZDT对F3使能帧返回ACK，然后发送36位置查询帧。 */
        if ((Zdt_GetState() == ZDT_STATE_READY) && !Zdt_IsCommandPending()) {
            if (Zdt_RequestPosition(now_ms)) {
                g_profile = "ORIGIN";
                enter_state(BALL_CTRL_READ_ORIGIN, now_ms);
            }
        } else if ((now_ms - g_state_since_ms) > CTRL_ORIGIN_TIMEOUT_MS) {
            enter_fault(BALL_FAULT_ORIGIN_TIMEOUT, now_ms);
        }
        break;

    case BALL_CTRL_READ_ORIGIN:
        /*
         * 把ZDT编码器当前位置保存为软件原点。
         * 例如读到绝对坐标10000pulse，则相对+1deg命令会发10009pulse左右；
         * 驱动器内部零点从未被修改。
         */
        if (Zdt_HasPosition() && !Zdt_IsCommandPending()) {
            g_origin_pulse = Zdt_GetPositionPulse();
            g_command_pulse = g_origin_pulse;
            g_command_deg = 0.0f;
            g_origin_valid = true;
            g_profile = "ARMED";
            enter_state(BALL_CTRL_ARMED, now_ms);
        } else if ((now_ms - g_state_since_ms) > CTRL_ORIGIN_TIMEOUT_MS) {
            enter_fault(BALL_FAULT_ORIGIN_TIMEOUT, now_ms);
        }
        break;

    case BALL_CTRL_ARMED:
        /*
         * ARMED表示通信和原点已准备好，但控制器尚未运行。
         * 此时只低频查询编码器，不根据小球位置发运动命令；KEY1调用Start()。
         */
        if (!Zdt_IsCommandPending() &&
            ((now_ms - g_last_query_ms) >= CTRL_QUERY_PERIOD_MS)) {
            if (Zdt_RequestPosition(now_ms)) g_last_query_ms = now_ms;
        }
        break;

    case BALL_CTRL_RUNNING:
        /*
         * RUNNING阶段先检查传感器新鲜度，再发送最新目标。
         * 视觉分两级判断（原因见文件头）：
         *   1) 链路层：根本没有新帧       -> VISION_LOST（相机或接线问题）；
         *   2) 消费层：在收帧但没有可用的新样本 -> BALL_LOST（丢球/遮挡）。
         * 加速度220ms没有更新视为IMU链路故障；
         * 编码器查询超过1200ms未刷新则认为无法监督电机实际位置。
         */
        if (!Vision_IsOnline(CTRL_VISION_TIMEOUT_MS)) {
            enter_fault(BALL_FAULT_VISION_LOST, now_ms);
        } else if ((now_ms - g_last_sample_ms) > CTRL_VISION_TIMEOUT_MS) {
            /*
             * 修复点：原来这里只判断 Vision_IsOnline()，而它的时间戳在
             * "CRC 通过"时就刷新，与 flags/sequence 无关。相机丢球时会持续
             * 发送 flags=0 的帧，链路判据永远成立，于是 PD 被静默冻结：
             * 反馈角、误差、边缘保护、超速保护全部停在最后一次有效样本上，
             * 而 IMU 前馈还在推电机。现在改为判断"最后一次真正消费样本"的时刻，
             * 故障会被如实锁存并显示（OLED 上显示为 F8）。
             */
            enter_fault(BALL_FAULT_BALL_LOST, now_ms);
        } else if (!JY61P_AccelIsOnline() ||
                   ((now_ms - g_last_accel_ms) > CTRL_FF_TIMEOUT_MS)) {
            enter_fault(BALL_FAULT_IMU_LOST, now_ms);
        } else {
            dispatch_target(now_ms);
            /*
             * 位置查询优先级低于控制目标。只有目标已发送、ZDT空闲、且距离
             * 最近位置命令至少10ms时才插入查询，避免查询阻塞下一帧控制。
             */
            if (!g_target_dirty && !Zdt_IsCommandPending() &&
                ((now_ms - g_last_command_ms) >= CTRL_QUERY_AFTER_COMMAND_MS) &&
                ((now_ms - g_last_query_ms) >= CTRL_QUERY_PERIOD_MS)) {
                if (Zdt_RequestPosition(now_ms)) g_last_query_ms = now_ms;
            }
            if (Zdt_HasPosition() &&
                !Zdt_PositionIsFresh(now_ms, CTRL_POSITION_LOST_MS)) {
                enter_fault(BALL_FAULT_MOTOR_FEEDBACK, now_ms);
            }
        }
        break;

    case BALL_CTRL_STOPPING:
        /* 等待STOP ACK；超时250ms也退出等待，防止状态机永久卡住。 */
        if (!Zdt_IsCommandPending() ||
            ((now_ms - g_state_since_ms) > 250U)) {
            g_profile = "ARMED";
            enter_state(BALL_CTRL_ARMED, now_ms);
        }
        break;

    case BALL_CTRL_FAULT:
        /*
         * 故障停机分两步：先FE STOP停止运动，确认UART空闲后再F3 DISABLE。
         * 分步发送可避免紧接着的DISABLE覆盖仍在等待的STOP应答。
         */
        if (!g_shutdown_sent) {
            Zdt_Stop(now_ms);
            g_shutdown_sent = true;
        } else if (!g_disable_sent &&
                   ((now_ms - g_state_since_ms) > 50U) &&
                   !Zdt_IsCommandPending()) {
            if (Zdt_Disable(now_ms)) g_disable_sent = true;
        }
        break;

    default:
        enter_fault(BALL_FAULT_ZDT, now_ms);
        break;
    }
}

bool BallControl_Start(uint32_t now_ms)
{
    /*
     * 启动前置条件：状态必须为ARMED、软件原点有效、视觉和IMU均在线。
     * 视觉除了"链路在线"还要求"载荷在线"，也就是相机确实看到过球：
     * 否则一进 RUNNING 就会因为没有可用样本立刻锁存 BALL_LOST，
     * 与其让操作者看到一个莫名其妙的故障，不如在这里就拒绝启动。
     * 任一条件不满足就返回false，调用者可保持等待，不会让电机误动作。
     */
    if ((g_state != BALL_CTRL_ARMED) || !g_origin_valid ||
        !Vision_IsOnline(CTRL_VISION_TIMEOUT_MS) ||
        !Vision_PayloadIsOnline(CTRL_VISION_TIMEOUT_MS) ||
        !JY61P_AccelIsOnline()) return false;

    /*
     * 启动瞬间机构应静止：当前纵向加速度被记为零偏。
     * 如果在车辆已经加速或剧烈振动时按KEY1，零偏会错误，前馈方向/幅值也会错。
     */
    g_imu_accel_zero = selected_acceleration();
    g_imu_accel_filtered = 0.0f;
    g_last_accel_frame_count = JY61P_GetAccelFrameCount();
    g_last_accel_ms = now_ms;
    g_ff_warmup_hits = 0U;
    g_feedforward_deg = 0.0f;
    g_fast_velocity = 0.0f;
    g_slow_velocity = 0.0f;
    g_ball_acceleration = 0.0f;
    g_previous_velocity_ms = 0U;
    g_have_vision_sequence = false;
    /*
     * 令牌被清空后，下一帧一定会被当成新样本消费，所以这里把消费级时间戳
     * 对齐到当前时刻，给第一帧留出完整的超时窗口。
     * g_consumed_frames 是累计诊断量，不在这里清零。
     */
    g_last_sample_ms = now_ms;
    history_reset();
    g_hold_zone = true;
    g_overspeed_hits = 0U;
    g_startup_waiting = true;
    /*
     * 此处只是“武装”启动补偿，不立刻开始1.5s计时；
     * 真正超过0.15m/s^2后才由process_acceleration()触发。
     */
    g_startup_active = false;
    g_startup_wait_ms = now_ms;
    g_profile = "Q5_WAIT_ACCEL";
    g_target_dirty = true;
    g_last_command_ms = now_ms - CTRL_COMMAND_PERIOD_MS;
    enter_state(BALL_CTRL_RUNNING, now_ms);
    return true;
}

void BallControl_Stop(uint32_t now_ms)
{
    /* STOP会清空尚未发送的控制目标，并把反馈/前馈显示量归零。 */
    if ((g_state == BALL_CTRL_RUNNING) || (g_state == BALL_CTRL_ARMED)) {
        Zdt_Stop(now_ms);
        g_target_dirty = false;
        g_feedback_deg = 0.0f;
        g_feedforward_deg = 0.0f;
        g_profile = "STOP";
        enter_state(BALL_CTRL_STOPPING, now_ms);
    }
}

void BallControl_ResetFault(uint32_t now_ms)
{
    /*
     * 故障复位后重新回到WAIT_SENSORS，并重新执行使能和编码器原点捕获。
     * 不能直接从FAULT跳回RUNNING，这是为了避免沿用故障前的旧原点和旧目标。
     */
    if (g_state != BALL_CTRL_FAULT) return;
    g_fault = BALL_FAULT_NONE;
    g_origin_valid = false;
    g_have_vision_sequence = false;
    /* 复位后重新计时，避免带着故障前的旧时间戳再次触发同一个故障。 */
    g_last_sample_ms = now_ms;
    g_have_command = false;
    g_shutdown_sent = false;
    g_disable_sent = false;
    g_profile = "WAIT";
    enter_state(BALL_CTRL_WAIT_SENSORS, now_ms);
}

BallControlState_t BallControl_GetState(void) { return g_state; }
BallControlFault_t BallControl_GetFault(void) { return g_fault; }
const char *BallControl_GetProfileName(void) { return g_profile; }

const char *BallControl_GetStateName(void)
{
    static const char *const names[] = {
        "WAIT", "ENABLE", "ORIGIN", "ARMED", "RUN", "STOP", "FAULT"
    };
    return names[(uint8_t)g_state];
}

int16_t BallControl_GetBallX(void) { return g_ball_x; }
float BallControl_GetBallVelocity(void) { return g_velocity_used; }
float BallControl_GetBallAcceleration(void) { return g_ball_acceleration; }
float BallControl_GetImuAcceleration(void) { return g_imu_accel_filtered; }
float BallControl_GetFeedbackDeg(void) { return g_feedback_deg; }
float BallControl_GetFeedforwardDeg(void) { return g_feedforward_deg; }
float BallControl_GetCommandDeg(void) { return g_command_deg; }
int32_t BallControl_GetCommandPulse(void) { return g_command_pulse; }
bool BallControl_HasOrigin(void) { return g_origin_valid; }

uint32_t BallControl_GetConsumedFrames(void) { return g_consumed_frames; }

uint32_t BallControl_GetSampleAgeMs(void)
{
    return (uint32_t)(Timebase_NowMs() - g_last_sample_ms);
}
