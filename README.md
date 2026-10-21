# BUJIN — 视觉 + MSPM0 + ZDT 步进电机 小球位置闭环控制

H 题（钢球位置控制）的 MSPM0G3507 端固件。MaixCAM 负责看球，MSPM0 负责算，
张大头 ZDT 闭环步进驱动器负责执行：把钢球稳定在管道中心 O 点（x = 160 px）。

> 工程名沿用目录名 BUJIN。原 README 里的 "Liner_Car0" 是巡线小车模板的残留。

---

## 1. 一句话数据通路

```
MaixCAM ──12B帧/UART0(115200)+DMA──▶ vision_link ──CRC+令牌门──▶ 最新样本邮箱
                                                                    │
                                                            视觉 PD (位置P+速度D
                                                            +边缘二次项+静摩擦地板)
                                                                    │
JY61P ──0x51帧/UART1(9600)──▶ 加速度邮箱 ──新帧门控──▶ 加速度前馈 ───┤
                                                                    ▼
                                                      combined = 反馈 + 前馈
                                                                    │ ±13° 硬限位
                                                                    │ 投递口速率限制
                                                      绝对脉冲 = 原点 + deg/360*3200
                                                                    │ ZDT ACK 反压
                                                                    ▼
                                          ZDT Emm V5 ──UART2(115200)──▶ 步进驱动器
                                                                    │
                                              编码器位置查询(200ms 一次) ◀──┘
```

设计要点：**四段数据用了四种投递语义**，而不是一套机制硬套。

| 段 | 语义 | 丢弃策略 |
| --- | --- | --- |
| `pending` 块（DMA→前台） | 不能丢且必须完整 | 双缓冲 + 12 字节关中断拷贝 |
| 视觉样本（`g_latest`） | 可丢，但必须新 | 单槽覆盖，令牌 `sequence` 去重 |
| IMU 加速度 | 可丢，但必须新 | 单槽覆盖，帧计数去重 |
| 目标角（`g_feedback_deg + g_feedforward_deg`） | 可丢，必须新，且受执行器反压 | 覆盖 + `pending` 锁 + 紧急旁路 |
| 控制节拍（`g_control_ticks_pending`） | 只表示"该算几次" | 饱和计数，上限 4（20ms） |

**为什么全部选择覆盖而不是排队**：小球-管道是双积分对象，回路里任何排队都会
注入纯延迟。按 `Kp = 0.165 deg/px`、`Kd = 0.070 deg/(px/s)` 估算，闭环自然频率
约 5.7 rad/s；一帧延迟只贡献约 5.5° 相移，而 4 帧队列就有 22°、8 帧队列 44°。
对 D 项主导的过阻尼回路，44° 相移足以把它推向振荡。所以"丢旧取新"是设计选择，
不是偷懒。

---

## 2. 模块地图

### 在编译产物里（`Debug/BUJIN.map` 可核对）

| 文件 | 职责 |
| --- | --- |
| `main.c` | 初始化顺序、5ms 节拍、主循环调度、OLED、按键分发 |
| `timebase.c/.h` | 统一毫秒时基（唯一计数器，只在定时器中断里推进） |
| `vision_link.c/.h` | 相机 12 字节帧：DMA + 双缓冲 + 帧头重同步 + CRC16 + 两级在线判据 |
| `jy61p.c/.h` | JY61P/WIT 0x51/0x52/0x53 帧解析，加速度/角速度/角度 |
| `zdt_stepper.c/.h` | ZDT Emm V5 协议：使能/停止/绝对位置/位置查询 + ACK 与故障解析 |
| `ball_stepper_control.c/.h` | 状态机 + 视觉 PD + 前馈 + 安全保护 + 目标投递 |
| `key.c/.h` | 4 按键消抖与事件 |
| `oled.c/.h` | SSD1306 显存与软件 I2C 刷新 |
| `delay.c/.h` | 忙等延时（仅用于 I2C 时序与初始化） |

### 编译了但没有链接进映像（巡线小车的残留，属于死代码）

`motor.c`（只保留了被 `GROUP1_IRQHandler` 调用的 `Encoder_ISR`）、
`route_fsm.c`（整个模块被链接器剔除）、`bluetooth.c`（只保留了向量表引用的
`UART3_IRQHandler`）、`gray_sensor.c`、`ir_sensor.c`、`buzzer.c`、
`mpu6050.c`、`mpu6050_platform.c`、`soft_i2c.c`。

其中 `motor.c` 里的 `PID_Init/PID_Compute` **不属于本工程**：那是巡线小车的
轮速 PID。本工程的控制器是 PD + 前馈，没有积分项（见第 5 节）。

---

## 3. 硬件接线

| 链路 | MSPM0 外设 | 引脚 | 波特率 | 方向 |
| --- | --- | --- | --- | --- |
| MaixCAM 位置帧 | UART0（SysConfig 实例名 `UART_MaixCAM`） | TX PA28 / RX PA31 | 115200 | 收，DMA_CH0 12 字节 |
| JY61P | UART1 | TX PA17 / RX PA18 | 9600 | 收 |
| ZDT Emm V5 | UART2 | TX PA23 / RX PA24 | 115200 | 收发 |
| 按键 KEY1~KEY4 | GPIO | PB1 / PB10 / PB11 / PB14 | — | 输入，下降沿 |
| OLED | **I2C0（硬件）** | SDA PA0 / SCL PA1 | 400 kHz | 显示 |
| 调试 | SWD | PA19 / PA20 | — | — |

所有设备必须共地；UART 交叉接线（A 的 TX 接 B 的 RX）；确认全部为 3.3V 逻辑。

### ✅ 已解决：OLED 已从软件 I2C 迁到硬件 I2C0（PA0/PA1）——见第 13 节

`oled.c` 用**软件 I2C**，硬编码在 **PA28(SDA) / PA31(SCL)**（`IOMUX_PINCM3` /
`IOMUX_PINCM6`）；而 SysConfig 把 `UART_MaixCAM` 也分配在 **PA28(TX) / PA31(RX)**
（同样是 `PINCM3` / `PINCM6`）。两者是同一对物理引脚、同一组 IOMUX 寄存器。

后果：`OLED_WR_Byte()` 每个字节都会把 PA28/PA31 重配成 GPIO（`DL_GPIO_initDigitalInputFeatures`
/ `initDigitalOutput`），**UART0 会从这两个引脚上被摘掉**，相机链路在第一次 OLED
刷新之后就再也收不到数据。这两者不可能同时工作。

解决方向（任选其一，需要同时改代码和接线）：

1. **保留相机在 PA28/PA31**：把 `oled.c` 顶部的 4 个引脚宏改到一对空闲引脚上，
   并确认该对引脚在转接板上引出。当前 syscfg 里空闲的候选中继号：PA0~PA6、
   PA8~PA11、PA21/PA22、PB0、PB4/PB5、PB8/PB9、PB12/PB13、PB15~PB18、PB20~PB23。
2. **保留 OLED 在 PA28/PA31**：在 SysConfig 里把 `UART_MaixCAM` 换一对引脚
   （重新生成 `Debug/syscfg`）。
3. 比赛版本干脆不初始化 OLED，把 PA28/PA31 完整让给相机。

`oled.h` 里的 `#define OLED_INST I2C_BUS_INST` 是死宏：生成的配置里已经不存在
`I2C_BUS`（OLED 早已从硬件 I2C 改成软件 I2C），这个宏没有任何地方使用。

---

## 4. 三个时刻，三个时间戳

排障时最容易搞混的就是这三个，它们由不同的人刷新、表达不同的事：

| 时刻 | 变量 | 谁刷新 | 用途 |
| --- | --- | --- | --- |
| ① 帧到达 | `VisionSample_t.received_ms` | `vision_link.c` 的 DMA 完成中断 | 传输层在线判据 |
| ② 帧被消费 | `g_last_sample_ms`（控制器内） | 令牌门通过 + `flags` 有效时 | **安全停机判据**、速度差分 |
| ③ 命令下发 | `g_last_command_ms` / `g_command_pulse` | `Zdt_MoveAbsolute()` 成功返回 | 15ms 节流、重复目标去重 |

`Vision_Poll()` 里 `dt = received_ms - old_ms` 用的是①，所以速度估计是实测帧间隔，
不假定 60Hz；窗口限定在 8~100ms，挡掉首帧、合帧过度和异常时间戳。

**两级在线判据**（都由 `vision_link` 提供）：

- `Vision_IsOnline(timeout)` —— 传输层：最近有 CRC 正确的帧。
- `Vision_PayloadIsOnline(timeout)` —— 载荷层：最近有 `flags` 有效的帧。
- 控制器还有第三级：`g_last_sample_ms`，即"最近真正被消费的新样本"。

`RUNNING` 的安全停机用的是第三级。只查第一级会漏掉"相机持续发送无效帧"这种故障
（原因与修法见第 11 节第 1 条）。

---

## 5. 控制律

单位统一为电机轴角度（deg）。

```
error      = 160 - x                                    # px
velocity   = 0.35*慢速 + 0.65*快速                       # px/s，限幅 ±240
feedback   = Kp*p_error - Kd*velocity + edge_term       # deg
ff         = FF_SIGN * gain * (accel - 静止零偏)          # deg，限幅 ±3
motor_deg  = DIRECTION_SIGN * clamp(feedback + ff, -13, +13)
absolute   = origin_pulse + motor_deg/360*3200           # ZDT 绝对坐标
```

非线性部分（全部来自 MaixCAM v6.12.37 Q5 直道快速版）：

- **HOLD/CAPTURE 双增益区**：进 5px、出 8px 迟滞；5~11px 之间线性插值
  `Kp 0.100→0.165`、`Kd 0.055→0.070`。
- **向外运动增强**：`error*velocity < 0` 且 |v| ≥ 4px/s 时 `Kd×1.8`。
- **回归预测减速（taper）**：朝目标速度 ≥ 8px/s 时预测 0.2s 后误差；
  若已越过中心则把 P 项清零只留 D 制动，否则取当前/预测误差中较小者。
  速度到 22px/s 时边缘项衰减到 0。
- **边缘二次救援**：超出 6px 后 `sign(error)*0.035*(d-6)^2`，向外时再 ×1.5。
- **静摩擦地板**：有误差但朝目标速度 < 14px/s 时保证控制量不低于 1.2°/1.5°
  （左右不同，补偿机构不对称摩擦）。
- **启动窗口**：KEY1 后等真实加速度 ≥ 0.15 m/s² 才触发 1.5s 增强窗口
  （而不是对齐按键时刻），窗口内前馈增益 ×1.45、α=0.78、限幅 3.2°，
  同时前 250ms 关闭常规边缘项（若小球确实在向外逃且 ≥7px，至少开放 80%）。
- **前馈权限**：误差 3px 内全开，12px 归零；误差 ≥4px 且前馈与视觉反向时
  屏蔽本次前馈（视觉安全优先）。
- **投递口速率限制**：`CTRL_COMMAND_SLEW_DEG_PER_S = 900 deg/s`，
  按实际经过时间限幅，只拦单帧误识别造成的阶跃（15ms 内允许 13.5°，满量程 26°），
  正常工况下不生效。

**没有积分项**：Q5 原算法就是"位置 P + 速度 D"，积分明确为 0。回路里有 12~27ms
延迟、±13° 饱和和库仑摩擦，加 I 项几乎必然积分累积到饱和，球过中心后仍挂着大倾角。
稳态误差由 **IMU 加速度前馈** 和 **静摩擦地板** 处理，这两个机制没有累积风险。

---

## 6. 状态机与按键

```
上电 → WAIT_SENSORS ──IMU在线 且 视觉链路在线──▶ ENABLING ──F3使能──▶ 等ACK
                                                                    │
                    FAULT_ORIGIN_TIMEOUT ◀── 超时1600ms ────────────┤
                                                                    ▼
      ARMED ◀── 读到编码器位置(存为软件原点，不发 SetZero) ── READ_ORIGIN
        │
        │ KEY1 (Start)
        ▼
     RUNNING ── 15ms 一次绝对位置命令 + 200ms 一次编码器查询
        │ KEY4 → STOPPING ──等STOP ACK(或250ms兜底)──▶ ARMED
        │ 任一故障 → FAULT（锁存，需 KEY3 复位）
```

| 按键 | 功能 |
| --- | --- |
| KEY1 | 启动闭环控制。前置条件：状态为 ARMED + 原点有效 + 视觉链路与载荷均在线 + IMU 在线。**按下瞬间记录静止加速度零偏，所以必须等机构静止再按。** |
| KEY2 | JY61P 航向归零（只影响 yaw，不影响纵向加速度零偏） |
| KEY3 | 故障复位 → 回到 WAIT_SENSORS，重新使能并重新捕获原点 |
| KEY4 | 主动停止 |

---

## 7. 故障表（OLED 显示为 `F` + 故障码）

| 码 | 名称 | 含义 | 做法 |
| --- | --- | --- | --- |
| 0 | `NONE` | 正常 | — |
| 1 | `VISION_LOST` | 相机**链路**超过 100ms 没有帧 | 查接线、波特率、PA31，以及第 3 节的引脚冲突 |
| 2 | `IMU_LOST` | JY61P 加速度超过 220ms 没有新帧 | 查 9600 波特率与 0x51 输出 |
| 3 | `ZDT` | ZDT 返回 NACK，或连续 3 次 ACK 超时 | 查驱动器使能、波特率、共地 |
| 4 | `ORIGIN_TIMEOUT` | 1600ms 内没能读到编码器原点 | 查 36 查询帧的应答 |
| 5 | `BALL_EDGE` | 小球进入图像左右 12px 危险区 | 机械/增益问题 |
| 6 | `BALL_OVERSPEED` | 连续 2 帧速度超过 420px/s | 增益过大或管道振动 |
| 7 | `MOTOR_FEEDBACK` | 编码器位置反馈超过 1200ms 未刷新 | 查回复解析 |
| **8** | **`BALL_LOST`** | **链路在收帧，但 100ms 内没有可用的新样本**（`flags` 无效或序号不更新） | 相机没看到球：把球放回画面，KEY3 复位 |

故障是**锁存**的：第一次原因最有诊断价值，后续连锁错误不会覆盖它；传感器恢复也不会
自动重跑，必须 KEY3 确认。

---

## 8. 构建与烧录

用 CCS Theia / CCS 打开本目录，构建 `Debug` 配置：

```
D:\TI_MSPM0\CCS_Workspace\BUJIN\Debug\BUJIN.out
```

工程基于 MSPM0 SDK 的 empty 示例 + SysConfig。改动 `empty_mspm0g3507.syscfg`
后会自动重新生成 `Debug/syscfg/`。编译器为 `ti-cgt-armllvm_5.1.1.LTS`。

近期占用（`BUJIN.map`）：约 25 KB Flash + 4 KB RAM。

---

## 9. 台架调试流程（建议顺序）

### 第 0 步：让视觉链路先通

相机端发送程序还不存在（见第 11 节第 2 条），所以先用 PC 代替相机：

```bash
python tools/vision_frame_sender.py --port COM5 --mode fixed --x 160
```

PC 的 USB-TTL TX 接 MSPM0 的 **PA31**，共地。OLED 上 `CAM` 应该亮起，
控制器应离开 `WAIT` 进入 `ENABLE → ORIGIN → ARMED`。

### 第 1 步：验证故障判据（这是本次修复的重点）

```bash
python tools/vision_frame_sender.py --port COM5 --mode lost       # flags 全 0
python tools/vision_frame_sender.py --port COM5 --mode stuck-seq  # 序号不递增
```

两种模式都应让 RUNNING 状态在约 100ms 内锁存 **F8** 并停机。
如果显示 `CAM` 亮着却什么都不报，说明判据又退化回传输层了。

### 第 2 步：方向与零点

1. `--mode fixed --x 160`：球在中心时机构应保持水平（电机停在原点角）。
2. `--mode fixed --x 200`：球应朝中心方向移动；若反向，改
   `CTRL_DIRECTION_SIGN`（`ball_stepper_control.c`）。
3. 确认 `CTRL_IMU_ACCEL_AXIS`（0=X/1=Y/2=Z）与安装方向一致，
   必要时改 `CTRL_IMU_ACCEL_SIGN`、`CTRL_FF_SIGN`。

### 第 3 步：闭环

```bash
python tools/vision_frame_sender.py --port COM5 --mode normal --amplitude 60 --period 2
```

球不动但视觉在动，正好可以单独检验视觉 PD 的开环响应；之后再上真相机跑闭环。

### 第 4 步：接上真相机

把 `tools/maixcam_pos_link.py` 放进 MaixCAM 工程，按文件头注释接入 `main.py`，
TX 接到 MSPM0 的 PA31。要点：**丢球时继续发帧并把 flags 清 0，不要停发**，
否则 MSPM0 无法区分"链路断了"和"球丢了"。

---

## 10. 调参入口

所有参数集中在 `ball_stepper_control.c` 顶部的 8 个小节（用宏，不用魔数散落）。
必须按顺序标定，不要跳步：

1. `CTRL_DIRECTION_SIGN` —— 方向错了后面全白调。
2. `CTRL_TARGET_X_PX`（默认 160）—— 重新标定相机 O 点只改这里，
   不要用电机零点去补偿像素误差。
3. `CTRL_HARD_LIMIT_DEG`（默认 13）—— 机械限位，先保守。
4. `CTRL_IMU_ACCEL_AXIS / SIGN`、`CTRL_FF_SIGN`、`CTRL_FF_GAIN_DEG_PER_MPS2`
   —— 前馈方向与强度。
5. `CTRL_HOLD_KP/KD` → `CTRL_CAPTURE_KP/KD` —— 先调中心区，再调拉回区。
6. `CTRL_CREEP_LEFT/RIGHT_DEG` —— 静摩擦地板，只在低速滞留时起作用。

`CTRL_ZDT_RPM=80` / `CTRL_ZDT_ACCELERATION=24` 是驱动器侧的速度与加速度档位，
和上面这些是两回事（前者是电机转速，后者是控制律增益）。

---

## 11. 已知问题与待办

按严重程度排列。

### 1. ✅ 已修：丢球时静默冻结（`ball_stepper_control.c`）

原实现在 RUNNING 里只查 `Vision_IsOnline()`，而它的时间戳在"CRC 通过"时就刷新，
与 `flags`/`sequence` 无关。相机丢球时会持续发送 `flags=0` 的帧，于是：

- 链路判据永远成立，`BALL_FAULT_VISION_LOST` 不触发；
- `compute_feedback()` 一次都不再执行，`g_feedback_deg`/`g_error_px` 冻结；
- `motion_is_safe()` 只在 `compute_feedback()` 里调用，**边缘保护和超速保护同时失效**；
- 但 `process_acceleration()` 仍在置 `g_target_dirty`，命令继续下发。

净效果是变成一个"只按 IMU 前馈摆角度、且失去全部安全网"的开环驱动器，而 OLED
显示 CAM 在线、X 稳定。

修法：新增消费级时间戳 `g_last_sample_ms`（令牌门之后才刷新）+ 新故障码
`BALL_FAULT_BALL_LOST(F8)`，并把"链路断"与"球丢了"分开报。`Start()` 也要求
载荷在线，避免一进 RUNNING 就立刻故障。

### 2. 🔴 待办：相机端没有发送程序

`D:\E_Game_2026\Vision_Code\ec_vision_h_visual_state8_uart2_v6.12.37_q5_startup_triggered`
（最后修改 2026-08-01，早于 08-10 的迁移）内部是**相机直接驱动 ZDT**，
`q5_ff_link.py` 只处理"接收主控发来的单字节 IMU 前馈"。那张 12 字节位置帧的
发送端**不存在**，所以现在的 MSPM0 会永远停在 `WAIT_SENSORS`。

对策：`tools/maixcam_pos_link.py` 已经按协议写好（CRC 参数与 C 端逐位一致，
标准校验值 `"123456789" → 0x29B1` 已验证）。把它放进相机工程并按文件头注释接入
`main.py` 即可；在相机就绪之前，先用 `tools/vision_frame_sender.py` 在台架上跑通
MSPM0 这一侧。

### 3. ✅ 已解决：PA28/PA31 引脚冲突（改用硬件 I2C0 PA0/PA1，见第 13 节）

只要 OLED 还在用软件 I2C 驱动 PA28/PA31，相机链路就不可能工作。这是当前
"链路完全收不到帧"的直接原因之一。

### 4. 🟡 已知限制：速度估计有 ±15% 量化抖动

时间戳分辨率是 5ms（等于控制节拍周期），真实帧间隔 16.67ms 只能取到 15 或 20，
于是快速速度带约 ±17% 误差，加权 0.65 后约 ±11% 进入 D 项。中低速段
（v≈100px/s 时 D≈7°）表现为 ±0.8° 的逐帧纹波，与 `Kp` 在 10px 时的 1.65°
同量级，属于可接受但可改进项。

**修法（未实施）**：TIMG6 预分频 256、装载值 624（`CTRL_TIMER_INST_LOAD_VALUE`），
即 625 计数 = 5ms，一个计数 = 8us。在 `timebase.c` 里补一个 `Timebase_NowUs()`，
用 `DL_Timer_getTimerCount(CTRL_TIMER_INST)` 与 `g_ms_ticks` 组合成微秒时间
（注意两者之间的竞态，需要"读-读-再读"重试），再把 `received_ms` 改成 `received_us`
即可。`timebase.h` 里已记录该方案。

### 5. ✅ 已解决：OLED 刷新吃掉前台时间（硬件 I2C + 整页事务，见第 13 节）

`OLED_WR_Byte()` 每写 **1 个字节**就发一个完整的 I2C 事务（Start+地址+控制字+数据+Stop），
而 `OLED_Refresh()` 对每个数据字节各调一次；一次整屏 = 8 页 × 131 = 1048 次事务。
`OLED_ClearBuffer()` 又无条件把 8 页全标脏，dirty 优化被绕过。

按 `delay.c` 的 32MHz 标定推导，单次事务在 0.1~0.2ms 量级 → 整屏 0.1~0.25 秒，
而它每 100ms 触发一次。**这个数字没有实测**，但量级足够大到值得先量一下
（用 `Timebase_NowMs()` 在 `OLED_Refresh()` 前后各取一次就能测出来）。

好的一面：因为视觉用的是覆盖式邮箱，这种卡顿**只降低控制更新率，不累积相位**，
系统不会因此失稳。改善方向：把整页数据合并成一次长事务（SSD1306 支持连续写）、
把 `OLED_I2C_DELAY_US` 从 2 降到 1、或直接改用硬件 I2C。

### 6. 🟡 死代码：巡线小车的模块仍在工程里

`motor.c` / `route_fsm.c` / `bluetooth.c` / `gray_sensor.c` / `ir_sensor.c` /
`buzzer.c` / `mpu6050*.c` / `soft_i2c.c` 都不参与小球控制，其中 `route_fsm.o`
已被链接器剔除。`main.c` 仍会把 `GROUP1_IRQHandler` 里的 `Encoder_ISR()` 执行一遍
（轮式编码器计数，无人读取），`UART3_IRQHandler` 也仍挂着。清理它们能减小映像、
消除混淆（尤其 `motor.c` 里的 `PID_Compute` 容易被误认为是本工程的控制器）。

---

## 12. 变更记录

### 控制链路修复（本次）

- **新增** `timebase.c/.h`：统一毫秒时基，取代 `main.c` 里的 `g_ms_ticks`
  （视觉得以在中断里打"到达时刻"，且不再需要跨模块读别人的静态变量）。
- **修复** 丢球静默冻结：新增 `g_last_sample_ms` 消费级时间戳 +
  `BALL_FAULT_BALL_LOST(F8)`；`RUNNING` 判据改为"最后消费样本"层。
- **修复** `Vision_Poll()` 时间戳语义：改在 DMA 完成中断里打点（到达而非取走），
  并由模块自己读时基，去掉容易误解的 `now_ms` 形参。
- **新增** 两级在线判据 `Vision_IsOnline()` / `Vision_PayloadIsOnline()`，
  名字与语义对应（原 `Vision_IsOnline` 的注释写的是"有效更新"，实际只查链路）。
- **新增** 链路统计 `VisionStats_t`（CRC 通过 / 载荷有效 / flags 无效 / 序号重复 /
  CRC 错误 / 丢块），可用 `Vision_GetStats()` 读出；控制器侧配套
  `BallControl_GetConsumedFrames()`、`BallControl_GetSampleAgeMs()`。
  两者对比即可判断单槽邮箱有没有把帧合并掉。
- **新增** 投递口角度速率限制 `CTRL_COMMAND_SLEW_DEG_PER_S`（默认 900 deg/s，
  正常工况不生效，只拦单帧跳变）。
- **修复** `BallControl_Start()` 增加载荷在线前置条件。
- **新增** `tools/vision_frame_sender.py`（PC 台架帧发生器，含丢球/序号卡死/
  CRC 错误注入）与 `tools/maixcam_pos_link.py`（相机端发送模块）。
- **关闭** 上电电机自检：`main.c` 的 `zdt_start_position_demo()` 现在由
  `ENABLE_ZDT_DEMO`（默认 0）控制，不再每次上电都走 3200 脉冲。
- **重写** 本 README。

### 更早

- MaixCAM v6.12.37 Q5 直道快速控制律移植到 MSPM0（见 `BALL_CONTROL_MIGRATION.md`）。
- 12 字节视觉协议、DMA 双缓冲、CRC16、ZDT Emm V5 协议层、软件原点方案。

---

## 13. 更新：OLED 迁移到硬件 I2C

**问题**：`oled.c` 原来是软件 I2C，硬编码在 PA28/PA31，与 `UART_MaixCAM`（UART0）抢同一对 PINCM。
每次 `OLED_WR_Byte()` 都会把 PA28/PA31 重配成 GPIO，相机链路被摘掉；同时每写 1 个字节发一个完整
I2C 事务（1048 次事务/整屏），每个 bit 还有 2 次 `delay_us(2)` 忙等，主循环被占住。

**改法**：

1. SysConfig 新增 `I2C_Screen` = **I2C0**，SDA=**PA0**(PINCM1) / SCL=**PA1**(PINCM2)，**400 kHz**，
   并补上 `basicEnableController = true` 与 `basicControllerBusSpeed = 400000`
   （只加实例不设这两项时，生成的初始化里没有 `controllerInit`，硬件 I2C 不会工作）。
2. `oled.c` 传输层整体替换为 `DL_I2C_*` 轮询 + FIFO 流式写法：
   - 等 IDLE → `startControllerTransfer` 声明总长 → 按 FIFO 空位逐个塞字节 → 等 BUSY 清零 → 查 ERROR；
   - 三处超时守卫，任何一处卡住都返回失败，**不阻塞主循环**；
   - 遵守 errata I2C_ERR_13（启动后延时几个功能时钟再填 FIFO）。
3. 刷新策略从"每字节一次事务"改为**整页 2 次事务**（3 字节定位命令 + 128 字节显存数据）。
4. 对外 API 完全不变（`main.c` 一行没改），旧引脚定义全部删除。
5. 新增 `OLED_GetErrorCount()`：一次事务失败就 +1，用于判断"屏幕不亮是接线还是代码"。

**收益**：

| 项 | 软件 I2C | 硬件 I2C 400kHz |
| --- | --- | --- |
| 整屏 I2C 事务数 | 1048 | 16 |
| 整屏耗时（估算） | ~250 ms | **~25 ms** |
| 主循环占用 | 几乎占满 | < 3% |
| PA28/PA31 冲突 | 有 | **无** |
| 代码体积 | 3826 B | 4426 B（+600 B，换掉忙等循环） |

**同一时刻的控制节拍丢弃问题随之消失**：之前 250 ms 的刷屏会把 5ms 节拍封顶到 4 个，
控制率掉到十几赫兹；现在整屏 25 ms，控制节拍基本不再丢。