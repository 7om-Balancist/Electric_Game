# Electric_Game

2026 年全国电子设计竞赛 H 题全部工作内容。

本仓库用**两个分支**存放两个独立工程，每个分支的根目录就是一个完整的 CCS 工程：

| 分支 | 工程 | 技术要点 |
| --- | --- | --- |
| [`LinerCar`](https://github.com/7om-Balancist/Electric_Game/tree/LinerCar) | 循迹小车 | 8 路灰度串行采样 · 加权质心定位 · 位置 PI（KP 2.0 / KI 0.3）· L 形直角状态机 · 丢线保持 · HC-05 蓝牙在线调参 · ILI9341 实时监控 |
| [`ZDT_Ball-Controller`](https://github.com/7om-Balancist/Electric_Game/tree/ZDT_Ball-Controller) | 钢球位置闭环 | MaixCAM 视觉（12B 帧 + CRC16 + DMA）· JY61P 加速度前馈 · 位置-速度 PD + 非线性补偿 · ZDT Emm V5 绝对位置指令 · 7 态状态机 + 8 个锁存故障 · 硬件 I2C 显示 |

## 取用

```bash
# 循迹小车
git clone -b LinerCar https://github.com/7om-Balancist/Electric_Game.git

# 钢球位置闭环
git clone -b ZDT_Ball-Controller https://github.com/7om-Balancist/Electric_Game.git
```

用 CCS Theia / CCS 打开对应目录即可构建 `Debug` 配置。

> `Debug/`（编译产物）、`.project/.cproject`（CCS 自动生成）、`.settings/`、`targetConfigs/` 未入库，
> 导入工程后由 CCS 与 SysConfig 重新生成即可。

## 硬件平台

- MCU：**MSPM0G3507**（LQFP-64），SysConfig + DriverLib，编译器 `ti-cgt-armllvm_5.1.1.LTS`
- 视觉：MaixCAM / K230（钢球位置）· 灰度 / 红外（循迹）
- 执行：ZDT 闭环步进（钢球）· TB6612 双直流电机 + 编码器（循迹）
- 传感：JY61P IMU（加速度前馈 / 航向）