# MaixCAM + MSPM0 + ZDT Ball Control

## Wiring

| Link | MSPM0 peripheral | Pins | Baud |
| --- | --- | --- | --- |
| MaixCAM position | UART0 | PA28 TX, PA31 RX | 115200 |
| JY61P | UART1 | PA17 TX, PA18 RX | 9600 |
| ZDT Emm V5 | UART2 | PA23 TX, PA24 RX | 115200 |
| Bluetooth (retained, unused by main) | UART3 | PB2 TX, PB3 RX | 38400 |

All boards and the ZDT driver must share signal ground. Cross TX and RX between
devices. Confirm that every UART signal uses 3.3 V logic before connecting it to
the MSPM0.

## Vision packet

MaixCAM sends one little-endian 12-byte packet for every valid camera result:

| Offset | Field | Format |
| --- | --- | --- |
| 0 | Header | `0xA5` |
| 1 | Header | `0x5A` |
| 2 | Sequence | uint8, increments every frame |
| 3 | Flags | bit 0 = position valid |
| 4 | Ball X low | int16 little-endian |
| 5 | Ball X high | int16 little-endian |
| 6 | Camera time low | uint16 milliseconds |
| 7 | Camera time high | uint16 milliseconds |
| 8 | Confidence | uint8, 0 to 255 |
| 9 | Reserved | send zero |
| 10 | CRC low | CRC-16/CCITT-FALSE over bytes 0 through 9 |
| 11 | CRC high | CRC-16/CCITT-FALSE over bytes 0 through 9 |

CRC parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR. The MSPM0 parser searches for `A5 5A`, so it can recover after a
partial packet or a reset on either side.

## Operation

- Power-up waits for valid vision and JY61P frames.
- The controller enables ZDT and reads its encoder position as a software zero.
- It never writes the ZDT driver's stored zero.
- KEY1 starts closed-loop control after the display shows `ARMED`.
- KEY4 stops motion.
- KEY3 resets a latched fault after the cause has been corrected.
- KEY2 zeroes yaw only; it does not change longitudinal acceleration zero.
- KEY1 captures the current JY61P acceleration as the stationary feed-forward
  zero and then starts the Q5 controller.

The controller faults and stops on a 100 ms vision timeout, JY61P acceleration
timeout, repeated ZDT timeout, or failure to read the startup encoder position.

## Migrated control law

The MSPM0 controller follows the MaixCAM v6.12.37 Q5 straight-fast law:

- 65% fresh velocity plus 35% long-window velocity;
- hold/capture PD gain blending from 5 to 11 pixels;
- stronger damping and quadratic rescue while the ball moves outward;
- predicted-error taper while the ball is returning to the center;
- continuous left/right static-friction floors;
- JY61P longitudinal acceleration feed-forward with warm-up, low-pass,
  slew limit, error-dependent authority and conflict guard;
- startup acceleration trigger with temporarily stronger feed-forward;
- final 13-degree motor hard limit.

The default longitudinal IMU channel is JY61P AccX. Change
`CTRL_IMU_ACCEL_AXIS` in `ball_stepper_control.c` to `1` for AccY or `2` for
AccZ after checking the physical installation. `CTRL_IMU_ACCEL_SIGN` controls
the sensor direction. JY61P must output WIT frame `0x51` continuously.

## Initial tuning

All control parameters are at the top of `ball_stepper_control.c`. Verify these
in order on a supported bench before placing a ball on the mechanism:

1. Confirm encoder position read and absolute command direction.
2. Confirm `CTRL_DIRECTION_SIGN` moves the ball toward the target.
3. Confirm AccX matches the vehicle longitudinal axis; select AccY/AccZ if needed.
4. Determine `CTRL_IMU_ACCEL_SIGN` and `CTRL_FF_SIGN` on the bench.
5. Increase feedback and feedforward gains gradually while retaining the
   13-degree hard limit.

## SysConfig status

The checked configuration enables only MaixCAM RX DMA. TX DMA is disabled,
the transfer size is 12 bytes, and the RX channel is `DMA_CH0`.
