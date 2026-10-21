/**
 * mpu6050_platform.c — 已停用，I2C 已从 SysConfig 移除
 *
 * 当前工程使用 JY61P (UART0) 代替 BNO055/MPU6050 (I2C)。
 * 此文件保留为空壳以避免链接错误。
 *
 * 学习重点：
 * 这是硬件抽象层的占位实现。上层 IMU 逻辑不直接关心 I2C 来自硬件外设、
 * 软件 I2C，还是暂时不可用的平台空壳。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data,
    size_t length, uint32_t timeout_ticks)
{
    /*
     * 当前工程未启用硬件 I2C，因此平台写函数固定失败。
     * 如果以后恢复 BNO055/MPU6050，可在这里接入 SoftI2C_WriteReg()
     * 或 SysConfig 生成的硬件 I2C 发送函数。
     */
    (void)address; (void)data; (void)length; (void)timeout_ticks;
    return false;
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    /*
     * 典型寄存器读流程是：先写寄存器地址，再 repeated-start 读回数据。
     * 当前为空壳，保留函数签名是为了让旧 IMU 代码能通过链接。
     */
    (void)address; (void)tx; (void)tx_length;
    (void)rx; (void)rx_length; (void)timeout_ticks;
    return false;
}
