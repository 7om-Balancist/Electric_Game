#ifndef __KEY_H__
#define __KEY_H__

/*
 * key.h - 四键输入接口。
 *
 * 使用方式：
 * - 初始化：Key_Init()
 * - GPIO 中断：GROUP1_IRQHandler 中调用 Key_GPIO_IRQHandler()
 * - 5ms 周期：Key_Scan5ms()
 * - 主循环：Key_GetPressEvent(KEY_ID_x) 获取一次性按下事件
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* KEY1~KEY4 对应 SysConfig 中 PB1/PB10/PB11/PB14 四个上拉按键。 */
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_NUM
} KeyId_t;

void Key_Init(void);
void Key_GPIO_IRQHandler(void);
void Key_Scan5ms(void);
bool Key_GetPressEvent(KeyId_t id);

#endif /* __KEY_H__ */
