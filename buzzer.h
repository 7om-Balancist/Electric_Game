#ifndef __BUZZER_H__
#define __BUZZER_H__

/*
 * buzzer.h - 非阻塞蜂鸣器接口。
 *
 * 学习重点：
 * Buzzer_Beep(ms) 启动一次鸣叫；Buzzer_Update5ms() 负责倒计时关闭。
 * 这种设计不会卡住主循环，适合在状态切换、故障提示时使用。
 */

#include <stdbool.h>
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Beep(uint32_t duration_ms);
void Buzzer_Update5ms(void);
void Buzzer_Stop(void);

#endif /* __BUZZER_H__ */
