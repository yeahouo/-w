/**
 * @file led_status.h
 * @brief 板载 LED 状态指示 — 把整车状态映射到闪烁模式
 *
 * 没屏幕时最直观的排查手段。100Hz tick 驱动。
 *
 * 状态 → LED:
 *   IDLE     1Hz 慢闪(0.5s on / 0.5s off)
 *   START    5Hz 快闪(短促,表示正在加速起步)
 *   TRACKING 常亮
 *   ELEMENT  双闪(50ms on / 50ms off / 50ms on / 850ms off)
 *   FINISH   长亮 2 秒后熄灭
 *   ERROR    10Hz 急闪(报警)
 */
#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"

void LED_Status_Init(void);

/**
 * @brief 100Hz tick 驱动(由 Port_OnTick100Hz 调用)
 *   内部读 FSM_GetState 自动切换 pattern
 */
void LED_Status_Tick(void);

/* 手动覆盖(调试用,优先级最高,设 true 后 LED 完全由用户控制) */
void LED_Status_Override(bool on);
void LED_Status_Release(void);

#endif /* LED_STATUS_H */
