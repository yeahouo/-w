/**
 * @file button.h
 * @brief USER 按键驱动 — PB8, 低电平有效, 时间戳去抖 + 按下事件
 *
 * 接线: USER 按键一端接 PB8, 一端接 GND, 内部上拉
 *       未按 = 高电平, 按下 = 低电平 (BUTTON_ACTIVE_LEVEL = 0)
 *       若实测相反, 改 button.c 顶部的 BUTTON_ACTIVE_LEVEL
 *
 * 用法:
 *   Button_Init();
 *   while (1) {
 *       if (Button_Consume()) {  / * 检测到一次按下 (下降沿), 读后清零 * /
 *           ... 响应 ...
 *       }
 *   }
 *
 * 去抖原理: 原始电平持续稳定 >= BUTTON_DEBOUNCE_MS (20ms) 才采纳为新状态,
 *           消除按键抖动。需在主循环周期调用 (当前 ~10Hz 即可)。
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

/* PB8 配为数字输入 + 内部上拉 */
void Button_Init(void);

/**
 * @brief 去抖后的当前状态
 * @return true=按下, false=松开
 * @note  每次调用都会更新去抖状态, 需周期调用
 */
bool Button_IsPressed(void);

/**
 * @brief 短按事件 (下降沿)
 * @return 自上次 Consume 以来是否检测到一次"新按下"; 读后自动清零
 * @note  内部会先调用 Button_IsPressed() 刷新状态
 */
bool Button_Consume(void);

/**
 * @brief 长按事件 — 按住超过 LONG_PRESS_MS 触发一次, 松开后才允许再次触发
 * @return true=刚满长按阈值的那一帧
 */
bool Button_ConsumeLong(void);

#endif /* BUTTON_H */
