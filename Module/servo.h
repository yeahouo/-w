/**
 * @file servo.h
 * @brief HWZ020 舵机驱动 (20kg·cm 数字舵机, 50Hz PWM, 1-2ms 脉宽, 0-180°)
 *
 * 接线 (WHEELTEC HWZ020):
 *   信号(橙) = PA0  (TIMA0_CCP0)
 *   VCC (红) = 5~7.4V (单独供电, 勿接主控 3.3V — 大扭矩电流会拉垮主控)
 *   GND (棕) = 共地 (电池地 + 主控 GND 必须共地)
 *
 * 前提: 无 — servo.c 纯代码自配 TimerA0 (不用 SysConfig)。
 *   TimerA0 CCP0 → PA0, 50Hz PWM, main 里调 Servo_Init() 即可。
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/**
 * @brief 初始化舵机 (PA0 配 GPIO 输出 + 软件 PWM 中位 90°)
 */
void Servo_Init(void);

/**
 * @brief 软件 PWM 节拍, 由 SysTick_Handler (2kHz) 调用
 * @note  必须加进 SysTick_Handler, 否则舵机不动
 */
void Servo_Tick(void);

/**
 * @brief 设置舵机角度
 * @param angle 0~180° (越界截到 180); 线性映射到脉宽 1.0~2.0ms
 */
void Servo_SetAngle(uint8_t angle);

/**
 * @brief 直接设置脉宽 (精细调校用)
 * @param us 1000~2000us (越界截到范围); 1500us = 中位
 */
void Servo_SetPulseUs(uint16_t us);

#endif /* SERVO_H */
