/**
 * @file motor.h
 * @brief 电机层 — 四路 PWM + 单电机速度 PI 闭环(1kHz)
 *
 * 上层通过 Motor_SetTargetSpeed 下发目标速度(mm/s),
 * 内部在 1kHz tick 中跑 PI 闭环,输出 PWM 给 H 桥。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* 初始化(在 Port_HalInit 之后调用) */
void Motor_Init(void);

/**
 * @brief 设置单电机目标速度
 * @param ch     通道
 * @param rpm    目标转速,正=前进,负=后退,单位 mm/s(由运动学换算)
 */
void Motor_SetTargetSpeed(MotorChannel_t ch, int16_t target_mm_s);

/**
 * @brief 直接设置 PWM(调试用,绕过 PI)
 */
void Motor_SetRawPWM(MotorChannel_t ch, int16_t pwm);

/**
 * @brief 1kHz tick — PI 闭环驱动(由 Port_OnTick1kHz 调用)
 */
void Motor_PI_Tick(void);

/**
 * @brief 全部电机 PWM 归零
 */
void Motor_AllStop(void);

/* ---------- 遥测/调试 getter ---------- */
int16_t Motor_GetTargetSpeed(MotorChannel_t ch);   /* 目标 mm/s */
int16_t Motor_GetOutputPWM(MotorChannel_t ch);     /* 当前 PWM(已限幅) */
bool    Motor_IsStalled(MotorChannel_t ch);        /* 堵转标志(目标≠0 但实测≈0 持续 300ms) */

#endif /* MOTOR_H */
