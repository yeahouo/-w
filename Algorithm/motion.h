/**
 * @file motion.h
 * @brief 运动学解算 — 将 (线速度 v, 角速度 ω) 解算为四轮目标速度
 *
 * 四轮驱动按"两轮差速模型"建模:同侧两轮同速。
 * 公式:
 *   v_L = v - ω * WHEEL_TRACK / 2
 *   v_R = v + ω * WHEEL_TRACK / 2
 * 左两轮目标 = v_L,右两轮目标 = v_R(单位:mm/s)
 *
 * 内部不调用任何硬件 API,输出经 Motor_SetTargetSpeed 下去。
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/**
 * @brief 设置整车目标 twist
 * @param v_lin  线速度(mm/s),前进为正
 * @param omega  角速度(rad/s),左转为正
 */
void Motion_SetTwist(float v_lin, float omega);

/**
 * @brief 直接给定左右轮目标速度(mm/s)
 */
void Motion_DiffDrive(float v_left, float v_right);

/**
 * @brief 急停(目标速度归零)
 */
void Motion_Brake(void);

#endif /* MOTION_H */
