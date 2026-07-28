/**
 * @file motion.c
 * @brief 运动学解算 — 差速模型(v,ω) → 四轮目标速度(mm/s)
 *
 * 公式:
 *   v_L = v - ω * TRACK/2
 *   v_R = v + ω * TRACK/2
 *   左前 = 左后 = v_L ; 右前 = 右后 = v_R
 *
 * 不直接调硬件,通过 Motor_SetTargetSpeed 输出。
 */
#include "motion.h"
#include "config.h"
#include "motor.h"

void Motion_SetTwist(float v_lin, float omega)
{
    float half_track = CHASSIS_WHEEL_TRACK * 0.5f;
    float v_left  = v_lin - omega * half_track;
    float v_right = v_lin + omega * half_track;

    Motor_SetTargetSpeed(MOTOR_LF, (int16_t)v_left);
    Motor_SetTargetSpeed(MOTOR_LR, (int16_t)v_left);
    Motor_SetTargetSpeed(MOTOR_RF, (int16_t)v_right);
    Motor_SetTargetSpeed(MOTOR_RR, (int16_t)v_right);
}

void Motion_DiffDrive(float v_left, float v_right)
{
    Motor_SetTargetSpeed(MOTOR_LF, (int16_t)v_left);
    Motor_SetTargetSpeed(MOTOR_LR, (int16_t)v_left);
    Motor_SetTargetSpeed(MOTOR_RF, (int16_t)v_right);
    Motor_SetTargetSpeed(MOTOR_RR, (int16_t)v_right);
}

void Motion_Brake(void)
{
    Motor_SetTargetSpeed(MOTOR_LF, 0);
    Motor_SetTargetSpeed(MOTOR_LR, 0);
    Motor_SetTargetSpeed(MOTOR_RF, 0);
    Motor_SetTargetSpeed(MOTOR_RR, 0);
}
