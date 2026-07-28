/**
 * @file motor_drive.h
 * @brief 两轮差速电机驱动 — TB6612FNG, 软件 PWM (SysTick 2kHz) + 方向 GPIO
 *
 * 接线 (TB6612):
 *   左轮 = 电机 A: AIN1=PA13 AIN2=PA14 (方向)  PWMA=PB2 (PWM)
 *   右轮 = 电机 B: BIN1=PA17 BIN2=PA16 (方向)  PWMB=PB3 (PWM)
 *
 * PWM: 软件 bit-bang, SysTick 2kHz × 20 档相位 → PWM 100Hz, 占空比 0~20 档
 *      MotorDrive_Tick() 由 SysTick_Handler 每 tick 调用 (中断上下文, 仅 GPIO 翻转)
 *
 * 极性: 左轮接线反向, 故 LEFT_MOTOR_POLARITY=-1 翻转; dir=1 统一为物理前进
 */
#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

/* 配 6 个电机引脚 (AIN1/2/BIN1/2/PWMA/B) 为输出, 初值全低 */
void MotorDrive_Init(void);

/**
 * @brief 设左右轮 duty + 方向, 并立即应用到方向 GPIO
 * @param ld   左轮 duty  (0~20)
 * @param ldir 左轮方向  (1=正转/前进, 0=反转)
 * @param rd   右轮 duty  (0~20)
 * @param rdir 右轮方向  (1/0)
 * @note  duty=0 时该轮方向脚自动清零 (松轮)
 *       PWM 由 SysTick 调 MotorDrive_Tick() 持续输出, 无需主循环干预
 */
void MotorDrive_Set(uint8_t ld, uint8_t ldir, uint8_t rd, uint8_t rdir);

/* 软件 PWM 一拍 — 由 SysTick_Handler 每 tick 调用 (2kHz, 中断上下文) */
void MotorDrive_Tick(void);

/* 两轮全停 (duty=0 + 清方向) */
void MotorDrive_Stop(void);

/* 当前 duty 查询 (日志用) */
uint8_t MotorDrive_GetLeftDuty(void);
uint8_t MotorDrive_GetRightDuty(void);

#endif /* MOTOR_DRIVE_H */
