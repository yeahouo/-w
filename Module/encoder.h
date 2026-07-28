/**
 * @file encoder.h
 * @brief 编码器测速 — TIM 正交解码,100Hz 采样
 *
 * 工作流:
 *   - Port_EncoderRead 累计计数(由 BSP 在硬件层维护)
 *   - Encoder_SampleTick 每 10ms 调用一次,差分得转速
 *   - Encoder_GetSpeed 返回最近一次转速(单位:mm/s)
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "config.h"

void   Encoder_Init(void);

/**
 * @brief 100Hz 采样任务(由 Port_OnTick100Hz 调用)
 */
void   Encoder_SampleTick(void);

/**
 * @brief 读取最近一次转速
 * @return mm/s,带方向
 */
float  Encoder_GetSpeed(MotorChannel_t ch);

#endif /* ENCODER_H */
