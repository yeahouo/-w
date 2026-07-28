/**
 * @file encoder.c
 * @brief 编码器测速 — 100Hz 采样,差分得转速(mm/s)
 *
 * 工作流:
 *   - 硬件层(TIM Encoder)维护计数累加
 *   - Encoder_SampleTick 每 10ms 调用一次 Port_EncoderRead 拿增量计数
 *   - 转换为 mm/s 缓存,供 Encoder_GetSpeed 查询
 */
#include "encoder.h"
#include "port.h"
#include "config.h"

static float s_speed_mm_s[MOTOR_CH_COUNT];

void Encoder_Init(void)
{
    Port_EncoderResetAll();
    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        s_speed_mm_s[i] = 0.0f;
    }
}

void Encoder_SampleTick(void)
{
    /* 100Hz(每 10ms 一次) */
    const float dt_sec = (float)TICK_ENCODER_MS * 0.001f;
    const float counts_per_mm = (float)ENCODER_COUNTS_PER_REV / (float)CHASSIS_WHEEL_CIRCUM;

    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        int32_t cnt = Port_EncoderRead((MotorChannel_t)i);  /* 自上次以来的增量,带方向 */
        /* 计数 → mm → mm/s */
        s_speed_mm_s[i] = ((float)cnt / counts_per_mm) / dt_sec;
    }
}

float Encoder_GetSpeed(MotorChannel_t ch)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return 0.0f;
    return s_speed_mm_s[ch];
}
