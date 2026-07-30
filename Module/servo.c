/**
 * @file servo.c
 * @brief HWZ020 舵机驱动 — 软件 PWM (SysTick 2kHz 驱动 A8, 50Hz)
 *
 * A8 = 核心板丝印, 对应芯片 PA0 (IOMUX_PINCM1 / GPIOA / PIN_0)
 *
 * 原理: SysTick 2kHz = 0.5ms/tick
 *   50Hz 周期 = 40 ticks (20ms)
 *   脉宽 1-5 ticks = 0.5-2.5ms (覆盖 HWZ020 的 500-2500us)
 *
 * 接线: 信号(黄)=A8, VCC(红)=5~7.4V单独, GND(棕)=共地
 */
#include "ti_msp_dl_config.h"
#include "servo.h"

#define A8_PHASE_MAX   (40u)             /* 50Hz = 40 ticks (20ms @ 0.5ms/tick) */
#define A8_PORT        GPIOA             /* A8 在 GPIOA */
#define A8_PIN         DL_GPIO_PIN_0     /* A8 = PA0 */
#define A8_PINCM       IOMUX_PINCM1      /* A8 (PA0) 的引脚配置寄存器 */

static volatile uint8_t s_servo_pulse = 3;  /* 脉宽 ticks (3=1.5ms 中位) */
static volatile uint8_t s_servo_phase = 0;  /* 周期计数 0..39 */

void Servo_Init(void)
{
    /* A8 配推挽输出 */
    DL_GPIO_initDigitalOutput(A8_PINCM);
    DL_GPIO_enableOutput(A8_PORT, A8_PIN);
    DL_GPIO_clearPins(A8_PORT, A8_PIN);
    s_servo_pulse = 3;    /* 上电中位 90° */
    s_servo_phase = 0;
}

/* 由 SysTick_Handler (2kHz) 调用 — 软件 PWM 翻转 A8 */
void Servo_Tick(void)
{
    if (s_servo_phase < s_servo_pulse) {
        DL_GPIO_setPins(A8_PORT, A8_PIN);    /* 脉宽内: 高 */
    } else {
        DL_GPIO_clearPins(A8_PORT, A8_PIN);  /* 其余: 低 */
    }
    if (++s_servo_phase >= A8_PHASE_MAX) s_servo_phase = 0;
}

void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    /* angle 0..180 → 脉宽 1..5 ticks (0.5..2.5ms) */
    s_servo_pulse = (uint8_t)(1u + (uint32_t)4u * angle / 180u);
}

void Servo_SetPulseUs(uint16_t us)
{
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    /* us → ticks (0.5ms = 500us = 1 tick) */
    uint8_t ticks = (uint8_t)(us / 500u);
    if (ticks < 1) ticks = 1;
    if (ticks > 5) ticks = 5;
    s_servo_pulse = ticks;
}
