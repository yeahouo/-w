/**
 * @file line_sensor.c
 * @brief 6 路灰度传感器读取实现 — 多次采样 + 多数表决 (v6.2)
 *
 * 引脚 (从左到右 S1..S6):
 *   S1=OUT7=PA12  S2=OUT6=PB17  S3=OUT5=PA22
 *   S4=OUT4=PB16  S5=OUT3=PA27  S6=OUT2=PA9
 *   S1/S3/S5/S6 在 GPIOA, S2/S4 在 GPIOB
 *
 * 目的: 补偿硬件灵敏度不足 (检测延迟 / 压线不触发 / 边缘模糊)
 * 做法: 每次调用连读 N 次 (间隔 NOP), 每路累计命中次数
 *       ≥ V6_SENSOR_THRESHOLD 才算真命中
 */

#include "ti_msp_dl_config.h"
#include "line_sensor.h"

/* ============================================================
 *  引脚定义 (从左到右 S1..S6)
 * ============================================================ */
#define LINE_ACTIVE_LEVEL  0   /* 0 = 黑线时读到 0;  1 = 黑线时读到 1 */

#define S1_PINCM   IOMUX_PINCM34   /* PA12, OUT7, 最左 */
#define S2_PINCM   IOMUX_PINCM43   /* PB17, OUT6 */
#define S3_PINCM   IOMUX_PINCM47   /* PA22, OUT5 */
#define S4_PINCM   IOMUX_PINCM33   /* PB16, OUT4 */
#define S5_PINCM   IOMUX_PINCM60   /* PA27, OUT3 */
#define S6_PINCM   IOMUX_PINCM20   /* PA9,  OUT2, 最右 */

#define S1_PIN     DL_GPIO_PIN_12
#define S2_PIN     DL_GPIO_PIN_17
#define S3_PIN     DL_GPIO_PIN_22
#define S4_PIN     DL_GPIO_PIN_16
#define S5_PIN     DL_GPIO_PIN_27
#define S6_PIN     DL_GPIO_PIN_9

#define LINE_PINS_A_MASK  (S1_PIN | S3_PIN | S5_PIN | S6_PIN)   /* GPIOA */
#define LINE_PINS_B_MASK  (S2_PIN | S4_PIN)                     /* GPIOB */

/* ============================================================
 *  多数表决参数
 * ============================================================ */
#define V6_SENSOR_SAMPLES    (5u)   /* 采样次数 (越多越稳但越慢, 奇数便于表决) */
#define V6_SENSOR_THRESHOLD  (3u)   /* 命中阈值 (≥ 半数) */
#define V6_SENSOR_DELAY_NOP  (20u)  /* 采样间隔 NOP 数 (硬件响应慢时调大) */

static inline void sensor_delay(void)
{
    for (volatile uint32_t i = 0; i < V6_SENSOR_DELAY_NOP; i++) {
        __NOP();
    }
}

/* ============================================================
 *  公开 API
 * ============================================================ */
void LineSensor_Init(void)
{
    /* 6 路: 数字输入 + 内部上拉 */
    DL_GPIO_initDigitalInputFeatures(S1_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(S2_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(S3_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(S4_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(S5_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(S6_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t LineSensor_Read(void)
{
    uint8_t s1_cnt = 0, s2_cnt = 0, s3_cnt = 0, s4_cnt = 0, s5_cnt = 0, s6_cnt = 0;

    /* 多次采样 + 累计 */
    for (uint8_t i = 0; i < V6_SENSOR_SAMPLES; i++) {
        uint32_t a = DL_GPIO_readPins(GPIOA, LINE_PINS_A_MASK);
        uint32_t b = DL_GPIO_readPins(GPIOB, LINE_PINS_B_MASK);

        uint8_t s1 = (a & S1_PIN) ? 1 : 0;
        uint8_t s2 = (b & S2_PIN) ? 1 : 0;
        uint8_t s3 = (a & S3_PIN) ? 1 : 0;
        uint8_t s4 = (b & S4_PIN) ? 1 : 0;
        uint8_t s5 = (a & S5_PIN) ? 1 : 0;
        uint8_t s6 = (a & S6_PIN) ? 1 : 0;

#if LINE_ACTIVE_LEVEL == 1
        s1 ^= 1; s2 ^= 1; s3 ^= 1; s4 ^= 1; s5 ^= 1; s6 ^= 1;
#endif

        s1_cnt += s1; s2_cnt += s2; s3_cnt += s3;
        s4_cnt += s4; s5_cnt += s5; s6_cnt += s6;

        if (i < V6_SENSOR_SAMPLES - 1) sensor_delay();
    }

    /* 多数表决 */
    uint8_t s1 = (s1_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t s2 = (s2_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t s3 = (s3_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t s4 = (s4_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t s5 = (s5_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t s6 = (s6_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;

    return (uint8_t)((s1 << 0) | (s2 << 1) | (s3 << 2) | (s4 << 3) | (s5 << 4) | (s6 << 5));
}
