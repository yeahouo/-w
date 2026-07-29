/**
 * @file line_sensor.c
 * @brief 5 路灰度传感器读取实现 — 多次采样 + 多数表决 (v6.2)
 *
 * 引脚: L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9
 *       L1 在 GPIOB, 其余在 GPIOA
 * 当前只用 3 路 (L1/M/R1); L2/R2 未装, LineSensor_Read 强制清零 (空响应)
 *
 * 目的: 补偿硬件灵敏度不足 (检测延迟 / 压线不触发 / 边缘模糊)
 * 做法: 每次调用连读 N 次 (间隔 NOP), 每路累计命中次数
 *       ≥ V6_SENSOR_THRESHOLD 才算真命中
 */

#include "ti_msp_dl_config.h"
#include "line_sensor.h"

/* ============================================================
 *  引脚定义
 * ============================================================ */
#define LINE_ACTIVE_LEVEL  0   /* 0 = 黑线时读到 0;  1 = 黑线时读到 1 */

#define L1_PINCM   IOMUX_PINCM43   /* PB17 */
#define L2_PINCM   IOMUX_PINCM34   /* PA12 */
#define M_PINCM    IOMUX_PINCM47   /* PA22 */
#define R1_PINCM   IOMUX_PINCM60   /* PA27 */
#define R2_PINCM   IOMUX_PINCM20   /* PA9  */

#define L1_PIN     DL_GPIO_PIN_17
#define L2_PIN     DL_GPIO_PIN_12
#define M_PIN      DL_GPIO_PIN_22
#define R1_PIN     DL_GPIO_PIN_27
#define R2_PIN     DL_GPIO_PIN_9

#define LINE_PINS_A_MASK  (L2_PIN | M_PIN | R1_PIN | R2_PIN)   /* GPIOA */
#define LINE_PINS_B_MASK  (L1_PIN)                             /* GPIOB */

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
    /* 5 路: 数字输入 + 内部上拉
     * (当前灰度未装, main.c 暂不调用; 装上后取消 main.c 条件编译) */
    DL_GPIO_initDigitalInputFeatures(L1_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(L2_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(M_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R1_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R2_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t LineSensor_Read(void)
{
    uint8_t l1_cnt = 0, l2_cnt = 0, m_cnt = 0, r1_cnt = 0, r2_cnt = 0;

    /* 多次采样 + 累计 */
    for (uint8_t i = 0; i < V6_SENSOR_SAMPLES; i++) {
        uint32_t a = DL_GPIO_readPins(GPIOA, LINE_PINS_A_MASK);
        uint32_t b = DL_GPIO_readPins(GPIOB, LINE_PINS_B_MASK);

        uint8_t l1 = (b & L1_PIN) ? 1 : 0;
        uint8_t l2 = (a & L2_PIN) ? 1 : 0;
        uint8_t m  = (a & M_PIN)  ? 1 : 0;
        uint8_t r1 = (a & R1_PIN) ? 1 : 0;
        uint8_t r2 = (a & R2_PIN) ? 1 : 0;

#if LINE_ACTIVE_LEVEL == 1
        l1 ^= 1; l2 ^= 1; m ^= 1; r1 ^= 1; r2 ^= 1;
#endif

        l1_cnt += l1; l2_cnt += l2; m_cnt += m;
        r1_cnt += r1; r2_cnt += r2;

        if (i < V6_SENSOR_SAMPLES - 1) sensor_delay();
    }

    /* 多数表决 */
    uint8_t l1 = (l1_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t l2 = (l2_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t m  = (m_cnt  >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t r1 = (r1_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t r2 = (r2_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;

    /* L2/R2 未装, 空响应 — 强制无黑线, 不参与循迹 */
    l2 = 0; r2 = 0;

    return (uint8_t)((l1 << 0) | (l2 << 1) | (m << 2) | (r1 << 3) | (r2 << 4));
}
