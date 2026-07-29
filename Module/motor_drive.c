/**
 * @file motor_drive.c
 * @brief 两轮差速电机驱动实现 — 软件 PWM + 方向 GPIO
 *
 * 引脚: AIN1=PA13 AIN2=PA14 BIN1=PA17 BIN2=PA16 (方向, GPIOA)
 *       PWMA=PB2   PWMB=PB3                       (PWM,   GPIOB, 软件 bit-bang)
 */

#include "ti_msp_dl_config.h"
#include "motor_drive.h"

/* ============================================================
 *  引脚定义
 * ============================================================ */
#define AIN1_PINCM      IOMUX_PINCM35   /* PA13 */
#define AIN2_PINCM      IOMUX_PINCM36   /* PA14 */
#define PWMA_PINCM      IOMUX_PINCM15   /* PB2  */
#define BIN1_PINCM      IOMUX_PINCM39   /* PA17 */
#define BIN2_PINCM      IOMUX_PINCM38   /* PA16 */
#define PWMB_PINCM      IOMUX_PINCM16   /* PB3  */

#define AIN1_PIN        DL_GPIO_PIN_13
#define AIN2_PIN        DL_GPIO_PIN_14
#define BIN1_PIN        DL_GPIO_PIN_17
#define BIN2_PIN        DL_GPIO_PIN_16
#define PWMA_PIN        DL_GPIO_PIN_2
#define PWMB_PIN        DL_GPIO_PIN_3

#define DIR_PINS_MASK   (AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN)
#define PWM_PINS_MASK   (PWMA_PIN | PWMB_PIN)

/* 电机极性: +1=默认, -1=翻转。
 * 两轮同向安装 (+1 时皆后退), 两轮都翻 (-1) 使 dir=1 = 物理前进 (车头朝前)。 */
#define LEFT_MOTOR_POLARITY   (-1)
#define RIGHT_MOTOR_POLARITY  (-1)

/* 软件 PWM: SysTick 2kHz × 20 档 → 100Hz PWM, 占空比 0~20 */
#define PWM_PHASE_MAX   20

/* ============================================================
 *  模块状态 (volatile: SysTick 中断读, 主循环写)
 * ============================================================ */
static volatile uint8_t s_l_duty = 0;
static volatile uint8_t s_r_duty = 0;
static volatile uint8_t s_l_dir  = 0;
static volatile uint8_t s_r_dir  = 0;

/* ============================================================
 *  方向应用 — 把 duty/dir 翻译到 AIN/BIN 方向脚
 * ============================================================ */
static void apply_dir(void)
{
    /* 应用极性: POLARITY<0 时翻转该轮方向位(异或 1), 收敛到单点 */
    uint8_t l_dir = (uint8_t)(s_l_dir ^ (uint8_t)(LEFT_MOTOR_POLARITY  < 0));
    uint8_t r_dir = (uint8_t)(s_r_dir ^ (uint8_t)(RIGHT_MOTOR_POLARITY < 0));

    if (s_l_duty == 0) {
        DL_GPIO_clearPins(GPIOA, AIN1_PIN | AIN2_PIN);
    } else if (l_dir) {
        DL_GPIO_setPins  (GPIOA, AIN1_PIN);
        DL_GPIO_clearPins(GPIOA, AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, AIN1_PIN);
        DL_GPIO_setPins  (GPIOA, AIN2_PIN);
    }
    if (s_r_duty == 0) {
        DL_GPIO_clearPins(GPIOA, BIN1_PIN | BIN2_PIN);
    } else if (r_dir) {
        DL_GPIO_setPins  (GPIOA, BIN1_PIN);
        DL_GPIO_clearPins(GPIOA, BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, BIN1_PIN);
        DL_GPIO_setPins  (GPIOA, BIN2_PIN);
    }
}

/* ============================================================
 *  公开 API
 * ============================================================ */
void MotorDrive_Init(void)
{
    DL_GPIO_initDigitalOutput(AIN1_PINCM);
    DL_GPIO_initDigitalOutput(AIN2_PINCM);
    DL_GPIO_initDigitalOutput(BIN1_PINCM);
    DL_GPIO_initDigitalOutput(BIN2_PINCM);
    DL_GPIO_initDigitalOutput(PWMA_PINCM);
    DL_GPIO_initDigitalOutput(PWMB_PINCM);
    DL_GPIO_enableOutput(GPIOA, DIR_PINS_MASK);
    DL_GPIO_enableOutput(GPIOB, PWM_PINS_MASK);
    DL_GPIO_clearPins(GPIOA, DIR_PINS_MASK);
    DL_GPIO_clearPins(GPIOB, PWM_PINS_MASK);
}

void MotorDrive_Set(uint8_t ld, uint8_t ldir, uint8_t rd, uint8_t rdir)
{
    s_l_duty = ld;
    s_r_duty = rd;
    s_l_dir  = ldir;
    s_r_dir  = rdir;
    apply_dir();
}

void MotorDrive_Stop(void)
{
    s_l_duty = 0;
    s_r_duty = 0;
    apply_dir();
}

void MotorDrive_Tick(void)
{
    /* 此函数由 SysTick_Handler (2kHz, 中断上下文) 调用 — 仅 GPIO 翻转, 无阻塞 */
    static uint8_t pwm_phase = 0;

    if (pwm_phase < s_l_duty) DL_GPIO_setPins  (GPIOB, PWMA_PIN);
    else                      DL_GPIO_clearPins(GPIOB, PWMA_PIN);
    if (pwm_phase < s_r_duty) DL_GPIO_setPins  (GPIOB, PWMB_PIN);
    else                      DL_GPIO_clearPins(GPIOB, PWMB_PIN);

    if (++pwm_phase >= PWM_PHASE_MAX) pwm_phase = 0;
}

uint8_t MotorDrive_GetLeftDuty(void)  { return s_l_duty; }
uint8_t MotorDrive_GetRightDuty(void) { return s_r_duty; }
