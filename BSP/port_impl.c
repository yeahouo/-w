/**
 * @file port_impl.c
 * @brief Port 接口的 BSP 实现 — TI MSPM0G3507 + 外接 TB6612FNG (2 电机)
 *
 * 真实接线（S28A/S27F 底板资源分配图）：
 *   方向脚: PA13(AIN1) PA14(AIN2) PA17(BIN1) PA16(BIN2)  — 全 GPIOA
 *   PWM 脚: PB2(PWMA) PB3(PWMB)                          — GPIOB
 *   编码器: PA25(左 A 相 EXTI) PA26(左 B 相)
 *           PB20(右 A 相 EXTI) PB24(右 B 相)
 *   LED:    PB9
 *   KEY:    PB8 (USER 启动)
 *   UART0:  PA10(TX) / PA11(RX) 460800 — 调试 + OpenMV RX 共用
 *   STBY:   硬接 3.3V 常使能
 *
 * SysTick (Cortex-M0+ 自带,1kHz) 提供 PI tick + 系统时钟
 *   不依赖 SysConfig Timer 模块,避免 // UNSURE 属性名问题
 *
 * Step A: PWM 用 GPIO 拉高/拉低模拟 100%/0% 占空比
 * Step B: 升级为 TIMG6 真 PWM (待后续)
 */
#include "port.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  引脚硬编码 — 不依赖 SysConfig 生成的引脚宏名,只依赖物理引脚号
 *  SysConfig 已经在 SYSCFG_DL_init() 里配好 PINCM/上下拉/EXTI/NVIC
 * ============================================================ */

/* 方向脚 GPIOA */
#define DIR_LF_PORT    GPIOA
#define DIR_LF_PIN     DL_GPIO_PIN_13   /* PA13 = 左 AIN1 */
#define DIR_LR_PORT    GPIOA
#define DIR_LR_PIN     DL_GPIO_PIN_14   /* PA14 = 左 AIN2 */
#define DIR_RF_PORT    GPIOA
#define DIR_RF_PIN     DL_GPIO_PIN_17   /* PA17 = 右 BIN1 */
#define DIR_RR_PORT    GPIOA
#define DIR_RR_PIN     DL_GPIO_PIN_16   /* PA16 = 右 BIN2 */
#define DIR_PINS_ALL   (DIR_LF_PIN | DIR_LR_PIN | DIR_RF_PIN | DIR_RR_PIN)

/* PWM 脚 GPIOB */
#define PWM_PORT       GPIOB
#define PWM_LF_PIN     DL_GPIO_PIN_2    /* PB2 = 左 PWMA */
#define PWM_RF_PIN     DL_GPIO_PIN_3    /* PB3 = 右 PWMB */
#define PWM_PINS_ALL   (PWM_LF_PIN | PWM_RF_PIN)

/* 编码器 A 相 EXTI */
#define ENC_A_LF_PIN   DL_GPIO_PIN_25   /* PA25 = 左编码器 A 相 */
#define ENC_A_RF_PIN   DL_GPIO_PIN_20   /* PB20 = 右编码器 A 相 */
/* 编码器 B 相输入 */
#define ENC_B_LF_PIN   DL_GPIO_PIN_26   /* PA26 = 左编码器 B 相 */
#define ENC_B_RF_PIN   DL_GPIO_PIN_24   /* PB24 = 右编码器 B 相 */

/* LED + KEY */
#define LED_PORT       GPIOB
#define LED_PIN        DL_GPIO_PIN_9    /* PB9 */
#define KEY_PORT       GPIOB
#define KEY_PIN        DL_GPIO_PIN_8    /* PB8 */

/* UART */
#define UART_DEBUG_INST    UART_0_INST
#define UART_VISION_INST   UART_0_INST

/* SysTick 1kHz,80MHz / 1000 = 80000 cycles */
#define SYSTICK_LOAD_VAL   (80000u - 1u)

/* 主循环外需要的中断处理函数,与 Application/main.c 中定义的回调对接 */
extern void Port_OnTick1kHz(void);
extern void Port_OnTick100Hz(void);

/* ============================================================
 *  1. 平台生命周期
 * ============================================================ */
void Port_HalInit(void)
{
    /* SysConfig 生成的初始化(时钟/GPIO/UART/EXTI/NVIC 全部起来) */
    SYSCFG_DL_init();

    /* SysTick 1kHz — Cortex-M0+ 自带,不依赖 SysConfig Timer 模块 */
    SysTick_Config(SYSTICK_LOAD_VAL);
    NVIC_SetPriority(SysTick_IRQn, 0);  /* 最高优先级,PI 实时性 */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* 使能 UART 中断(Vision RX 用中断接收) */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void Port_Idle(void)
{
    /* 进入低功耗等待中断(WFI),降低主循空闲功耗 */
    __WFI();
}

/* 全局 ms 计数,SysTick 中断里 ++ */
volatile uint32_t g_system_ms = 0;

uint32_t Port_NowMs(void)
{
    return g_system_ms;
}

uint32_t Port_NowUs(void)
{
    /* 粗略:ms * 1000 + SysTick->VAL 反算残余 us */
    uint32_t ms = g_system_ms;
    uint32_t val = SysTick->VAL;
    return ms * 1000u + (SYSTICK_LOAD_VAL - val) / 80u;  /* 80MHz → 80 cycles/us */
}

/* ============================================================
 *  2. 临界区
 * ============================================================ */
uint32_t Port_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void Port_ExitCritical(uint32_t prev)
{
    if (!prev) __enable_irq();
}

/* ============================================================
 *  3. 电机 PWM (Step A:GPIO 软件 PWM)
 *
 *    Step A 实现策略:
 *      pwm > 0  → 方向脚配置正转 + PWM 脚 GPIO 拉高(100% 占空比)
 *      pwm < 0  → 方向脚配置反转 + PWM 脚 GPIO 拉高
 *      pwm == 0 → 方向脚 AIN1=AIN2=0(滑行) + PWM 脚拉低
 *
 *    注意:这只能开环驱动,真 PI 闭环需要 Step B 升级 TIMG6 真 PWM
 *    因为 PI 算出的中间占空比 GPIO 无法表达
 *    但本实现配合 main.c 的 ENC_CLOSED_LOOP_TEST 已经能验证编码器计数
 * ============================================================ */

/* 极性反转(若电机接反,改这里) */
static const int8_t motor_invert[MOTOR_CH_COUNT] = {+1, +1, +1, +1};

void Port_MotorSetPWM(MotorChannel_t ch, int16_t pwm)
{
    /* 应用极性 */
    pwm = (int16_t)(pwm * motor_invert[ch]);

    /* 限幅 (Step A:固定阈值 1,>1 视为 ON) */
    if (pwm >  1) pwm =  1;
    if (pwm < -1) pwm = -1;

    /* LR/RR 是 4 电机模板残留,实际只用 LF + RF */
    /* LR/RR 在 2 电机里其实是 AIN2/BIN2(方向控制副脚),下面 case 处理 */

    uint32_t dir_port_a, dir_pin_a, dir_pin_b;
    uint32_t pwm_pin;

    switch (ch) {
    case MOTOR_LF:
        /* 左轮:AIN1=LF, AIN2=LR */
        dir_port_a = DIR_LF_PORT;
        dir_pin_a = DIR_LF_PIN;
        dir_pin_b = DIR_LR_PIN;
        pwm_pin = PWM_LF_PIN;
        break;
    case MOTOR_RF:
        /* 右轮:BIN1=RF, BIN2=RR */
        dir_port_a = DIR_RF_PORT;
        dir_pin_a = DIR_RF_PIN;
        dir_pin_b = DIR_RR_PIN;
        pwm_pin = PWM_RF_PIN;
        break;
    case MOTOR_LR:
    case MOTOR_RR:
    default:
        /* 2 电机板子不用 LR/RR,直接 no-op */
        return;
    }

    if (pwm > 0) {
        /* 正转:AIN1=1, AIN2=0, PWM=1 */
        DL_GPIO_setPins  (dir_port_a, dir_pin_a);
        DL_GPIO_clearPins(dir_port_a, dir_pin_b);
        DL_GPIO_setPins  (PWM_PORT, pwm_pin);
    } else if (pwm < 0) {
        /* 反转:AIN1=0, AIN2=1, PWM=1 */
        DL_GPIO_clearPins(dir_port_a, dir_pin_a);
        DL_GPIO_setPins  (dir_port_a, dir_pin_b);
        DL_GPIO_setPins  (PWM_PORT, pwm_pin);
    } else {
        /* 停:AIN1=0, AIN2=0, PWM=0 (滑行) */
        DL_GPIO_clearPins(dir_port_a, dir_pin_a | dir_pin_b);
        DL_GPIO_clearPins(PWM_PORT, pwm_pin);
    }
}

int16_t Port_MotorGetPWM(MotorChannel_t ch)
{
    /* Step A:读回 PWM 引脚电平,返回 0 或 1(占空比的离散值) */
    uint32_t pwm_pin;
    switch (ch) {
    case MOTOR_LF: pwm_pin = PWM_LF_PIN; break;
    case MOTOR_RF: pwm_pin = PWM_RF_PIN; break;
    default: return 0;
    }
    bool high = (DL_GPIO_readPins(PWM_PORT, pwm_pin) != 0);
    return high ? 1 : 0;
}

void Port_MotorAllStop(void)
{
    /* 全部方向脚清零 + PWM 拉低 */
    DL_GPIO_clearPins(GPIOA, DIR_PINS_ALL);
    DL_GPIO_clearPins(PWM_PORT, PWM_PINS_ALL);
}

/* ============================================================
 *  4. 编码器 — 软件 EXTI 计数
 *
 *    A 相下降沿触发,瞬间读 B 相电平判方向
 *      B=高 → 正转 (+1)
 *      B=低 → 反转 (-1)
 *    若实测方向反:read_enc_dir_*() 返回值符号翻转
 * ============================================================ */
static int32_t s_enc_count[MOTOR_CH_COUNT];

int32_t Port_EncoderRead(MotorChannel_t ch)
{
    /* 软件累积,读后清零 */
    int32_t v = s_enc_count[ch];
    s_enc_count[ch] = 0;
    return v;
}

void Port_EncoderResetAll(void)
{
    memset(s_enc_count, 0, sizeof(s_enc_count));
}

/* ISR 里调用,把脉冲累积进来 */
void Port_EncoderISR(MotorChannel_t ch, int8_t dir)
{
    s_enc_count[ch] += dir;
}

/* ============================================================
 *  5. 调试串口 — 阻塞发送
 * ============================================================ */
void Port_DebugSend(const uint8_t *data, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) {
        while (DL_UART_isBusy(UART_DEBUG_INST)) {}
        DL_UART_Main_transmitData(UART_DEBUG_INST, data[i]);
    }
}

bool Port_DebugTxReady(void)
{
    return true;
}

void Port_DebugLog(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    extern void Log_Write(int lvl, const char *tag, const char *fmt, ...);
    Log_Write(2, "DBG", "%s", buf);
}

/* ============================================================
 *  6. LED 状态指示
 * ============================================================ */
void Port_LED_Set(bool on)
{
    if (on) DL_GPIO_setPins(LED_PORT, LED_PIN);
    else    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

bool Port_LED_Get(void)
{
    return DL_GPIO_readPins(LED_PORT, LED_PIN) != 0;
}

/* ============================================================
 *  7. OpenMV UART 接收 — 中断里喂给协议解析器
 * ============================================================ */
extern void UART_Vision_Feed(const uint8_t *data, uint16_t len);

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_VISION_INST)) {
    case DL_UART_IIDX_RX: {
        uint8_t b = (uint8_t)DL_UART_Main_receiveData(UART_VISION_INST);
        UART_Vision_Feed(&b, 1);
        break;
    }
    default:
        break;
    }
}

/* ============================================================
 *  8. SysTick 中断 — 1kHz 调度入口
 *     M0+ SysTick 是固定向量名 SysTick_IRQn
 * ============================================================ */
static uint8_t s_tick100_prescaler;

void SysTick_Handler(void)
{
    g_system_ms++;
    Port_OnTick1kHz();
    /* 100Hz 软分频 */
    if (++s_tick100_prescaler >= 10) {
        s_tick100_prescaler = 0;
        Port_OnTick100Hz();
    }
}

/* ============================================================
 *  9. 启动触发(USER 按键 PB8,低有效)
 * ============================================================ */
bool Port_StartTrigger(void)
{
    return (DL_GPIO_readPins(KEY_PORT, KEY_PIN) == 0);
}

/* ============================================================
 *  10. 编码器 EXTI ISR — 2 路 dispatch
 *
 *     SysConfig 把 PA25 配成 GPIOA 下降沿 EXTI → 进 GPIOA_IRQHandler
 *                把 PB20 配成 GPIOB 下降沿 EXTI → 进 GPIOB_IRQHandler
 *     本文件提供这两个函数体
 * ============================================================ */

/* 左编码器方向:读 PA26(B 相) */
static int8_t read_enc_dir_left(void)
{
    return (DL_GPIO_readPins(GPIOA, ENC_B_LF_PIN) != 0) ? +1 : -1;
}
/* 右编码器方向:读 PB24(B 相) */
static int8_t read_enc_dir_right(void)
{
    return (DL_GPIO_readPins(GPIOB, ENC_B_RF_PIN) != 0) ? +1 : -1;
}

/* GPIOA 中断 — PA25 左编码器 A 相下降沿触发 */
void GPIOA_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOA, ENC_A_LF_PIN);
    if (status & ENC_A_LF_PIN) {
        Port_EncoderISR(MOTOR_LF, read_enc_dir_left());
    }
    DL_GPIO_clearInterruptStatus(GPIOA, ENC_A_LF_PIN);
}

/* GPIOB 中断 — PB20 右编码器 A 相 + PB8 USER 按键 */
void GPIOB_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                       ENC_A_RF_PIN | KEY_PIN);
    if (status & ENC_A_RF_PIN) {
        Port_EncoderISR(MOTOR_RF, read_enc_dir_right());
    }
    /* KEY_USER 中断:此处不处理,Port_StartTrigger 用轮询读取 */
    DL_GPIO_clearInterruptStatus(GPIOB, ENC_A_RF_PIN | KEY_PIN);
}
