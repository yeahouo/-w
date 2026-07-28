/**
 * @file port_min.c
 * @brief Port 接口最小可工作实现 — 阶段 4 首次烧录看日志用
 *
 * 只实现"真能工作"的三件事:
 *   - UART0 460800 TX 阻塞发送  → 主人能在串口看到 BOOT 日志
 *   - LED1 (PB22) 操作           → 主人能看到 LED 慢闪
 *   - SysTick 1ms 中断           → 提供 ms 时钟 + 驱动 1kHz/100Hz tick
 *
 * 空实现(后续 SysConfig 扩展后切到 port_impl.c):
 *   - Port_MotorSetPWM/GetPWM       (不驱动电机)
 *   - Port_EncoderRead/ResetAll/ISR (编码器不读)
 *   - Port_StartTrigger             (USER 按键没配)
 *   - Port_VisionRegisterCb         (OpenMV 没接, UART RX 收到字节丢弃)
 *
 * 主人烧录后预期看到:
 *   1. 串口输出 BOOT 日志(0.5 秒内出现)
 *   2. LED 慢闪(IDLE 状态,约 1Hz)
 *   3. 状态机停在 IDLE(因为 StartTrigger 永远 false)
 */

/* LINE_FOLLOW 单文件架构下:
   - main.c 自己定义了 g_system_ms / UART0_IRQHandler / SysTick_Handler
   - port_min.c 只保留 Port_* stub,让 Module 层(fsm/motor/encoder/log/led_status)
     链接通过。这些 Module 文件是死代码(line_follow main.c 不调用),但
     Keil 工程文件保留了它们,所以需要 port_min.c 提供 Port_* 符号。
   - 想回归 FSM 架构时,把 main.c 换回完整 FSM 版本,这里恢复 g_system_ms、
     UART0_IRQHandler、SysTick_Handler 即可。 */
#include "port.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>

/* ---------- 全局状态 ---------- */
/* g_system_ms 由 Application/main.c(line_follow v5)定义,这里 extern 引用 */
extern volatile uint32_t g_system_ms;

/* 外部回调(main.c 定义) */
extern void Port_OnTick1kHz(void);
extern void Port_OnTick100Hz(void);

/* ============================================================
 *  1. 平台生命周期
 * ============================================================ */
void Port_HalInit(void)
{
    /* SysConfig 生成的初始化(时钟 80MHz + UART0 + LED GPIO) */
    SYSCFG_DL_init();

    /* 配 SysTick 1ms 中断(80MHz / 80000 = 1kHz) */
    SysTick_Config(80000);

    /* 使能 UART0 中断(为 OpenMV RX 预留,port_min 里 RX 字节直接丢弃) */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void Port_Idle(void)
{
    __WFI();
}

uint32_t Port_NowMs(void)
{
    return g_system_ms;
}

uint32_t Port_NowUs(void)
{
    return g_system_ms * 1000u;
}

/* ============================================================
 *  2. 临界区(保护环形 buffer 多上下文)
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
 *  3. 电机 PWM(空 — 阶段 4 不驱动电机)
 * ============================================================ */
void Port_MotorSetPWM(MotorChannel_t ch, int16_t pwm)
{
    (void)ch; (void)pwm;
}

int16_t Port_MotorGetPWM(MotorChannel_t ch)
{
    (void)ch;
    return 0;
}

/* ============================================================
 *  4. 编码器(空 — 阶段 4 不读编码器)
 * ============================================================ */
int32_t Port_EncoderRead(MotorChannel_t ch)
{
    (void)ch;
    return 0;
}

void Port_EncoderResetAll(void)
{
}

void Port_EncoderISR(MotorChannel_t ch, int8_t dir)
{
    (void)ch; (void)dir;
}

/* ============================================================
 *  5. 调试串口 — UART0 阻塞发送
 * ============================================================ */
void Port_DebugSend(const uint8_t *data, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) {
        while (DL_UART_isBusy(UART_0_INST)) {}
        DL_UART_Main_transmitData(UART_0_INST, data[i]);
    }
}

bool Port_DebugTxReady(void)
{
    return true;  /* 阻塞模式,任何时候都"就绪" */
}

void Port_DebugLog(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    extern void Log_Write(int lvl, const char *tag, const char *fmt, ...);
    Log_Write(2 /*LOG_LEVEL_INFO*/, "DBG", "%s", buf);
}

/* ============================================================
 *  6. LED — PB22(天猛星板载 LED,低电平有效)
 * ============================================================ */
void Port_LED_Set(bool on)
{
    /* 板载 LED 阴极接 GPIO(默认 SET=高=熄灭)
       on=true  → clearPins(拉低) → 点亮
       on=false → setPins(拉高)   → 熄灭 */
    if (on) DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
    else    DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
}

bool Port_LED_Get(void)
{
    /* 读到的位为 0 表示当前输出低 → LED 点亮 */
    return (DL_GPIO_readPins(LED1_PORT, LED1_PIN_22_PIN) == 0);
}

/* ============================================================
 *  7. 视觉 RX / UART0 中断 / SysTick 中断
 *
 *  这三个符号(g_system_ms / UART0_IRQHandler / SysTick_Handler)
 *  都由 Application/main.c (line_follow v5) 独占定义。
 *  port_min.c 这里不再提供,避免链接器 multiply defined 错误。
 * ============================================================ */

/* ============================================================
 *  8. 启动按键(空 — 阶段 4 没配 USER 按键 GPIO)
 *    FSM 会永远停在 IDLE(没启动信号)
 * ============================================================ */
bool Port_StartTrigger(void)
{
    return false;
}
