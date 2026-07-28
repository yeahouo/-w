/**
 * @file button.c
 * @brief USER 按键驱动实现 — PB8, 低电平有效, 时间戳去抖
 *
 * 引脚 (PINCM 已查 TI SysConfig deviceData MSPM0G350X.json 核实):
 *   USER 按键 = PB8 = IOMUX_PINCM25, 内部上拉, 低电平有效
 */

#include "ti_msp_dl_config.h"
#include "button.h"

/* ============================================================
 *  引脚定义
 * ============================================================ */
#define BUTTON_PORT            GPIOB
#define BUTTON_PIN             DL_GPIO_PIN_8
#define BUTTON_PINCM           IOMUX_PINCM25

/* 按键有效电平: 0 = 按下时读到低 (常见接法, 一端接 GND);
 *              1 = 按下时读到高。实测反了改这里。 */
#define BUTTON_ACTIVE_LEVEL    (0)

/* 去抖时间 (ms): 原始电平持续稳定至此才采纳为新状态 */
#define BUTTON_DEBOUNCE_MS     (20)

/* ============================================================
 *  ms 时钟 — 复用 main.c 的 SysTick 毫秒时钟
 * ============================================================ */
extern volatile uint32_t g_system_ms;

/* ============================================================
 *  内部状态
 * ============================================================ */
static bool     s_db       = false;   /* 去抖后状态 (true=按下) */
static bool     s_raw_prev = false;   /* 上一次原始读数 */
static uint32_t s_change   = 0;       /* 原始电平开始变化的时间戳 */
static bool     s_event    = false;   /* 待消费的"新按下"事件 */

/* 读原始电平, 归一化为 true=按下 */
static bool button_read_raw(void)
{
    bool high = (DL_GPIO_readPins(BUTTON_PORT, BUTTON_PIN) != 0);
    return (BUTTON_ACTIVE_LEVEL == 0) ? (bool)!high : high;
}

/* ============================================================
 *  公开 API
 * ============================================================ */

void Button_Init(void)
{
    /* PB8: 数字输入 + 内部上拉 (与 main.c 灰度传感器引脚同一 API) */
    DL_GPIO_initDigitalInputFeatures(BUTTON_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 建立去抖基线, 避免上电瞬间因 raw != s_db 误触发一次按下事件 */
    bool raw = button_read_raw();
    s_db       = raw;
    s_raw_prev = raw;
    s_change   = 0;
    s_event    = false;
}

bool Button_IsPressed(void)
{
    bool raw = button_read_raw();
    uint32_t now = g_system_ms;

    /* 原始电平翻转 → 记录变化时刻 */
    if (raw != s_raw_prev) {
        s_raw_prev = raw;
        s_change   = now;
    }

    /* 原始电平与去抖状态不一致, 且已稳定 >= 去抖时间 → 采纳 */
    if (raw != s_db && (now - s_change) >= BUTTON_DEBOUNCE_MS) {
        s_db = raw;
        if (raw) s_event = true;   /* 一次"新按下"产生事件 */
    }
    return s_db;
}

bool Button_Consume(void)
{
    Button_IsPressed();          /* 先刷新去抖状态 */
    bool e = s_event;
    s_event = false;
    return e;
}
