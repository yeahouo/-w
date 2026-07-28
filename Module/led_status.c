/**
 * @file led_status.c
 * @brief 板载 LED 状态指示实现 — 16-bit pattern,每 bit = 100ms
 */
#include "led_status.h"
#include "port.h"
#include "config.h"

/* pattern: bit i = 第 i 个 100ms 时段 LED 是否亮(从最低位开始)
   每个周期 16 个 bit = 1.6 秒 */
#define PAT_IDLE      (0x000F)   /* _ _ _ _ * * * *  → 0.4s 灭 / 0.4s 亮,慢闪近似 1Hz */
#define PAT_START     (0x3333)   /* 0011 0011 ...  5Hz 快闪 */
#define PAT_TRACKING  (0xFFFF)   /* 全亮 */
#define PAT_ELEMENT   (0x8009)   /* bit0,3,15 亮 — 模拟双闪(简化的视觉效果) */
#define PAT_FINISH    (0xFFFF)   /* 长亮(配合时间逻辑) */
#define PAT_ERROR     (0xAAAA)   /* 1010...  10Hz 急闪 */

static uint8_t  s_phase;          /* 0~15 循环 */
static uint32_t s_state_change_ms;
static FsmState_t s_last_fsm;
static bool     s_override_active;
static bool     s_override_value;

void LED_Status_Init(void)
{
    s_phase           = 0;
    s_last_fsm        = FSM_STATE_IDLE;
    s_state_change_ms = Port_NowMs();
    s_override_active = false;
    Port_LED_Set(false);
}

static uint16_t pattern_for(FsmState_t s)
{
    switch (s) {
    case FSM_STATE_IDLE:     return PAT_IDLE;
    case FSM_STATE_START:    return PAT_START;
    case FSM_STATE_TRACKING: return PAT_TRACKING;
    case FSM_STATE_ELEMENT:  return PAT_ELEMENT;
    case FSM_STATE_ERROR:    return PAT_ERROR;
    case FSM_STATE_FINISH:   return PAT_FINISH;
    default:                 return PAT_IDLE;
    }
}

void LED_Status_Tick(void)
{
    /* 100Hz 调用,每 10 个 tick 推进一帧(每 pattern bit = 100ms) */
    static uint8_t prescaler = 0;
    if (++prescaler < 10) return;
    prescaler = 0;

    s_phase = (s_phase + 1) & 0x0F;   /* 0~15 循环 */

    /* 检测 FSM 状态变化(用于 FINISH 的 2 秒后灭) */
    FsmState_t cur = FSM_GetState();
    if (cur != s_last_fsm) {
        s_last_fsm        = cur;
        s_state_change_ms = Port_NowMs();
    }

    if (s_override_active) {
        Port_LED_Set(s_override_value);
        return;
    }

    uint16_t pat = pattern_for(cur);

    /* FINISH: 2 秒后熄灭 */
    if (cur == FSM_STATE_FINISH) {
        if (Port_NowMs() - s_state_change_ms > 2000) {
            Port_LED_Set(false);
            return;
        }
    }

    bool on = (pat >> s_phase) & 0x0001;
    Port_LED_Set(on);
}

void LED_Status_Override(bool on)
{
    s_override_active = true;
    s_override_value  = on;
    Port_LED_Set(on);
}

void LED_Status_Release(void)
{
    s_override_active = false;
}
