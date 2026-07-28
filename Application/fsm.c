/**
 * @file fsm.c
 * @brief 主状态机 — IDLE / START / TRACKING / ELEMENT / FINISH / ERROR
 *
 * 当前为最简骨架:
 *   IDLE     → 按键/启动信号 → START
 *   START    → 加速 1 秒      → TRACKING
 *   TRACKING → 元素非 STRAIGHT/UNKNOWN → ELEMENT
 *   ELEMENT  → 元素回到 STRAIGHT → TRACKING
 *   TRACKING → 通信故障/丢线超时 → ERROR
 *   ERROR    → 恢复  → TRACKING
 *   任意     → 收到 STOP_LINE → FINISH
 *
 * 元素子状态机(CROSS/ROUNDABOUT/SLOPE 的多步骤)留给后续扩展。
 */
#include "fsm.h"
#include "tracker.h"
#include "motion.h"
#include "motor.h"
#include "uart_vision.h"
#include "errcode.h"
#include "log.h"
#include "port.h"
#include "config.h"
#include <stdio.h>

static FsmState_t s_state;
static uint32_t   s_state_enter_ms;

/* 启动加速计时 */
static uint32_t   s_start_begin_ms;
#define START_RAMP_MS  (1000)

/* ERROR 状态恢复计时 */
static uint32_t   s_error_begin_ms;
#define ERROR_AUTO_RECOVER_MS (500)

/* !! 临时自动启动 — boot 后 N ms 仍在 IDLE 就强制进 START
   Stage 5 加 USER 按键后,改成 0 即可禁用自动启动(只走 Port_StartTrigger)
   保留 Port_StartTrigger 检测,主人加按键时按一下立刻启动,不必等满 */
#define FSM_AUTO_START_MS      (2000)

void FSM_Init(void)
{
    s_state          = FSM_STATE_IDLE;
    s_state_enter_ms = Port_NowMs();
    Motion_Brake();
    LOG_I("FSM", "init -> IDLE");
}

static void enter_state(FsmState_t next)
{
    if (next == s_state) return;
    LOG_I("FSM", "%s -> %s", FSM_StateName(s_state), FSM_StateName(next));
    s_state          = next;
    s_state_enter_ms = Port_NowMs();

    if (next == FSM_STATE_START)  s_start_begin_ms = s_state_enter_ms;
    if (next == FSM_STATE_ERROR) {
        s_error_begin_ms = s_state_enter_ms;
        Err_Report(ERR_UNKNOWN);   /* 上层知道进了 ERROR,具体原因看其他错误码 */
        LOG_E("FSM", "enter ERROR state");
    }
    if (next == FSM_STATE_IDLE || next == FSM_STATE_FINISH || next == FSM_STATE_ERROR) {
        Motion_Brake();
    }
}

const char *FSM_StateName(FsmState_t s)
{
    switch (s) {
        case FSM_STATE_IDLE:     return "IDLE";
        case FSM_STATE_START:    return "START";
        case FSM_STATE_TRACKING: return "TRACKING";
        case FSM_STATE_ELEMENT:  return "ELEMENT";
        case FSM_STATE_FINISH:   return "FINISH";
        case FSM_STATE_ERROR:    return "ERROR";
        default:                 return "?";
    }
}

FsmState_t FSM_GetState(void) { return s_state; }

/* 启动信号源:Port 层提供 Port_StartTrigger() 读取 USER 按键
   fsm.c 这里直接调用,具体实现在 BSP/port_impl.c */

void FSM_Step(void)
{
    uint32_t now = Port_NowMs();
    TrackElement_t elem = Tracker_GetElement();
    TrackState_t   ts   = Tracker_GetState();

    /* 任何状态下收到 STOP_LINE 都进 FINISH */
    if (elem == ELEM_STOP_LINE && s_state != FSM_STATE_IDLE) {
        enter_state(FSM_STATE_FINISH);
        return;
    }

    switch (s_state) {

    case FSM_STATE_IDLE:
        Motion_Brake();
        /* 启动信号优先级:USER 按键 → 自动启动(超时) */
        if (Port_StartTrigger()) {
            enter_state(FSM_STATE_START);
        } else if (FSM_AUTO_START_MS > 0 &&
                   (now - s_state_enter_ms) >= FSM_AUTO_START_MS) {
            /* !! 临时:boot 后 FSM_AUTO_START_MS 仍没按键就自动启动
               Stage 5 接 USER 按键后,把 FSM_AUTO_START_MS 改 0 即禁用 */
            LOG_I("FSM", "auto-start after %d ms (no USER key)", FSM_AUTO_START_MS);
            enter_state(FSM_STATE_START);
        }
        break;

    case FSM_STATE_START:
        /* 短暂低速直行,帮助建速 */
        Motion_SetTwist((float)SPEED_TARGET_TRACKING * 0.5f, 0.0f);
        if (now - s_start_begin_ms >= START_RAMP_MS) {
            enter_state(FSM_STATE_TRACKING);
        }
        break;

    case FSM_STATE_TRACKING:
        /* tracker.c 已经在 Tracker_Update 里跑 PD + Motion_SetTwist */
        if (ts == TRACK_STATE_BLIND) {
            enter_state(FSM_STATE_ERROR);
        } else if (elem != ELEM_STRAIGHT && elem != ELEM_UNKNOWN) {
            enter_state(FSM_STATE_ELEMENT);
        }
        break;

    case FSM_STATE_ELEMENT:
        /* 元素状态:tracker 仍会按元素类型选不同速度
           简化策略:元素回到 STRAIGHT 即退出 */
        if (ts == TRACK_STATE_BLIND) {
            enter_state(FSM_STATE_ERROR);
        } else if (elem == ELEM_STRAIGHT || elem == ELEM_UNKNOWN) {
            enter_state(FSM_STATE_TRACKING);
        }
        break;

    case FSM_STATE_ERROR:
        Motion_Brake();
        /* 简单超时自动恢复(可选) */
        if (now - s_error_begin_ms >= ERROR_AUTO_RECOVER_MS) {
            if (ts != TRACK_STATE_BLIND) {
                enter_state(FSM_STATE_TRACKING);
            }
        }
        break;

    case FSM_STATE_FINISH:
        Motion_Brake();
        break;
    }
}

void FSM_DebugDump(void)
{
    LOG_I("FSM", "state=%s elem=%d err=%d tstate=%d lastErr=%s(%lu) vframes=%lu vdrops=%lu dropped=%lu",
          FSM_StateName(s_state),
          (int)Tracker_GetElement(),
          (int)Tracker_GetLastError(),
          (int)Tracker_GetState(),
          Err_Name(Err_GetLast()),
          (unsigned long)Err_GetTotalCount(),
          (unsigned long)UART_Vision_GetFrameCount(),
          (unsigned long)UART_Vision_GetDropCount(),
          (unsigned long)Log_GetDroppedCount());
}
