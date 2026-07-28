/**
 * @file fsm.h
 * @brief 主状态机 — IDLE / START / TRACKING / ELEMENT / FINISH / ERROR
 *
 * 每个状态有自己的速度上限与 PD 参数,元素状态内嵌子状态机。
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef enum {
    FSM_STATE_IDLE    = 0,
    FSM_STATE_START   = 1,
    FSM_STATE_TRACKING= 2,
    FSM_STATE_ELEMENT = 3,
    FSM_STATE_FINISH  = 4,
    FSM_STATE_ERROR   = 5,
} FsmState_t;

void      FSM_Init(void);
void      FSM_Step(void);              /* 100Hz,主循环调用 */
FsmState_t FSM_GetState(void);
const char *FSM_StateName(FsmState_t s);

/* 调试输出 */
void      FSM_DebugDump(void);

/* 元素子状态(给 ELEMENT 状态用) */
typedef enum {
    ELEM_SUB_NONE = 0,
    /* 十字:进入→通过→离开 */
    ELEM_SUB_CROSS_ENTER, ELEM_SUB_CROSS_PASS, ELEM_SUB_CROSS_LEAVE,
    /* 环岛:入口→环内→出口 */
    ELEM_SUB_RB_ENTER, ELEM_SUB_RB_INSIDE, ELEM_SUB_RB_EXIT,
    /* 坡道:上坡→坡顶→下坡 */
    ELEM_SUB_SLOPE_UP, ELEM_SUB_SLOPE_TOP, ELEM_SUB_SLOPE_DOWN,
} ElemSubState_t;

#endif /* FSM_H */
