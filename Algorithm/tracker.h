/**
 * @file tracker.h
 * @brief 循迹外环 — 转向 PD 控制器(50Hz)
 *
 * 数据流:
 *   OpenMV 帧 → Tracker_OnVisionFrame(更新内部偏差/元素缓冲)
 *              → Tracker_Update(50Hz,跑 PD,输出 (v,ω) 给 Motion)
 */
#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef enum {
    TRACK_STATE_OK    = 0,   /* 正常循迹          */
    TRACK_STATE_LOST  = 1,   /* 丢线              */
    TRACK_STATE_BLIND = 2,   /* 通信故障(200ms 无帧) */
} TrackState_t;

void TrackState_GetState(TrackState_t *state);

/* 初始化 */
void Tracker_Init(void);

/**
 * @brief OpenMV 帧到达回调(由 port 层注册并调用)
 * @param frame 11 字节已校验帧
 * @param len   帧长度(应 == 11)
 */
void Tracker_OnVisionFrame(const uint8_t *frame, uint16_t len);

/**
 * @brief 转向 PD 周期任务(50Hz,在主循环中调用)
 *   内部读最新偏差 → 跑 PD → 调用 Motion_SetTwist
 */
void Tracker_Update(void);

/* 给状态机查询当前元素 */
TrackElement_t Tracker_GetElement(void);
int16_t         Tracker_GetLastError(void);    /* 像素 */
TrackState_t    Tracker_GetState(void);

#endif /* TRACKER_H */
