/**
 * @file tracker.c
 * @brief 循迹外环 — 转向 PD 控制器(50Hz)
 *
 * 数据流:
 *   OpenMV 帧 → Tracker_OnVisionFrame(更新偏差/元素/时戳)
 *   Tracker_Update(50Hz):
 *     1) 超时检测 → 进入 TRACK_STATE_BLIND
 *     2) 偏差低通滤波
 *     3) PD 计算 ω
 *     4) 根据状态/元素决定目标线速度 v
 *     5) Motion_SetTwist(v, ω)
 */
#include "tracker.h"
#include "motion.h"
#include "uart_vision.h"
#include "errcode.h"
#include "log.h"
#include "config.h"
#include "port.h"
#include <string.h>

/* 内部状态 */
static int16_t         s_err_raw;        /* 最新一帧的原始偏差(像素) */
static int16_t         s_err_filtered;   /* 滤波后偏差 */
static TrackElement_t  s_element;
static uint32_t        s_last_frame_ms;
static TrackState_t    s_state;

void Tracker_Init(void)
{
    s_err_raw      = 0;
    s_err_filtered = 0;
    s_element      = ELEM_UNKNOWN;
    s_last_frame_ms = Port_NowMs();
    s_state        = TRACK_STATE_OK;
    LOG_I("TRACK", "init ok, Kp=%.3f Kd=%.3f omega_max=%.1f",
          TRACK_PD_KP, TRACK_PD_KD, TRACK_PD_OUT_MAX);
}

/* OpenMV 帧到达 — 由 port 层桥接调用 */
void Tracker_OnVisionFrame(const uint8_t *frame, uint16_t len)
{
    if (len < VISION_FRAME_LEN || !frame) return;
    /* 字节 2-3: int16 小端 = err */
    int16_t err = (int16_t)((uint16_t)frame[2] | ((uint16_t)frame[3] << 8));
    /* 字节 4: element */
    uint8_t elem = frame[4];

    /* 元素变化时打日志 */
    static TrackElement_t s_last_logged_elem = ELEM_UNKNOWN + 100;
    if ((TrackElement_t)elem != s_last_logged_elem) {
        LOG_I("TRACK", "elem: %d->%d (err=%d)", (int)s_last_logged_elem, (int)elem, (int)err);
        s_last_logged_elem = (TrackElement_t)elem;
    }

    s_err_raw       = err;
    s_element       = (TrackElement_t)elem;
    s_last_frame_ms = Port_NowMs();
}

static int16_t select_speed_for_element(TrackElement_t e)
{
    switch (e) {
        case ELEM_STRAIGHT:   return SPEED_TARGET_TRACKING;
        case ELEM_LEFT:
        case ELEM_RIGHT:      return SPEED_TARGET_CURVE;
        case ELEM_CROSS:      return SPEED_TARGET_CROSS;
        case ELEM_ROUNDABOUT: return SPEED_TARGET_ROUNDABOUT;
        case ELEM_SLOPE:      return SPEED_TARGET_SLOPE;
        case ELEM_STOP_LINE:  return 0;     /* 停车 */
        default:              return SPEED_TARGET_TRACKING;
    }
}

void Tracker_Update(void)
{
    uint32_t now = Port_NowMs();
    TrackState_t prev_state = s_state;

    /* 1) 通信超时检测 */
    if (now - s_last_frame_ms > TICK_VISION_TIMEOUT_MS) {
        s_state = TRACK_STATE_BLIND;
        if (prev_state != TRACK_STATE_BLIND) {
            Err_Report(ERR_VISION_TIMEOUT);
            LOG_E("TRACK", "BLIND: no frame for %lums, braking",
                  (unsigned long)(now - s_last_frame_ms));
        }
        Motion_Brake();
        return;
    }

    /* 2) 丢线检测(ERR = 0x7FFF) */
    if (s_err_raw == 0x7FFF) {
        s_state = TRACK_STATE_LOST;
        if (prev_state != TRACK_STATE_LOST) {
            Err_Report(ERR_VISION_LOST_LINE);
            LOG_W("TRACK", "LOST line (err=0x7FFF), crawling");
        }
        /* 慢速直行一点点尝试重新捕获 */
        Motion_SetTwist(SPEED_TARGET_LOST, 0.0f);
        return;
    }

    /* 恢复边沿 */
    if (prev_state != TRACK_STATE_OK) {
        LOG_I("TRACK", "recovered: %d->OK", (int)prev_state);
    }
    s_state = TRACK_STATE_OK;

    /* 3) 偏差低通滤波: y = (1-α)*y + α*x */
    {
        float a = TRACK_E_FILTER_ALPHA;
        float x = (float)s_err_raw;
        float y = (1.0f - a) * (float)s_err_filtered + a * x;
        s_err_filtered = (int16_t)y;
    }

    /* 4) 转向 PD(基于滤波后偏差) */
    static int16_t s_last_err_filt;
    float e  = (float)s_err_filtered;
    float de = e - (float)s_last_err_filt;
    float omega = TRACK_PD_KP * e + TRACK_PD_KD * de;
    /* ω 限幅 */
    if (omega >  TRACK_PD_OUT_MAX) omega =  TRACK_PD_OUT_MAX;
    if (omega < -TRACK_PD_OUT_MAX) omega = -TRACK_PD_OUT_MAX;
    s_last_err_filt = s_err_filtered;

    /* 5) 根据元素选速度 */
    int16_t v = select_speed_for_element(s_element);

    /* 6) 输出 */
    Motion_SetTwist((float)v, omega);
}

TrackElement_t Tracker_GetElement(void)  { return s_element; }
int16_t        Tracker_GetLastError(void){ return s_err_raw; }
TrackState_t   Tracker_GetState(void)    { return s_state; }
