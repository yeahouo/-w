/**
 * @file telemetry.c
 * @brief 实时遥测实现 — 100Hz 采样快照,50Hz CSV 输出
 *
 * 数据流:
 *   各模块暴露 getter ──> Telemetry_Capture(100Hz)写本地快照
 *                                  │
 *                                  ↓
 *                  Telemetry_Poll(50Hz)拼 CSV ──> Port_DebugSend
 */
#include "telemetry.h"
#include "motor.h"
#include "encoder.h"
#include "tracker.h"
#include "uart_vision.h"
#include "errcode.h"
#include "fsm.h"
#include "config.h"
#include "port.h"
#include <stdio.h>
#include <string.h>

/* 100Hz 采样的本地快照(避免输出帧过程中被新数据覆盖) */
typedef struct {
    uint32_t ms;
    int16_t  err;
    uint8_t  elem;
    uint8_t  track_state;
    uint8_t  last_err;
    uint32_t vframes;
    uint32_t vdrops;
    int16_t  target[MOTOR_CH_COUNT];
    int16_t  actual[MOTOR_CH_COUNT];
    int16_t  pwm[MOTOR_CH_COUNT];
} Snapshot_t;

static Snapshot_t s_snap;
static uint8_t    s_rate_hz;
static uint32_t   s_last_send_ms;
static TelFmt_t   s_fmt;

/* 输出 buffer(够装一行 CSV) */
static char s_outbuf[200];

void Telemetry_Init(void)
{
    memset(&s_snap, 0, sizeof(s_snap));
    s_rate_hz     = 50;
    s_last_send_ms= 0;
    s_fmt         = TEL_FMT_CSV;
}

void Telemetry_Capture(void)
{
    s_snap.ms          = Port_NowMs();
    s_snap.err         = Tracker_GetLastError();
    s_snap.elem        = (uint8_t)Tracker_GetElement();
    s_snap.track_state = (uint8_t)Tracker_GetState();
    s_snap.last_err    = (uint8_t)Err_GetLast();
    s_snap.vframes     = UART_Vision_GetFrameCount();
    s_snap.vdrops      = UART_Vision_GetDropCount();

    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        MotorChannel_t ch = (MotorChannel_t)i;
        s_snap.target[i] = Motor_GetTargetSpeed(ch);
        s_snap.actual[i] = (int16_t)Encoder_GetSpeed(ch);
        s_snap.pwm[i]    = Motor_GetOutputPWM(ch);
    }
}

void Telemetry_Poll(void)
{
    if (s_fmt == TEL_FMT_OFF) return;
    if (s_rate_hz == 0) return;

    uint32_t now = Port_NowMs();
    uint32_t period = 1000u / s_rate_hz;
    if ((now - s_last_send_ms) < period) return;
    s_last_send_ms = now;

    /* 头注释 — 每秒打一次,便于上位机识别字段
       (注释行以 # 开头,SerialPlot/Serial Studio 支持) */
    static uint32_t s_last_header_ms;
    if (now - s_last_header_ms >= 1000) {
        static const char header[] =
            "# ms,err,elem,state,last_err,vframes,vdrops,"
            "tlf,tlr,trf,trr,alf,alr,arf,arr,plf,plr,prf,prr\r\n";
        Port_DebugSend((const uint8_t *)header, sizeof(header) - 1);
        s_last_header_ms = now;
    }

    int n = snprintf(s_outbuf, sizeof(s_outbuf),
        "%lu,%d,%u,%u,%u,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        (unsigned long)s_snap.ms,
        (int)s_snap.err,
        (unsigned)s_snap.elem,
        (unsigned)s_snap.track_state,
        (unsigned)s_snap.last_err,
        (unsigned long)s_snap.vframes,
        (unsigned long)s_snap.vdrops,
        (int)s_snap.target[MOTOR_LF], (int)s_snap.target[MOTOR_LR],
        (int)s_snap.target[MOTOR_RF], (int)s_snap.target[MOTOR_RR],
        (int)s_snap.actual[MOTOR_LF], (int)s_snap.actual[MOTOR_LR],
        (int)s_snap.actual[MOTOR_RF], (int)s_snap.actual[MOTOR_RR],
        (int)s_snap.pwm[MOTOR_LF],    (int)s_snap.pwm[MOTOR_LR],
        (int)s_snap.pwm[MOTOR_RF],    (int)s_snap.pwm[MOTOR_RR]
    );

    if (n > 0 && (uint32_t)n < sizeof(s_outbuf)) {
        Port_DebugSend((const uint8_t *)s_outbuf, (uint16_t)n);
    }
}

void Telemetry_SetFmt(TelFmt_t f)   { s_fmt = f; }
void Telemetry_SetRate(uint8_t hz)
{
    if (hz > 100) hz = 100;
    if (hz < 1 && hz != 0) hz = 1;
    s_rate_hz = hz;
}
