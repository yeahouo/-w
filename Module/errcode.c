/**
 * @file errcode.c
 * @brief 错误码系统实现 — 简单计数与时戳
 */
#include "errcode.h"
#include "port.h"

static volatile ErrCode_t s_last_err;
static volatile uint32_t  s_last_err_ms;
static volatile uint32_t  s_total;
static volatile uint32_t  s_counts[ERR_RESERVED + 1];

void Err_Init(void)
{
    s_last_err    = ERR_OK;
    s_last_err_ms = 0;
    s_total       = 0;
    for (int i = 0; i <= (int)ERR_RESERVED; ++i) s_counts[i] = 0;
}

void Err_Report(ErrCode_t e)
{
    if (e == ERR_OK) return;
    if (e > ERR_RESERVED) e = ERR_UNKNOWN;

    s_last_err    = e;
    s_last_err_ms = Port_NowMs();
    s_counts[(int)e]++;
    s_total++;
}

ErrCode_t Err_GetLast(void)     { return s_last_err; }
uint32_t  Err_GetLastMs(void)   { return s_last_err_ms; }
uint32_t  Err_GetTotalCount(void) { return s_total; }

uint32_t  Err_GetCount(ErrCode_t e)
{
    if ((int)e < 0 || (int)e > (int)ERR_RESERVED) return 0;
    return s_counts[(int)e];
}

void Err_ResetCount(ErrCode_t e)
{
    if ((int)e >= 0 && (int)e <= (int)ERR_RESERVED) s_counts[(int)e] = 0;
}

const char *Err_Name(ErrCode_t e)
{
    switch (e) {
    case ERR_OK:                return "OK";
    case ERR_VISION_TIMEOUT:    return "VISION_TIMEOUT";
    case ERR_VISION_CRC_FAIL:   return "VISION_CRC_FAIL";
    case ERR_VISION_NO_HDR:     return "VISION_NO_HDR";
    case ERR_VISION_LOST_LINE:  return "VISION_LOST_LINE";
    case ERR_MOTOR_STALL:       return "MOTOR_STALL";
    case ERR_MOTOR_OVERCURRENT: return "MOTOR_OVERCURRENT";
    case ERR_PI_DIVERGE:        return "PI_DIVERGE";
    case ERR_BATTERY_LOW:       return "BATTERY_LOW";
    case ERR_TASK_OVERRUN:      return "TASK_OVERRUN";
    case ERR_UNKNOWN:           return "UNKNOWN";
    default:                    return "?";
    }
}
