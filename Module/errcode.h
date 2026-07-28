/**
 * @file errcode.h
 * @brief 错误码系统 — 分类上报 + 累计计数 + 最后错误时戳
 *
 * 使用:
 *   Err_Report(ERR_VISION_CRC_FAIL);   // 上报一次
 *   ErrCode_t e = Err_GetLast();       // 查最近错误
 *   uint32_t n = Err_GetCount(ERR_VISION_TIMEOUT);  // 查某类错误累计次数
 */
#ifndef ERRCODE_H
#define ERRCODE_H

#include <stdint.h>

typedef enum {
    ERR_OK                = 0,
    ERR_VISION_TIMEOUT    = 1,   /* OpenMV 200ms 没来帧 */
    ERR_VISION_CRC_FAIL   = 2,   /* XOR 校验失败 */
    ERR_VISION_NO_HDR     = 3,   /* 长时间未对齐帧头 */
    ERR_VISION_LOST_LINE  = 4,   /* ERR 字段 = 0x7FFF 丢线 */
    ERR_MOTOR_STALL       = 5,   /* 电机堵转(目标≠0 但实测≈0 持续 N ms) */
    ERR_MOTOR_OVERCURRENT = 6,   /* 过流(如有电流采样) */
    ERR_PI_DIVERGE        = 7,   /* PI 输出长时间打满,可能模型失稳 */
    ERR_BATTERY_LOW       = 8,   /* 电池电压过低 */
    ERR_TASK_OVERRUN      = 9,   /* 周期任务超时(CPU 跑不过来) */
    ERR_UNKNOWN           = 0xFE,
    ERR_RESERVED          = 0xFF,
} ErrCode_t;

void      Err_Init(void);
void      Err_Report(ErrCode_t e);
ErrCode_t Err_GetLast(void);
uint32_t  Err_GetLastMs(void);             /* 最近一次错误的发生时戳 */
uint32_t  Err_GetCount(ErrCode_t e);       /* 某类错误累计次数 */
uint32_t  Err_GetTotalCount(void);         /* 总错误次数 */
void      Err_ResetCount(ErrCode_t e);

const char *Err_Name(ErrCode_t e);         /* 错误码字符串名,供日志用 */

#endif /* ERRCODE_H */
