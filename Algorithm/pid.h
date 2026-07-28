/**
 * @file pid.h
 * @brief 通用 PID 控制器(位置式与增量式可选,带抗饱和与输出限幅)
 *
 * 与硬件无关,可在 PC 上做单元测试。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PID_MODE_POSITIONAL = 0,   /* 位置式(全量输出)   */
    PID_MODE_INCREMENTAL = 1,  /* 增量式(输出增量)   */
} PidMode_t;

typedef struct {
    PidMode_t mode;
    float    Kp, Ki, Kd;
    float    out_min, out_max;     /* 输出限幅          */
    float    integral_max;         /* 积分项限幅(位置式) */
    /* 内部状态 — 不要在外部修改 */
    float    integral;
    float    last_err;
    float    last_out;
} Pid_t;

void  Pid_Init(Pid_t *pid, PidMode_t mode,
               float kp, float ki, float kd,
               float out_min, float out_max);
float Pid_Calc(Pid_t *pid, float setpoint, float measured);
void  Pid_Reset(Pid_t *pid);

#endif /* PID_H */
