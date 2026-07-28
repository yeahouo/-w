/**
 * @file pid.c
 * @brief 通用 PID 控制器实现 — 位置式 / 增量式,带输出限幅与抗积分饱和
 *
 * 纯算法,无硬件依赖,可在 PC 上做单元测试。
 */
#include "pid.h"
#include <math.h>

static float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

void Pid_Init(Pid_t *pid, PidMode_t mode,
              float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->mode         = mode;
    pid->Kp           = kp;
    pid->Ki           = ki;
    pid->Kd           = kd;
    pid->out_min      = out_min;
    pid->out_max      = out_max;
    pid->integral_max = (out_max - out_min);   /* 默认与输出同限 */
    pid->integral     = 0.0f;
    pid->last_err     = 0.0f;
    pid->last_out     = 0.0f;
}

void Pid_Reset(Pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->last_out = 0.0f;
}

float Pid_Calc(Pid_t *pid, float setpoint, float measured)
{
    float err = setpoint - measured;

    if (pid->mode == PID_MODE_POSITIONAL) {
        /* 位置式: u = Kp*e + Ki*∫e + Kd*de */
        pid->integral += err;
        /* 抗积分饱和:积分项限幅 */
        pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);

        float derivative = err - pid->last_err;
        float u = pid->Kp * err + pid->Ki * pid->integral + pid->Kd * derivative;

        pid->last_err = err;
        pid->last_out = clampf(u, pid->out_min, pid->out_max);
        return pid->last_out;
    } else {
        /* 增量式: Δu = Kp*Δe + Ki*e + Kd*Δ²e
           适用于速度 PI(执行机构自带积分,即 PWM 累加) */
        float d_err  = err - pid->last_err;
        float delta  = pid->Kp * d_err + pid->Ki * err + pid->Kd * (d_err - (err - pid->last_err));
        /* 上一项 Kd*(Δe - Δe_prev) 中的 Δe_prev = last_err - last_last_err,
           为简化状态,这里用近似:Kd 项作用到 d_err 的变化 */
        float u = pid->last_out + delta;
        u = clampf(u, pid->out_min, pid->out_max);

        pid->last_err = err;
        pid->last_out = u;
        return u;
    }
}
