/**
 * @file motor.c
 * @brief 电机层 — 四路 PWM + 单电机速度 PI 闭环(增量式,1kHz)
 *        + 堵转检测 + PI 发散检测 + 遥测 getter
 */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "port.h"
#include "log.h"
#include "errcode.h"
#include "config.h"

/* 每通道一个 PI 控制器 + 状态缓存 */
static Pid_t   s_pi[MOTOR_CH_COUNT];
static int16_t s_target_mm_s[MOTOR_CH_COUNT];
static int16_t s_output_pwm[MOTOR_CH_COUNT];

/* 堵转检测:目标大但实测小持续 N ms */
#define STALL_TARGET_THR  (50)      /* 目标阈值(mm/s) */
#define STALL_ACTUAL_THR  (10)      /* 实测阈值(mm/s) */
#define STALL_TIME_MS     (300)
static uint32_t s_stall_begin_ms[MOTOR_CH_COUNT];
static bool     s_stalled[MOTOR_CH_COUNT];

/* PI 发散检测:输出长时间打满 */
#define PI_SAT_TIME_MS     (800)
static uint32_t s_sat_begin_ms[MOTOR_CH_COUNT];

void Motor_Init(void)
{
    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        Pid_Init(&s_pi[i], PID_MODE_INCREMENTAL,
                 SPEED_PI_KP, SPEED_PI_KI, SPEED_PI_KD,
                 SPEED_PI_OUT_MIN, SPEED_PI_OUT_MAX);
        s_target_mm_s[i]   = 0;
        s_output_pwm[i]    = 0;
        s_stall_begin_ms[i]= 0;
        s_stalled[i]       = false;
        s_sat_begin_ms[i]  = 0;
    }
    LOG_I("MOT", "init ok, Kp=%.3f Ki=%.3f Kd=%.3f, PWMmax=%d",
          SPEED_PI_KP, SPEED_PI_KI, SPEED_PI_KD, MOTOR_PWM_MAX);
}

void Motor_SetTargetSpeed(MotorChannel_t ch, int16_t target_mm_s)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return;
    s_target_mm_s[ch] = target_mm_s;
}

void Motor_SetRawPWM(MotorChannel_t ch, int16_t pwm)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return;
    if (pwm >  MOTOR_PWM_MAX) pwm =  MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    Port_MotorSetPWM(ch, pwm);
    s_output_pwm[ch] = pwm;
    Pid_Reset(&s_pi[ch]);
    LOG_D("MOT", "ch%d raw pwm=%d", ch, pwm);
}

void Motor_PI_Tick(void)
{
    /* 1kHz 调用 */
    uint32_t now = Port_NowMs();

    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        MotorChannel_t ch = (MotorChannel_t)i;
        float fb = Encoder_GetSpeed(ch);
        float u  = Pid_Calc(&s_pi[i], (float)s_target_mm_s[i], fb);
        int16_t pwm = (int16_t)u;

        /* 死区补偿 */
        if (s_target_mm_s[i] != 0) {
            if (pwm > 0 && pwm < MOTOR_PWM_DEAD_ZONE) pwm = MOTOR_PWM_DEAD_ZONE;
            if (pwm < 0 && pwm > -MOTOR_PWM_DEAD_ZONE) pwm = -MOTOR_PWM_DEAD_ZONE;
        }

        s_output_pwm[i] = pwm;
        Port_MotorSetPWM(ch, pwm);

        /* ---------- 堵转检测 ---------- */
        int16_t abs_t = s_target_mm_s[i] > 0 ?  s_target_mm_s[i] : -s_target_mm_s[i];
        int16_t abs_a = (int16_t)(fb > 0 ? fb : -fb);
        if (abs_t > STALL_TARGET_THR && abs_a < STALL_ACTUAL_THR) {
            if (s_stall_begin_ms[i] == 0) {
                s_stall_begin_ms[i] = now;
            } else if (!s_stalled[i] && (now - s_stall_begin_ms[i]) > STALL_TIME_MS) {
                s_stalled[i] = true;
                Err_Report(ERR_MOTOR_STALL);
                LOG_E("MOT", "ch%d STALL: target=%d actual=%.0f pwm=%d",
                      i, s_target_mm_s[i], fb, pwm);
            }
        } else {
            if (s_stalled[i]) {
                LOG_I("MOT", "ch%d stall recovered", i);
            }
            s_stalled[i] = false;
            s_stall_begin_ms[i] = 0;
        }

        /* ---------- PI 发散检测(输出长时间打满) ---------- */
        int16_t abs_p = pwm > 0 ? pwm : -pwm;
        if (abs_p >= MOTOR_PWM_MAX - 5) {
            if (s_sat_begin_ms[i] == 0) s_sat_begin_ms[i] = now;
            else if ((now - s_sat_begin_ms[i]) > PI_SAT_TIME_MS) {
                Err_Report(ERR_PI_DIVERGE);
                LOG_W("MOT", "ch%d PI saturating >%dms, target=%d actual=%.0f",
                      i, PI_SAT_TIME_MS, s_target_mm_s[i], fb);
                s_sat_begin_ms[i] = now;   /* 避免每 tick 都报 */
            }
        } else {
            s_sat_begin_ms[i] = 0;
        }
    }
}

void Motor_AllStop(void)
{
    for (int i = 0; i < MOTOR_CH_COUNT; ++i) {
        s_target_mm_s[i] = 0;
        s_output_pwm[i]  = 0;
        Pid_Reset(&s_pi[i]);
        Port_MotorSetPWM((MotorChannel_t)i, 0);
    }
    LOG_I("MOT", "all stop");
}

/* ---------- 遥测 getter ---------- */
int16_t Motor_GetTargetSpeed(MotorChannel_t ch)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return 0;
    return s_target_mm_s[ch];
}

int16_t Motor_GetOutputPWM(MotorChannel_t ch)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return 0;
    return s_output_pwm[ch];
}

bool Motor_IsStalled(MotorChannel_t ch)
{
    if (ch < 0 || ch >= MOTOR_CH_COUNT) return false;
    return s_stalled[ch];
}
