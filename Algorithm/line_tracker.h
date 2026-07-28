/**
 * @file line_tracker.h
 * @brief LINE_FOLLOW v8 — 5 路灰度竞赛级循迹算法
 *
 * 架构(5 层管线):
 *   [1] 5-bit 命中读取
 *   [2] 加权位置法连续化 + 3 帧历史投票
 *   [3] 死区 + 一阶低通滤波
 *   [4] 5 档增益调度 PD (D1 直道 / D2 小弯 / D3 大弯 / D4 急弯 / D5 丢线)
 *   [5] 6 状态元素识别状态机 + 丢线三段式处理
 *   [6] 差速输出 → (pwm_l_duty, pwm_r_duty)
 *
 * 输入: 5-bit 命中模式 (bit0=L1 .. bit4=R2, 1=检测到黑线)
 * 输出: 左右轮 PWM duty (0~20 档) + 方向
 *
 * 调用约定:
 *   LineTracker_Init();
 *   while(1) {
 *       uint8_t bits = read_line_sensors();
 *       LineTracker_Update(bits);              // 100Hz
 *       pwm_l_duty = LineTracker_GetLeftDuty();
 *       pwm_r_duty = LineTracker_GetRightDuty();
 *   }
 *
 * 与硬件无关: 不直接读 GPIO / 不直接驱动电机,
 *            只做 bits → (duty_l, duty_r) 的纯算法映射。
 *            可在 PC 上做单元测试 (mock bits 序列)。
 */
#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  状态机: 6 个循迹状态
 * ============================================================ */
typedef enum {
    LT_ST_WAIT_LINE  = 0,   /* 启动等线 (还没见过线)            */
    LT_ST_STRAIGHT   = 1,   /* 直道 (position 稳定 ±0.2 持续)   */
    LT_ST_CURVE      = 2,   /* 弯道 (PD 正常输出)               */
    LT_ST_CROSS      = 3,   /* 十字 (最左+最右同时命中)         */
    LT_ST_SHARP      = 4,   /* 急弯 (最外传感器持续命中)        */
    LT_ST_LOST       = 5,   /* 丢线 (三段式处理)                */
    LT_ST_ERROR      = 6,   /* 持续丢线 > 3s 全停               */
} LT_State_t;

/* ============================================================
 *  5 档增益调度等级 (供日志/调试查询)
 * ============================================================ */
typedef enum {
    LT_GEAR_D1_STRAIGHT = 0,
    LT_GEAR_D2_SMALL    = 1,
    LT_GEAR_D3_LARGE    = 2,
    LT_GEAR_D4_SHARP    = 3,
    LT_GEAR_D5_LOST     = 4,
} LT_Gear_t;

/* ============================================================
 *  公开接口
 * ============================================================ */

/* 初始化 (清状态、重置 PID、记录启动时间) */
void LineTracker_Init(void);

/**
 * @brief 主循环周期任务 (建议 100Hz, 10ms/tick)
 * @param bits 5-bit 命中: bit0=L1 .. bit4=R2, 1=黑线
 * @param now_ms 系统时间戳 (ms), 用于状态计时
 */
void LineTracker_Update(uint8_t bits, uint32_t now_ms);

/* 取控制输出 — 直接喂给 main.c 的 pwm_l_duty / pwm_r_duty */
uint8_t LineTracker_GetLeftDuty(void);
uint8_t LineTracker_GetRightDuty(void);
uint8_t LineTracker_GetLeftDir(void);    /* 1=正转, 0=反转 */
uint8_t LineTracker_GetRightDir(void);

/* 调试查询 */
LT_State_t  LineTracker_GetState(void);
LT_Gear_t   LineTracker_GetGear(void);
float       LineTracker_GetPosition(void);     /* 滤波后 position [-2,+2] */
int8_t      LineTracker_GetLastError(void);    /* 原始误差 (像素/传感器位) */

#endif /* LINE_TRACKER_H */
