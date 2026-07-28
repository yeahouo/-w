/**
 * @file line_tracker.c
 * @brief LINE_FOLLOW v8 — 5 路灰度竞赛级循迹算法实现
 *
 * 5 层管线 (详见 line_tracker.h 头注释)
 *
 * 控制输出约定:
 *   position ∈ [-2, +2], 负=线在左, 正=线在右
 *   turn    = Kp*position + Kd*d(position)   (带增益调度)
 *   turn > 0 → 车头右转追线 → 左轮加速, 右轮减速
 *   turn < 0 → 车头左转追线 → 左轮减速, 右轮加速
 *
 * 状态机:
 *   WAIT_LINE → STRAIGHT/CURVE/CROSS/SHARP → LOST → ERROR
 *   任何状态丢线都进 LOST 三段式处理
 */
#include "line_tracker.h"
#include "config.h"
#include <math.h>

/* ============================================================
 *  内部状态
 * ============================================================ */

/* 3 帧历史投票缓冲 (每路传感器独立) */
static uint8_t s_vote_buf[LINE_VOTE_FRAMES];
static uint8_t s_vote_idx;

/* position 滤波后/原始 */
static float    s_pos_filtered;
static float    s_pos_prev;          /* 上一帧滤波后, 用于 D 项 */
static int8_t   s_last_pos_sign;     /* 最后一次有效方向 -1/+1 */

/* 状态机 */
static LT_State_t s_state;
static uint32_t   s_state_enter_ms;
static uint32_t   s_last_line_seen_ms;
static uint32_t   s_straight_stable_ms;  /* 直道稳定计时 */

/* 元素识别确认计数 */
static uint8_t   s_cross_confirm;
static uint8_t   s_sharp_confirm;

/* 控制输出 */
static uint8_t   s_out_l_duty;
static uint8_t   s_out_r_duty;
static uint8_t   s_out_l_dir;
static uint8_t   s_out_r_dir;
static int       s_last_delta;       /* delta rate limit 用 */

/* 当前档位 (供日志) */
static LT_Gear_t s_gear;

/* ============================================================
 *  内部工具
 * ============================================================ */

static float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

static int clampi(int x, int lo, int hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* 多数投票: bits 中某一位在 3 帧里出现 >= 2 次则视为 1 */
static uint8_t majority_vote(uint8_t bits)
{
    s_vote_buf[s_vote_idx] = bits;
    s_vote_idx = (s_vote_idx + 1) % LINE_VOTE_FRAMES;

    uint8_t result = 0;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t cnt = 0;
        for (uint8_t f = 0; f < LINE_VOTE_FRAMES; f++) {
            if (s_vote_buf[f] & (1u << i)) cnt++;
        }
        if (cnt > LINE_VOTE_FRAMES / 2) result |= (1u << i);
    }
    return result;
}

/* 加权位置法 — 返回 [-2, +2], 全丢线时返回 NaN 标志位 */
static bool weighted_position(uint8_t bits, float *pos_out)
{
    /* 拆位 */
    float s0 = (bits >> 0) & 1 ? 1.0f : 0.0f;   /* L1 */
    float s1 = (bits >> 1) & 1 ? 1.0f : 0.0f;   /* L2 */
    float s2 = (bits >> 2) & 1 ? 1.0f : 0.0f;   /* M  */
    float s3 = (bits >> 3) & 1 ? 1.0f : 0.0f;   /* R1 */
    float s4 = (bits >> 4) & 1 ? 1.0f : 0.0f;   /* R2 */

    float sum = s0 + s1 + s2 + s3 + s4;
    if (sum < 0.5f) {
        return false;   /* 全丢线 */
    }

    /* 加权 (权重见 config.h) */
    float pos = (LINE_W_L1*s0 + LINE_W_L2*s1 + LINE_W_M*s2
               + LINE_W_R1*s3 + LINE_W_R2*s4) / sum;

    /* 单传感器命中优先: 只 1 路亮时直接用该路权值, 避免被除法污染 */
    if (sum < 1.5f) {
        if      (s0 > 0.5f) pos = LINE_W_L1;
        else if (s1 > 0.5f) pos = LINE_W_L2;
        else if (s2 > 0.5f) pos = LINE_W_M;
        else if (s3 > 0.5f) pos = LINE_W_R1;
        else if (s4 > 0.5f) pos = LINE_W_R2;
    }

    *pos_out = pos;
    return true;
}

/* 选择当前档位 (带 0.1 迟滞带防乒乓) */
static LT_Gear_t select_gear(float pos_abs, LT_Gear_t prev)
{
    /* 迟滞带: 升档阈值 = 标称 + hyst/2, 降档阈值 = 标称 - hyst/2 */
    float h = LINE_HYSTERESIS * 0.5f;
    float b12_hi = LINE_BAND_D1_D2 + h, b12_lo = LINE_BAND_D1_D2 - h;
    float b23_hi = LINE_BAND_D2_D3 + h, b23_lo = LINE_BAND_D2_D3 - h;
    float b34_hi = LINE_BAND_D3_D4 + h, b34_lo = LINE_BAND_D3_D4 - h;

    switch (prev) {
    case LT_GEAR_D1_STRAIGHT:
        if (pos_abs >= b12_hi) return LT_GEAR_D2_SMALL;
        return LT_GEAR_D1_STRAIGHT;
    case LT_GEAR_D2_SMALL:
        if (pos_abs >= b23_hi) return LT_GEAR_D3_LARGE;
        if (pos_abs <  b12_lo) return LT_GEAR_D1_STRAIGHT;
        return LT_GEAR_D2_SMALL;
    case LT_GEAR_D3_LARGE:
        if (pos_abs >= b34_hi) return LT_GEAR_D4_SHARP;
        if (pos_abs <  b23_lo) return LT_GEAR_D2_SMALL;
        return LT_GEAR_D3_LARGE;
    case LT_GEAR_D4_SHARP:
        if (pos_abs <  b34_lo) return LT_GEAR_D3_LARGE;
        return LT_GEAR_D4_SHARP;
    default:
        return LT_GEAR_D1_STRAIGHT;
    }
}

/* 档位 → 基础速度 + PD 参数 */
static void gear_params(LT_Gear_t g, float *kp, float *kd, uint8_t *base_out, uint8_t *base_in)
{
    switch (g) {
    case LT_GEAR_D1_STRAIGHT:
        *kp = D1_KP; *kd = D1_KD;
        *base_out = SPEED_MIN_STRAIGHT; *base_in = SPEED_MIN_STRAIGHT;
        break;
    case LT_GEAR_D2_SMALL:
        *kp = D2_KP; *kd = D2_KD;
        *base_out = SPEED_MIN_CURVE_OUT; *base_in = SPEED_MIN_CURVE_IN;
        break;
    case LT_GEAR_D3_LARGE:
        *kp = D3_KP; *kd = D3_KD;
        *base_out = SPEED_MIN_CURVE_OUT; *base_in = SPEED_MIN_CURVE_IN;
        break;
    case LT_GEAR_D4_SHARP:
        *kp = D4_KP; *kd = D4_KD;
        *base_out = SPEED_MIN_SHARP_OUT; *base_in = SPEED_MIN_SHARP_IN;
        break;
    default:
        *kp = D1_KP; *kd = D1_KD;
        *base_out = SPEED_MIN_STRAIGHT; *base_in = SPEED_MIN_STRAIGHT;
        break;
    }
}

static void set_state(LT_State_t s, uint32_t now_ms)
{
    s_state          = s;
    s_state_enter_ms = now_ms;
    s_cross_confirm  = 0;
    s_sharp_confirm  = 0;
    s_straight_stable_ms = 0;
}

static void set_output(uint8_t l_duty, uint8_t r_duty, uint8_t l_dir, uint8_t r_dir)
{
    s_out_l_duty = l_duty;
    s_out_r_duty = r_duty;
    s_out_l_dir  = l_dir;
    s_out_r_dir  = r_dir;
}

/* ============================================================
 *  初始化
 * ============================================================ */

void LineTracker_Init(void)
{
    for (uint8_t i = 0; i < LINE_VOTE_FRAMES; i++) s_vote_buf[i] = 0;
    s_vote_idx = 0;

    s_pos_filtered       = 0.0f;
    s_pos_prev           = 0.0f;
    s_last_pos_sign      = 0;

    s_state              = LT_ST_WAIT_LINE;
    s_state_enter_ms     = 0;
    s_last_line_seen_ms  = 0;
    s_straight_stable_ms = 0;

    s_cross_confirm = 0;
    s_sharp_confirm = 0;

    s_out_l_duty = 0; s_out_r_duty = 0;
    s_out_l_dir  = 1; s_out_r_dir  = 1;
    s_last_delta = 0;

    s_gear = LT_GEAR_D1_STRAIGHT;
}

/* ============================================================
 *  核心: 主循环周期任务 (100Hz)
 * ============================================================ */

void LineTracker_Update(uint8_t bits_raw, uint32_t now_ms)
{
    /* ---- [1] 历史命中投票 (3 帧多数表决) ---- */
    uint8_t bits = majority_vote(bits_raw);

    /* ---- [2] 加权位置法 ---- */
    float pos_raw;
    bool   has_line = weighted_position(bits, &pos_raw);

    /* ---- [3] 死区 + 一阶低通滤波 ---- */
    float pos;
    if (!has_line) {
        /* 丢线时保持上一帧滤波值, 不更新 (避免突变) */
        pos = s_pos_filtered;
    } else {
        /* 死区: |pos| < DEAD_ZONE 强制 0 */
        if (fabsf(pos_raw) < LINE_DEAD_ZONE) pos_raw = 0.0f;
        /* 一阶低通: y = α*x + (1-α)*y_prev */
        pos = LINE_POS_FILTER_ALPHA * pos_raw
            + (1.0f - LINE_POS_FILTER_ALPHA) * s_pos_filtered;
    }

    /* 记录最后有效方向 */
    if (has_line && fabsf(pos) > LINE_DEAD_ZONE) {
        s_last_pos_sign  = (pos > 0) ? +1 : -1;
        s_last_line_seen_ms = now_ms;
    }

    /* ---- [4] PD 计算 + 增益调度 ---- */
    float kp, kd;
    uint8_t base_out, base_in;
    if (has_line) {
        s_gear = select_gear(fabsf(pos), s_gear);
    } else {
        s_gear = LT_GEAR_D5_LOST;
    }
    gear_params(s_gear, &kp, &kd, &base_out, &base_in);

    float d_pos = pos - s_pos_prev;
    float turn  = kp * pos + kd * d_pos;     /* +: 右转, -: 左转 */

    s_pos_prev = pos;
    s_pos_filtered = pos;

    /* ---- [5] 状态机: 元素识别 ---- */

    uint32_t lost_duration = now_ms - s_last_line_seen_ms;
    uint32_t state_duration = now_ms - s_state_enter_ms;

    /* 持续丢线 > 3s → ERROR 全停 (最高优先级) */
    if (has_line == false && lost_duration > LINE_LOST_FATAL_MS) {
        if (s_state != LT_ST_ERROR) set_state(LT_ST_ERROR, now_ms);
        set_output(0, 0, 1, 1);
        return;
    }

    /* 全丢线 → LOST 状态 (两段式: 惯性续行 → 扫掠搜索 → 超时全停) */
    if (has_line == false) {
        if (s_state != LT_ST_LOST) set_state(LT_ST_LOST, now_ms);

        uint32_t lost_ms = now_ms - s_last_line_seen_ms;
        uint8_t  duty    = SPEED_LOST_SEARCH;        /* 至少 2 档, 确保电机能动 */

        if (lost_ms < LINE_LOST_INERTIA_MS) {
            /* 阶段 1: 惯性续行 (0~500ms)
               两轮都正转, 按最后方向外侧快/内侧慢, 车继续前进找线
               没有方向记忆时 (s_last_pos_sign==0) 默认直行 */
            if (s_last_pos_sign > 0) {
                /* 线最后在右 → 左轮(外)快, 右轮(内)慢 */
                set_output(duty + 1, duty, 1, 1);
            } else if (s_last_pos_sign < 0) {
                /* 线最后在左 → 左轮(内)慢, 右轮(外)快 */
                set_output(duty, duty + 1, 1, 1);
            } else {
                /* 启动后没见过线 (不太可能走到这, WAIT_LINE 会拦) — 直行兜底 */
                set_output(duty, duty, 1, 1);
            }
        } else {
            /* 阶段 2: 扫掠搜索 (500ms~3s)
               内侧停, 外侧正转, 原地小转扩大搜索范围 */
            if (s_last_pos_sign >= 0) {
                /* 线最后在右 → 左轮(外)正转, 右轮(内)停 */
                set_output(duty + 1, 0, 1, 1);
            } else {
                /* 线最后在左 → 左轮(内)停, 右轮(外)正转 */
                set_output(0, duty + 1, 1, 1);
            }
        }
        return;
    }

    /* ===== 有线状态 ===== */

    /* 十字识别: 最左+最右同时命中, 持续 >= CROSS_CONFIRM_FRAMES */
    bool is_cross_pattern = (bits & 0b10001) == 0b10001;
    if (is_cross_pattern && fabsf(pos) < 0.5f) {
        s_cross_confirm++;
        if (s_cross_confirm >= CROSS_CONFIRM_FRAMES
            && s_state != LT_ST_CROSS) {
            set_state(LT_ST_CROSS, now_ms);
        }
    } else {
        s_cross_confirm = 0;
    }

    /* 急弯识别: 最外传感器持续命中 + 误差大 */
    bool is_sharp_pattern = ((bits & 0b00001) || (bits & 0b10000))
                            && fabsf(pos) >= LINE_BAND_D3_D4;
    if (is_sharp_pattern) {
        s_sharp_confirm++;
        if (s_sharp_confirm >= SHARP_CONFIRM_FRAMES
            && s_state != LT_ST_SHARP) {
            set_state(LT_ST_SHARP, now_ms);
        }
    } else {
        s_sharp_confirm = 0;
    }

    /* 直道稳定识别: |pos| < DEAD_ZONE 持续 STRAIGHT_CONFIRM_MS */
    if (fabsf(pos) < LINE_DEAD_ZONE) {
        s_straight_stable_ms += 10;   /* 100Hz = 10ms/tick */
    } else {
        s_straight_stable_ms = 0;
    }

    /* ===== 决定当前状态 ===== */
    /* 优先级: CROSS > SHARP > STRAIGHT > CURVE */
    LT_State_t target;
    if (s_state == LT_ST_CROSS && s_cross_confirm > 0) {
        target = LT_ST_CROSS;
    } else if (s_state == LT_ST_SHARP && s_sharp_confirm > 0) {
        target = LT_ST_SHARP;
    } else if (s_sharp_confirm >= SHARP_CONFIRM_FRAMES) {
        target = LT_ST_SHARP;
    } else if (s_cross_confirm >= CROSS_CONFIRM_FRAMES) {
        target = LT_ST_CROSS;
    } else if (s_straight_stable_ms >= STRAIGHT_CONFIRM_MS) {
        target = LT_ST_STRAIGHT;
    } else {
        target = LT_ST_CURVE;
    }

    if (target != s_state) set_state(target, now_ms);

    /* ===== 计算输出 (差速模型) ===== */

    /* PD 输出 turn 映射到 PWM 档差: 每 PD_SCALE 个 turn 单位 = 1 档差速
       base=2 (最低速) 时, delta 最大被 clamp 到 ±2; STRAIGHT 状态再限 ±1
       PD_SCALE=10 让 D1_Kp=10 时 pos=0.5 → turn=5 → delta=0.5 ≈ 1 (能修正) */
    const float PD_SCALE = 10.0f;     /* 经验缩放, 调参时可改 */
    int delta = (int)(turn / PD_SCALE + 0.5f);

    if (s_state == LT_ST_SHARP) {
        /* 急弯: 内侧反转, 外侧正转 (原地小转) */
        uint8_t out_duty = base_out;
        uint8_t in_duty  = base_in;
        if (s_last_pos_sign >= 0) {
            /* 线在右 → 右转 → 左轮(外)正转, 右轮(内)反转 */
            set_output(out_duty, in_duty, 1, 0);
        } else {
            /* 线在左 → 左转 → 左轮(内)反转, 右轮(外)正转 */
            set_output(in_duty, out_duty, 0, 1);
        }
        return;
    }

    if (s_state == LT_ST_CROSS) {
        /* 十字: 直行不转向 (PD 仍参与但不强化差速) */
        set_output(base_out, base_out, 1, 1);
        return;
    }

    /* STRAIGHT / CURVE: 正常 PD 差速输出
       turn > 0 (线在右): 左轮加, 右轮减
       turn < 0 (线在左): 左轮减, 右轮加 */
    delta = clampi(delta, -(int)base_in, (int)base_out);

    /* 直道状态专属限制: delta 最大 ±1, 防止任何量化跳变引起大幅修正 */
    if (s_state == LT_ST_STRAIGHT) {
        delta = clampi(delta, -1, 1);
    }

    /* delta rate limit: 每 tick 变化不超过 ±LINE_DELTA_RATE_LIMIT
       强制渐变转向, 防止档位切换/位置突变时 delta 瞬跳到极值 */
    if (delta > s_last_delta + LINE_DELTA_RATE_LIMIT) {
        delta = s_last_delta + LINE_DELTA_RATE_LIMIT;
    } else if (delta < s_last_delta - LINE_DELTA_RATE_LIMIT) {
        delta = s_last_delta - LINE_DELTA_RATE_LIMIT;
    }
    s_last_delta = delta;

    int v_l = (int)base_out + delta;
    int v_r = (int)base_out - delta;

    v_l = clampi(v_l, 0, (int)LINE_OUT_MAX_DUTY);
    v_r = clampi(v_r, 0, (int)LINE_OUT_MAX_DUTY);

    set_output((uint8_t)v_l, (uint8_t)v_r, 1, 1);
}

/* ============================================================
 *  Getter
 * ============================================================ */

uint8_t LineTracker_GetLeftDuty(void)  { return s_out_l_duty; }
uint8_t LineTracker_GetRightDuty(void) { return s_out_r_duty; }
uint8_t LineTracker_GetLeftDir(void)   { return s_out_l_dir; }
uint8_t LineTracker_GetRightDir(void)  { return s_out_r_dir; }

LT_State_t LineTracker_GetState(void)    { return s_state; }
LT_Gear_t  LineTracker_GetGear(void)     { return s_gear; }
float      LineTracker_GetPosition(void) { return s_pos_filtered; }
int8_t     LineTracker_GetLastError(void){ return (int8_t)(s_pos_filtered * 10); }
