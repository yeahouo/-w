/**
 * @file config.h
 * @brief 全局配置中心 — 底盘几何、PID 参数、引脚映射、调度周期
 *
 * 所有可调参数集中在此文件,便于上车实战调参时统一管理。
 * 修改参数后必须重新编译下载。
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ===========================================================
 * 1. 调度周期(单位 ms,基于 SysTick / Timer 中断)
 * =========================================================== */
#define TICK_MOTOR_PI_MS        (1)      /* 单电机速度 PI 内环 1kHz   */
#define TICK_ENCODER_MS         (10)     /* 编码器采样 100Hz          */
#define TICK_TRACKER_PD_MS      (20)     /* 转向 PD 外环 50Hz         */
#define TICK_VISION_TIMEOUT_MS  (200)    /* OpenMV 帧超时阈值         */
#define TICK_FSM_MS             (10)     /* 状态机 100Hz              */
#define TICK_DEBUG_MS           (100)    /* 调试串口输出 10Hz         */

/* ===========================================================
 * 1b. LINE_FOLLOW v8 — 5 路灰度竞赛级循迹算法参数
 *     架构: 加权位置法 + 位置式 PD + 5 档增益调度 + 元素状态机
 *     所有可调参数集中在此段, 方便上车实战调参。
 * =========================================================== */

/* 5 路灰度传感器权重 (中心 = 0, 向左负, 向右正) */
#define LINE_W_L1               (-2.0f)
#define LINE_W_L2               (-1.0f)
#define LINE_W_M                ( 0.0f)
#define LINE_W_R1               ( 1.0f)
#define LINE_W_R2               ( 2.0f)

/* 历史命中投票 (3 帧多数表决, 防单帧噪声) */
#define LINE_VOTE_FRAMES        (3)

/* 死区: |position| < 此值视为直线, 强制 0 (防量化噪声被 D 放大) */
#define LINE_DEAD_ZONE          (0.15f)

/* 一阶低通滤波系数 α (越大越相信当前帧)
   100Hz 采样建议 0.3~0.5, 滤掉传感器抖动但保留弯道响应 */
#define LINE_POS_FILTER_ALPHA   (0.40f)

/* 5 档增益调度边界 (|position| 阈值, 加 0.1 迟滞带防乒乓)
   D1 覆盖直线段所有小偏离 (< 1.0), 配合 STRAIGHT 状态 delta±1 限制 */
#define LINE_BAND_D1_D2         (1.00f)
#define LINE_BAND_D2_D3         (1.50f)
#define LINE_BAND_D3_D4         (1.80f)
#define LINE_HYSTERESIS         (0.10f)

/* 各档 PD 参数起点
   D1 直线段用纯 P (KD=0): 直线段不需要预测, D 只会放大 sensor 量化跳变
   D2~D4 才加 D 项抑制弯道振荡 */
#define D1_KP                   (15.0f)
#define D1_KD                   ( 0.0f)     /* !! 直线段纯 P, 关键防过冲 */
#define D2_KP                   (18.0f)
#define D2_KD                   ( 15.0f)
#define D3_KP                   (22.0f)
#define D3_KD                   ( 40.0f)
#define D4_KP                   (35.0f)
#define D4_KD                   (150.0f)

/* delta rate limit: 每 tick delta 变化不超过此值, 强制渐变转向
   防止档位切换/位置突变时 delta 瞬间跳到极值 */
#define LINE_DELTA_RATE_LIMIT   (1)

/* 各状态最低速 PWM 档 (0~20, 调参阶段全部最慢, 调好后 ×1.5 逐档加速) */
#define SPEED_MIN_STRAIGHT      (2u)   /* D1 直道外侧 = 内侧 (10%) */
#define SPEED_MIN_CURVE_OUT     (2u)   /* D2/D3 弯道外侧 (10%) */
#define SPEED_MIN_CURVE_IN      (2u)   /* D2/D3 弯道内侧 (10%) */
#define SPEED_MIN_SHARP_OUT     (2u)   /* D4 急弯外侧 (10%) */
#define SPEED_MIN_SHARP_IN      (1u)   /* D4 急弯内侧 (5%, 刚够动) */
#define SPEED_LOST_SEARCH       (2u)   /* D5 丢线搜索 (10%, 必须够力驱动电机) */

/* 丢线三段式超时 (ms, 主循环 100Hz = 10ms/tick) */
#define LINE_LOST_INERTIA_MS    (500u)  /* 阶段 1: 惯性续行 */
#define LINE_LOST_SEARCH_MS     (2000u) /* 阶段 2: 扫掠搜索 */
#define LINE_LOST_FATAL_MS      (3000u) /* 阶段 3: 全停 */

/* 元素识别确认帧数 (防瞬时误判) */
#define CROSS_CONFIRM_FRAMES    (2u)    /* 十字: 最左+最右同时命中 N 帧 */
#define SHARP_CONFIRM_FRAMES    (3u)    /* 急弯: 最外传感器持续 N 帧 */
#define STRAIGHT_CONFIRM_MS     (200u)  /* 直道: position 稳定 N ms 才进 STRAIGHT */

/* 控制输出限幅 (PWM 档 0~20) */
#define LINE_OUT_MAX_DUTY       (8u)    /* 单周期最大输出 = 40%, 防失控冲出 */

/* ===========================================================
 * 2. 底盘几何(单位 mm,装车后用尺子量准,精确到 1mm)
 *   待主人填实测值!!!
 * =========================================================== */
#define CHASSIS_WHEEL_BASE      (160)    /* 轴距(前后轴距离)       */
#define CHASSIS_WHEEL_TRACK     (170)    /* 轮距(左右轮中心距)     */
#define CHASSIS_WHEEL_RADIUS    (32)     /* 轮半径                   */
#define CHASSIS_WHEEL_CIRCUM    (2*3.14159f*CHASSIS_WHEEL_RADIUS)

/* 编码器参数(装电机后填) */
#define ENCODER_PPR             (11)     /* 编码器每转脉冲数         */
#define ENCODER_REDUCTION       (30)     /* 减速比(电机:轮)       */
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4 * ENCODER_REDUCTION) /* 4 倍频 */

/* ===========================================================
 * 3. 电机 PWM 参数
 * =========================================================== */
#define MOTOR_PWM_FREQ_HZ       (20000)  /* 20kHz,人耳听不到         */
#define MOTOR_PWM_PERIOD        (1000)   /* 计数周期,占空比 0~1000  */
/* !! 调试中临时降到 400 — Stage 4/6 单/多电机闭环测试的保险值
   Stage 9 上地板跟线前要改回 950(否则 SPEED_TARGET_TRACKING=400 跑不出力) */
#define MOTOR_PWM_MAX           (400)
#define MOTOR_PWM_DEAD_ZONE     (30)     /* 死区补偿,低于此值不动 */

/* ===========================================================
 * 4. 速度 PI 参数(每电机一组,增量式)
 *   初值用经验值,实际靠示波器响应曲线整定
 * =========================================================== */
#define SPEED_PI_KP             (0.6f)
#define SPEED_PI_KI             (0.05f)
#define SPEED_PI_KD             (0.0f)
#define SPEED_PI_OUT_MAX        ((float)MOTOR_PWM_MAX)
#define SPEED_PI_OUT_MIN        (-(float)MOTOR_PWM_MAX)

/* ===========================================================
 * 5. 转向 PD 参数(循迹外环)
 *   输入:偏差 e(像素),输出:目标角速度 ω(rad/s)
 * =========================================================== */
#define TRACK_PD_KP             (0.05f)
#define TRACK_PD_KD             (0.20f)
#define TRACK_PD_OUT_MAX        (5.0f)   /* ω 限幅,防甩出           */
#define TRACK_E_FILTER_ALPHA    (0.3f)   /* 偏差低通滤波系数         */

/* 各状态下的目标线速度(mm/s),元素场景慢速 */
#define SPEED_TARGET_TRACKING   (400)
#define SPEED_TARGET_CURVE      (250)
#define SPEED_TARGET_CROSS      (200)
#define SPEED_TARGET_ROUNDABOUT (180)
#define SPEED_TARGET_SLOPE      (220)
#define SPEED_TARGET_LOST       (0)      /* 丢线停车                 */

/* ===========================================================
 * 5b. 单 UART 共享配置(OpenMV 帧 + 调试日志 + 遥测 都走同一个 UART)
 * =========================================================== */
#define UART_SHARED_BAUD        (115200)  /* 波特率,OpenMV 端必须匹配 */
#define UART_SHARED_DATA_BITS   (8)
#define UART_SHARED_STOP_BITS   (1)
/* 日志默认级别(共享 UART 时,TX 流量不能过大,免得挤压 OpenMV RX) */
#define LOG_DEFAULT_LEVEL       (1)       /* 0=DEBUG 1=INFO 2=WARN 3=ERROR */
#define TELEMETRY_DEFAULT_RATE  (20)      /* Hz,共享 UART 建议不超过 50 */

/* ===========================================================
 * 6. 通道枚举(统一索引)
 * =========================================================== */
typedef enum {
    MOTOR_LF = 0,   /* Left Front   */
    MOTOR_LR,       /* Left Rear    */
    MOTOR_RF,       /* Right Front  */
    MOTOR_RR,       /* Right Rear   */
    MOTOR_CH_COUNT
} MotorChannel_t;

typedef enum {
    ELEM_UNKNOWN   = 0,
    ELEM_STRAIGHT  = 1,
    ELEM_LEFT      = 2,
    ELEM_RIGHT     = 3,
    ELEM_CROSS     = 4,
    ELEM_ROUNDABOUT= 5,
    ELEM_SLOPE     = 6,
    ELEM_STOP_LINE = 7,
} TrackElement_t;

#endif /* CONFIG_H */
