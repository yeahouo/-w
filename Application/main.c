/**
 * @file main.c
 * @brief tracer-car LINE_FOLLOW v8 — 5 路灰度竞赛级循迹
 *
 * 策略 (USE_V8_ADVANCED=1):
 *   - 5 路灰度传感器读 GPIO → bitmask (bit0=L1 .. bit4=R2)
 *   - 喂给 LineTracker_Update(): 加权位置法 + 5 档增益调度 PD + 6 状态机
 *   - 丢线三段式: 方向记忆 → 惯性续行 → 扫掠搜索 → 超时全停
 *   - 调参阶段所有档位降到最低 (SPEED_MIN_*=1~2 档)
 *
 * 回退 (USE_V8_ADVANCED=0): v7 8-pattern 查表法
 *
 * 引脚 (TB6612FNG + 5 路灰度):
 *   左轮 = 电机 A: AIN1=PA13  AIN2=PA14  PWMA=PB2
 *   右轮 = 电机 B: BIN1=PA17  BIN2=PA16  PWMB=PB3
 *   灰度:  L1=PB17  L2=PA12  M=PA22  R1=PA27  R2=PA9
 *   极性:  LINE_ACTIVE_LEVEL = 0  (黑线时 GPIO = 0)
 */

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "config.h"
#include "line_tracker.h"
#include "oled.h"
#include "button.h"

/* ============================================================
 *  LINE_FOLLOW 版本开关 (改一个值切算法)
 *    6 = v6 5-档 err 查表法 (2026-07-07 灰度初版, 最简单最稳)
 *    7 = v7 8-pattern 查表法 (v6 扩展, 加丢线超时全停)
 *    8 = v8 竞赛级 (加权位置法 + 5 档增益调度 PD + 状态机)
 * ============================================================ */
#ifndef LINE_FOLLOW_VERSION
#define LINE_FOLLOW_VERSION  6
#endif
/* 兼容旧开关名 */
#define USE_V8_ADVANCED  (LINE_FOLLOW_VERSION == 8)

/* ============================================================
 *  电机引脚
 * ============================================================ */

#define AIN1_PINCM      IOMUX_PINCM35   /* PA13 */
#define AIN2_PINCM      IOMUX_PINCM36   /* PA14 */
#define PWMA_PINCM      IOMUX_PINCM15   /* PB2  */
#define BIN1_PINCM      IOMUX_PINCM39   /* PA17 */
#define BIN2_PINCM      IOMUX_PINCM38   /* PA16 */
#define PWMB_PINCM      IOMUX_PINCM16   /* PB3  */

#define AIN1_PIN        DL_GPIO_PIN_13
#define AIN2_PIN        DL_GPIO_PIN_14
#define BIN1_PIN        DL_GPIO_PIN_17
#define BIN2_PIN        DL_GPIO_PIN_16
#define PWMA_PIN        DL_GPIO_PIN_2
#define PWMB_PIN        DL_GPIO_PIN_3

#define DIR_PINS_MASK   (AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN)
#define PWM_PINS_MASK   (PWMA_PIN | PWMB_PIN)

/* 电机极性: +1=默认, -1=翻转(接线反向/镜像安装时改对应侧)。
 * 实测结论: 左轮接线反向 (原始 dir=1 物理后退), 右轮正常;
 * 故翻转左轮, 两轮 dir=1 统一为物理前进。
 * 若实测仍不对, 把对应侧 +1/-1 互换。 */
#define LEFT_MOTOR_POLARITY   (-1)
#define RIGHT_MOTOR_POLARITY  (+1)

#define LED_PIN         DL_GPIO_PIN_22

/* TM1637 四位数码管 (灰度未装, 复用 PB17=CLK / PA12=DIO)
 * CLK 在 GPIOB, DIO 在 GPIOA, 跨端口 — TM1637 协议不区分端口 */
#define TM_CLK_PORT   GPIOB
#define TM_CLK_PIN    DL_GPIO_PIN_17   /* PB17, 原灰度 L1 */
#define TM_CLK_PINCM  IOMUX_PINCM43
#define TM_DIO_PORT   GPIOA
#define TM_DIO_PIN    DL_GPIO_PIN_12   /* PA12, 原灰度 L2 */
#define TM_DIO_PINCM  IOMUX_PINCM34

/* 数码管测试模式: =1 只跑数码管自检(1→…→9→0 循环), 跳过循迹; =0 正常循迹 */
#define TM1637_TEST   1

/* OLED 自检: =1 周期把运行秒数刷到屏 (不阻断主循环, 数码管/循迹照常); =0 不刷 */
#define OLED_TEST     1

/* ============================================================
 *  灰度传感器引脚
 * ============================================================ */

#define LINE_ACTIVE_LEVEL  0   /* 0 = 黑线时读到 0;  1 = 黑线时读到 1 */

#define L1_PINCM   IOMUX_PINCM43   /* PB17 */
#define L2_PINCM   IOMUX_PINCM34   /* PA12 */
#define M_PINCM    IOMUX_PINCM47   /* PA22 */
#define R1_PINCM   IOMUX_PINCM60   /* PA27 */
#define R2_PINCM   IOMUX_PINCM20   /* PA9  */

#define L1_PIN     DL_GPIO_PIN_17
#define L2_PIN     DL_GPIO_PIN_12
#define M_PIN      DL_GPIO_PIN_22
#define R1_PIN     DL_GPIO_PIN_27
#define R2_PIN     DL_GPIO_PIN_9

#define LINE_PINS_A_MASK  (L2_PIN | M_PIN | R1_PIN | R2_PIN)
#define LINE_PINS_B_MASK  (L1_PIN)

/* SysTick 2kHz @ 32MHz → reload = 16000-1, PWM 20 档 (100Hz) */
#define SYSTICK_LOAD    (16000u - 1u)
#define PWM_PHASE_MAX   20

/* ============================================================
 *  测试模式开关 (验证电机用, 测完改回 0)
 * ============================================================ */
#define MOTOR_TEST_DRIVE  0

/* ============================================================
 *  查表控制 — 速度档位
 *
 *  注意: 两个电机制造有公差, 即使 PWM 一样, 实际转速也可能不同。
 *  因此直行档拆成左右两个, 主人实测后微调 TRIM 值补偿机械差异:
 *
 *    测试方法: 把车放直线上, 看它跑一段后往哪偏
 *      车一直往左偏  → 左轮快  → 减 SPEED_L_BASE 或 加 SPEED_R_BASE
 *      车一直往右偏  → 右轮快  → 减 SPEED_R_BASE 或 加 SPEED_L_BASE
 *
 *  例: 车往左偏 → 左轮快 → SPEED_L_BASE 改 3 (或 SPEED_R_BASE 改 5)
 * ============================================================ */

#define SPEED_L_BASE      8u    /* 左轮直行基础档 (40%) */
#define SPEED_R_BASE      8u    /* 右轮直行基础档 (40%) */
#define SPEED_TURN_SOFT   6u    /* 小转内侧档 (30%) */
#define SPEED_TURN_HARD   4u    /* 大转内侧档 (20%) */

/* ============================================================
 *  全局状态
 * ============================================================ */

volatile uint32_t g_system_ms = 0;

volatile uint8_t pwm_l_duty = 0;
volatile uint8_t pwm_r_duty = 0;
volatile uint8_t pwm_l_dir  = 0;
volatile uint8_t pwm_r_dir  = 0;

static volatile uint8_t s_ctrl_flag;
static volatile uint8_t s_tick_100ms;

static volatile uint8_t  s_line_raw;
static volatile uint32_t s_line_last_seen_ms;

/* 上一次有效 bits (丢线时复用方向) */
static uint8_t s_last_bits = 0;

/* ============================================================
 *  UART 工具
 * ============================================================ */

static void uart_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST)) {}
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

static void uart_print(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_print_u32(uint32_t v)
{
    char buf[12];
    int i = 11;
    buf[i--] = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v && i >= 0) { buf[i--] = '0' + (v % 10); v /= 10; }
    uart_print(&buf[i+1]);
}

/* ============================================================
 *  SysTick 2kHz — 软件 PWM + 系统 ms + 10Hz tick
 * ============================================================ */

void SysTick_Handler(void)
{
    static uint8_t ms_prescaler = 0;
    if (++ms_prescaler >= 2) {
        ms_prescaler = 0;
        g_system_ms++;
    }

    static uint8_t pwm_phase = 0;

    if (pwm_phase < pwm_l_duty) DL_GPIO_setPins  (GPIOB, PWMA_PIN);
    else                        DL_GPIO_clearPins(GPIOB, PWMA_PIN);
    if (pwm_phase < pwm_r_duty) DL_GPIO_setPins  (GPIOB, PWMB_PIN);
    else                        DL_GPIO_clearPins(GPIOB, PWMB_PIN);

    if (++pwm_phase >= PWM_PHASE_MAX) pwm_phase = 0;

    if (++s_tick_100ms >= 200) {
        s_tick_100ms = 0;
        s_ctrl_flag = 1;
    }

    static uint16_t led_cnt = 0;
    if (++led_cnt >= 1000) {
        led_cnt = 0;
        DL_GPIO_togglePins(GPIOB, LED_PIN);
    }
}

/* ============================================================
 *  电机方向控制
 * ============================================================ */

static void motor_apply_dir(void)
{
    /* 应用极性: POLARITY<0 时翻转该轮方向位(异或 1), 收敛到单点 */
    uint8_t l_dir = (uint8_t)(pwm_l_dir ^ (uint8_t)(LEFT_MOTOR_POLARITY  < 0));
    uint8_t r_dir = (uint8_t)(pwm_r_dir ^ (uint8_t)(RIGHT_MOTOR_POLARITY < 0));

    if (pwm_l_duty == 0) {
        DL_GPIO_clearPins(GPIOA, AIN1_PIN | AIN2_PIN);
    } else if (l_dir) {
        DL_GPIO_setPins  (GPIOA, AIN1_PIN);
        DL_GPIO_clearPins(GPIOA, AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, AIN1_PIN);
        DL_GPIO_setPins  (GPIOA, AIN2_PIN);
    }
    if (pwm_r_duty == 0) {
        DL_GPIO_clearPins(GPIOA, BIN1_PIN | BIN2_PIN);
    } else if (r_dir) {
        DL_GPIO_setPins  (GPIOA, BIN1_PIN);
        DL_GPIO_clearPins(GPIOA, BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, BIN1_PIN);
        DL_GPIO_setPins  (GPIOA, BIN2_PIN);
    }
}

/* ============================================================
 *  TM1637 四位数码管驱动 (CLK/DIO 两线, 软件模拟时序)
 * ============================================================ */

/* 段码 0-9 (a-g, 共阴; dp=bit7 不点亮) */
static const uint8_t tm_seg7[10] = {
    0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
};

static void tm_delay(void)
{
    /* ~几 us @ 80MHz; TM1637 时钟 <250kHz, 边沿间隔需 ≥1us */
    for (volatile int i = 0; i < 40; ++i) { __NOP(); }
}

static void tm_start(void)
{
    /* DIO 在 CLK 高时下降沿 = START */
    DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
}

static void tm_stop(void)
{
    /* DIO 在 CLK 高时上升沿 = STOP */
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN); tm_delay();
}

static void tm_write(uint8_t b)
{
    /* LSB first, 8 位数据 + 第 9 个 ACK 时钟(不读) */
    for (int i = 0; i < 8; ++i) {
        DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
        if (b & 0x01) DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN);
        else          DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN);
        b >>= 1;
        tm_delay();
        DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();  /* 上升沿采样 */
    }
    /* ACK 周期: DIO 设低(让 TM1637 应答, 避免推挽冲突) */
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
}

/* 4 位同显一个数字 (自检用: 1111 / 2222 / ... / 0000) */
static void tm_show_same(uint8_t digit)
{
    if (digit > 9) digit = 9;
    uint8_t seg = tm_seg7[digit];
    tm_start();
    tm_write(0x40);       /* 数据命令: 写显示, 自动地址+1 */
    tm_stop();
    tm_start();
    tm_write(0xC0);       /* 起始地址: 第 0 位 */
    tm_write(seg);        /* 位 0 */
    tm_write(seg);        /* 位 1 */
    tm_write(seg);        /* 位 2 */
    tm_write(seg);        /* 位 3 */
    tm_stop();
    tm_start();
    tm_write(0x88 | 7);   /* 开显示, 亮度 7 (最亮) */
    tm_stop();
}

/* ============================================================
 *  灰度传感器扫描 — 返回 bitmask
 *    bit0=L1 bit1=L2 bit2=M bit3=R1 bit4=R2
 *    1 = 检测到黑线
 * ============================================================ */

/* ============================================================
 *  灰度传感器多次采样 + 多数表决 (v6.2)
 *
 *  目的: 补偿硬件灵敏度不足 (检测延迟 / 压线不触发 / 边缘模糊)
 *
 *  做法: 每次调用连读 N 次 (间隔 NOP), 每路传感器累计命中次数
 *        ≥ V6_SENSOR_THRESHOLD 次算真命中 (多数表决)
 *
 *  调参:
 *    V6_SENSOR_SAMPLES    采样次数 (越多越稳但越慢, 奇数便于表决)
 *    V6_SENSOR_THRESHOLD  命中阈值 (≥ 半数)
 *    V6_SENSOR_DELAY_NOP  采样间隔 NOP 数 (硬件响应慢时调大)
 * ============================================================ */
#define V6_SENSOR_SAMPLES    (5u)
#define V6_SENSOR_THRESHOLD  (3u)
#define V6_SENSOR_DELAY_NOP  (20u)

static inline void sensor_delay(void)
{
    for (volatile uint32_t i = 0; i < V6_SENSOR_DELAY_NOP; i++) {
        __NOP();
    }
}

static uint8_t read_line_sensors(void)
{
    uint8_t l1_cnt = 0, l2_cnt = 0, m_cnt = 0, r1_cnt = 0, r2_cnt = 0;

    /* 多次采样 + 累计 */
    for (uint8_t i = 0; i < V6_SENSOR_SAMPLES; i++) {
        uint32_t a = DL_GPIO_readPins(GPIOA, LINE_PINS_A_MASK);
        uint32_t b = DL_GPIO_readPins(GPIOB, LINE_PINS_B_MASK);

        uint8_t l1 = (b & L1_PIN) ? 1 : 0;
        uint8_t l2 = (a & L2_PIN) ? 1 : 0;
        uint8_t m  = (a & M_PIN)  ? 1 : 0;
        uint8_t r1 = (a & R1_PIN) ? 1 : 0;
        uint8_t r2 = (a & R2_PIN) ? 1 : 0;

#if LINE_ACTIVE_LEVEL == 1
        l1 ^= 1; l2 ^= 1; m ^= 1; r1 ^= 1; r2 ^= 1;
#endif

        l1_cnt += l1; l2_cnt += l2; m_cnt += m;
        r1_cnt += r1; r2_cnt += r2;

        if (i < V6_SENSOR_SAMPLES - 1) sensor_delay();
    }

    /* 多数表决 */
    uint8_t l1 = (l1_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t l2 = (l2_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t m  = (m_cnt  >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t r1 = (r1_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;
    uint8_t r2 = (r2_cnt >= V6_SENSOR_THRESHOLD) ? 1 : 0;

    return (l1 << 0) | (l2 << 1) | (m << 2) | (r1 << 3) | (r2 << 4);
}

/* ============================================================
 *  最简查表控制 — 5 路 bits → (duty_l, duty_r)
 *
 *  bits 约定: bit0=L1(最左) bit1=L2 bit2=M bit3=R1 bit4=R2(最右)
 *
 *  常见 pattern:
 *    0b00100  M 单亮         → 直行
 *    0b00010  L2 单亮        → 小左转 (车往线方向左转)
 *    0b01000  R1 单亮        → 小右转
 *    0b00001  L1 单亮        → 大左转
 *    0b10000  R2 单亮        → 大右转
 *
 *  转向约定 (重要):
 *    err<0 (线在左) → vL < vR (左轮慢, 右轮快) → 车头左转追线
 *    err>0 (线在右) → vL > vR (左轮快, 右轮慢) → 车头右转追线
 * ============================================================ */

static void line_control_simple(uint8_t bits)
{
    uint8_t dl, dr;
    switch (bits) {
        /* 直行 — 中线主导 */
        case 0b00100:                      /* M */
        case 0b01110:                      /* L2+M+R1 */
        case 0b11111:                      /* 全亮 (十字) */
            dl = SPEED_L_BASE; dr = SPEED_R_BASE; break;

        /* 小左转 — 线偏左 (L2 或 L2+M 亮), 左轮慢 */
        case 0b00010:
        case 0b00110:
            dl = SPEED_TURN_SOFT; dr = SPEED_R_BASE; break;

        /* 小右转 — 线偏右 (R1 或 M+R1 亮), 右轮慢 */
        case 0b01000:
        case 0b01100:
            dl = SPEED_L_BASE; dr = SPEED_TURN_SOFT; break;

        /* 大左转 — 线大偏左 (L1 或 L1+L2 亮), 左轮更慢 */
        case 0b00001:
        case 0b00011:
            dl = SPEED_TURN_HARD; dr = SPEED_R_BASE; break;

        /* 大右转 — 线大偏右 (R2 或 R1+R2 亮), 右轮更慢 */
        case 0b10000:
        case 0b11000:
            dl = SPEED_L_BASE; dr = SPEED_TURN_HARD; break;

        /* 其他罕见 pattern — 取外侧主导 */
        default:
            if      (bits & 0b10000) { dl = SPEED_L_BASE; dr = SPEED_TURN_HARD; }
            else if (bits & 0b00001) { dl = SPEED_TURN_HARD; dr = SPEED_R_BASE; }
            else                      { dl = SPEED_L_BASE; dr = SPEED_R_BASE; }
            break;
    }

    pwm_l_duty = dl;
    pwm_r_duty = dr;
    pwm_l_dir  = 1;
    pwm_r_dir  = 1;
    motor_apply_dir();
}

/* ============================================================
 *  v6.3 查表法 + 夹角修正 + 方向锁 + 直角原地旋转
 *
 *  ▶ NORMAL 阶段:
 *    bits → err (-2..+2) 最外优先
 *    场景 1: M 单独命中 → 直行
 *    场景 2: 丢线 → 按 last_err 方向追线
 *    场景 3: 次外命中 (err=±1) → 夹角修正 (dt 映射差速) + 方向锁
 *    场景 4: 其他 → 查表
 *
 *  ▶ CORNER_SPIN 阶段 (直角原地旋转):
 *    触发: 最外两路 (R2+R1 或 L2+L1) 同帧亮 或 50ms 窗口内都命中
 *    动作: 两轮等速反向 = 原地旋转
 *    退出: M 单独命中 或 超时
 * ============================================================ */

#define V6_CORNER_WINDOW_MS  (50u)         /* 最外两路凑齐窗口 */
#define V6_CORNER_TIMEOUT_MS (1500u)       /* 原地旋转超时兜底 */
#define V6_SPIN_DUTY         (3u)          /* 原地旋转 duty */
#define V6_DT_FAST_MS        (50u)         /* Δt < 此值 = 急偏 → 强修正 */
#define V6_DT_SLOW_MS        (200u)        /* Δt > 此值 = 缓偏 → 轻微修正 */
#define V6_CORRECT_ESCALATE  (3u)          /* 连续修正超过此数 = 强制升级 */

static int8_t  s_v6_last_err = 0;          /* 丢线时复用方向 */
static uint32_t s_v6_m_only_tick = 0;      /* 最后一次 M 单独命中的时间戳 */
static bool     s_v6_has_m_baseline = false;
static uint8_t  s_v6_correct_count = 0;    /* 连续修正次数 */
static int8_t  s_v6_correct_dir = 0;       /* 方向锁: +1/-1/0 */

typedef enum {
    V6_PHASE_NORMAL = 0,
    V6_PHASE_CORNER_SPIN,
} V6_Phase_t;
static V6_Phase_t s_v6_phase = V6_PHASE_NORMAL;
static uint32_t   s_v6_spin_start_tick = 0;
static int8_t     s_v6_spin_dir = 0;

/* 各传感器最近命中时间戳 (用于直角检测) */
static uint32_t s_v6_l2_tick = 0;
static uint32_t s_v6_l1_tick = 0;
static uint32_t s_v6_r1_tick = 0;
static uint32_t s_v6_r2_tick = 0;

static void line_control_v6(uint8_t bits)
{
    int8_t err;
    uint8_t dl, dr;
    uint32_t now = g_system_ms;

    /* ★ 直角转弯状态: 原地旋转中 */
    if (s_v6_phase == V6_PHASE_CORNER_SPIN) {
        /* 退出条件 1: M 单独命中 = 车头转够 90 度, 线回到 M */
        if (bits == 0b00100) {
            s_v6_phase = V6_PHASE_NORMAL;
            s_v6_correct_dir = 0;
            s_v6_correct_count = 0;
            s_v6_m_only_tick = now;
            /* 落入下面 NORMAL 处理 */
        }
        /* 退出条件 2: 超时兜底 */
        else if (now - s_v6_spin_start_tick > V6_CORNER_TIMEOUT_MS) {
            s_v6_phase = V6_PHASE_NORMAL;
            s_v6_correct_dir = 0;
            /* 落入下面 NORMAL 处理 */
        }
        /* 否则继续原地旋转 */
        else {
            if (s_v6_spin_dir > 0) {
                pwm_l_duty = V6_SPIN_DUTY; pwm_r_duty = V6_SPIN_DUTY;
                pwm_l_dir = 1; pwm_r_dir = 0;
            } else {
                pwm_l_duty = V6_SPIN_DUTY; pwm_r_duty = V6_SPIN_DUTY;
                pwm_l_dir = 0; pwm_r_dir = 1;
            }
            motor_apply_dir();
            return;
        }
    }

    /* ★ 更新各传感器命中时间戳 (用于直角检测)
     *   位分配 (与 read_line_sensors 一致): bit0=L1 bit1=L2 bit2=M bit3=R1 bit4=R2 */
    if (bits & 0b00001) s_v6_l1_tick = now;   /* bit0 = L1 内侧左 */
    if (bits & 0b00010) s_v6_l2_tick = now;   /* bit1 = L2 外侧左 */
    if (bits & 0b01000) s_v6_r1_tick = now;
    if (bits & 0b10000) s_v6_r2_tick = now;

    /* ★ 直角检测: 最外两路 (R2+R1 或 L2+L1) 同帧亮 或 50ms 窗口内都命中过 */
    bool r1_r2_now = (bits & 0b11000) == 0b11000;
    bool l1_l2_now = (bits & 0b00011) == 0b00011;
    bool right_corner = r1_r2_now ||
                        (s_v6_r2_tick > 0 && (now - s_v6_r2_tick) <= V6_CORNER_WINDOW_MS &&
                         s_v6_r1_tick > 0 && (now - s_v6_r1_tick) <= V6_CORNER_WINDOW_MS);
    bool left_corner = l1_l2_now ||
                       (s_v6_l2_tick > 0 && (now - s_v6_l2_tick) <= V6_CORNER_WINDOW_MS &&
                        s_v6_l1_tick > 0 && (now - s_v6_l1_tick) <= V6_CORNER_WINDOW_MS);

    if (right_corner || left_corner) {
        s_v6_phase = V6_PHASE_CORNER_SPIN;
        s_v6_spin_start_tick = now;
        s_v6_spin_dir = right_corner ? +1 : -1;
        s_v6_l2_tick = s_v6_l1_tick = s_v6_r1_tick = s_v6_r2_tick = 0;
        s_v6_correct_dir = 0;
        s_v6_correct_count = 0;

        if (s_v6_spin_dir > 0) {
            pwm_l_duty = V6_SPIN_DUTY; pwm_r_duty = V6_SPIN_DUTY;
            pwm_l_dir = 1; pwm_r_dir = 0;
        } else {
            pwm_l_duty = V6_SPIN_DUTY; pwm_r_duty = V6_SPIN_DUTY;
            pwm_l_dir = 0; pwm_r_dir = 1;
        }
        motor_apply_dir();
        return;
    }

    /* 场景 1: M 单独命中 = 直行 */
    if (bits == 0b00100) {
        s_v6_m_only_tick = now;
        s_v6_has_m_baseline = true;
        s_v6_last_err = 0;
        s_v6_correct_count = 0;
        s_v6_correct_dir = 0;
        pwm_l_duty = 3; pwm_r_duty = 3;
        pwm_l_dir = 1;  pwm_r_dir = 1;
        motor_apply_dir();
        return;
    }

    /* 场景 2: 丢线 → 按 last_err 方向追线 */
    if (bits == 0) {
        err = s_v6_last_err;
        if      (err < 0) { dl = 1; dr = 3; }
        else if (err > 0) { dl = 3; dr = 1; }
        else              { dl = 3; dr = 3; }
        pwm_l_duty = dl; pwm_r_duty = dr;
        pwm_l_dir = 1; pwm_r_dir = 1;
        motor_apply_dir();
        return;
    }

    /* bits → err (最外优先 + 正确位分配)
     *   位分配: bit0=L1 bit1=L2 bit2=M bit3=R1 bit4=R2
     *   L2(外侧左)=-2 严重偏, L1(内侧左)=-1 轻微偏
     *   R1(内侧右)=+1 轻微偏, R2(外侧右)=+2 严重偏 */
    if      (bits & 0b00010) err = -2;   /* bit1 = L2 外侧左 */
    else if (bits & 0b10000) err = +2;   /* bit4 = R2 外侧右 */
    else if (bits & 0b00001) err = -1;   /* bit0 = L1 内侧左 */
    else if (bits & 0b01000) err = +1;   /* bit3 = R1 内侧右 */
    else                      err =  0;

    s_v6_last_err = err;

    /* 场景 3: 次外命中 (err=±1) 且 M 也命中 且 有基准 → 夹角修正 + 方向锁 */
    bool m_active = (bits & 0b00100) != 0;
    if (m_active && (err == -1 || err == +1) && s_v6_has_m_baseline) {
        int8_t needed_dir = (err < 0) ? +1 : -1;

        /* 方向锁: 反向请求 → 直行等回正 */
        if (s_v6_correct_dir != 0 && s_v6_correct_dir != needed_dir) {
            pwm_l_duty = 3; pwm_r_duty = 3;
            pwm_l_dir = 1;  pwm_r_dir = 1;
            motor_apply_dir();
            return;
        }

        s_v6_correct_dir = needed_dir;
        s_v6_correct_count++;
        uint32_t dt = now - s_v6_m_only_tick;
        if (dt < 1) dt = 1;

        bool escalate = (s_v6_correct_count >= V6_CORRECT_ESCALATE);

        if (escalate || dt < V6_DT_FAST_MS) {
            /* 强修正: 内轮 1, 外轮 3, 差速 2 档 */
            if (err < 0) { dl = 1; dr = 3; }
            else         { dl = 3; dr = 1; }
        } else if (dt < V6_DT_SLOW_MS) {
            /* 中修正: 内轮 2, 外轮 3, 差速 1 档 */
            if (err < 0) { dl = 2; dr = 3; }
            else         { dl = 3; dr = 2; }
        } else {
            /* 缓偏: 几乎直行 */
            dl = 3; dr = 3;
        }
    }
    /* 场景 4: 其他 (最外命中 / M 不在 / 无基准) → 查表 */
    else {
        switch (err) {
            case -2:  dl = 1; dr = 3; break;
            case -1:  dl = 2; dr = 3; break;
            case  0:  dl = 3; dr = 3; break;
            case +1:  dl = 3; dr = 2; break;
            case +2:  dl = 3; dr = 1; break;
            default:  dl = 3; dr = 3; break;
        }
    }

    pwm_l_duty = dl;
    pwm_r_duty = dr;
    pwm_l_dir  = 1;
    pwm_r_dir  = 1;
    motor_apply_dir();
}

/* ============================================================
 *  状态机
 *    WAIT_LINE : 启动后还没见过线, 静止等
 *    TRACKING  : 检测到线, 查表控制
 *    LOST      : 临时丢线, 按 last_bits 方向缓行 (永不全停)
 *    ERROR     : 持续丢线 > 3s, 全停
 * ============================================================ */

typedef enum { ST_WAIT_LINE = 0, ST_TRACKING, ST_LOST, ST_ERROR } track_state_t;
static track_state_t s_state = ST_WAIT_LINE;
static uint32_t      s_state_ts;

static const char *state_name(track_state_t s)
{
    switch (s) {
    case ST_WAIT_LINE: return "WAIT_LINE";
    case ST_TRACKING:  return "TRACKING";
    case ST_LOST:      return "LOST";
    case ST_ERROR:     return "ERROR";
    }
    return "?";
}

static void enter_state(track_state_t s)
{
    s_state = s;
    s_state_ts = g_system_ms;
}

/* ============================================================
 *  主循环
 * ============================================================ */

int main(void)
{
    SYSCFG_DL_init();

    /* 配置电机引脚 */
    DL_GPIO_initDigitalOutput(AIN1_PINCM);
    DL_GPIO_initDigitalOutput(AIN2_PINCM);
    DL_GPIO_initDigitalOutput(BIN1_PINCM);
    DL_GPIO_initDigitalOutput(BIN2_PINCM);
    DL_GPIO_initDigitalOutput(PWMA_PINCM);
    DL_GPIO_initDigitalOutput(PWMB_PINCM);
    DL_GPIO_enableOutput(GPIOA, DIR_PINS_MASK);
    DL_GPIO_enableOutput(GPIOB, PWM_PINS_MASK);
    DL_GPIO_clearPins(GPIOA, DIR_PINS_MASK);
    DL_GPIO_clearPins(GPIOB, PWM_PINS_MASK);

#if 0  /* 灰度传感器未装, 注释释放引脚 (PB17/PA12 给数码管, 其余暂留) */
    /* 配置 5 路灰度传感器引脚 (数字输入 + 上拉) */
    DL_GPIO_initDigitalInputFeatures(L1_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(L2_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(M_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R1_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(R2_PINCM,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
#endif

    /* TM1637 数码管: CLK=PB17, DIO=PA12 (推挽输出, 空闲高) */
    DL_GPIO_initDigitalOutput(TM_CLK_PINCM);
    DL_GPIO_initDigitalOutput(TM_DIO_PINCM);
    DL_GPIO_enableOutput(TM_CLK_PORT, TM_CLK_PIN);
    DL_GPIO_enableOutput(TM_DIO_PORT, TM_DIO_PIN);
    DL_GPIO_setPins(TM_CLK_PORT, TM_CLK_PIN);
    DL_GPIO_setPins(TM_DIO_PORT, TM_DIO_PIN);

    /* OLED (SSD1306 软件 SPI): SCL=PA28 SDA=PA31 RES=PB14 DC=PB15, CS 接 GND
     * 4 脚推挽输出, 空闲全高 (引脚配置不需要 SysTick) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM3);    /* PA28 SCL */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM6);    /* PA31 SDA */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM31);   /* PB14 RES */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM32);   /* PB15 DC  */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_28 | DL_GPIO_PIN_31);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_14 | DL_GPIO_PIN_15);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_28 | DL_GPIO_PIN_31);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14 | DL_GPIO_PIN_15);

    SysTick_Config(SYSTICK_LOAD);
    NVIC_SetPriority(SysTick_IRQn, 0);
    NVIC_EnableIRQ(SysTick_IRQn);

    /* OLED 初始化 (需 SysTick 已启动, 内部用 g_system_ms 做复位延时) */
    OLED_Init();
    OLED_DrawString(0, 0, "tracer-car");
    OLED_DrawString(0, 1, "OLED OK");
    OLED_Refresh();
    uart_print("OLED init ok\r\n");

    /* USER 按键 (PB8) 初始化 */
    Button_Init();
    uart_print("button (PB8) init ok\r\n");

#if LINE_FOLLOW_VERSION == 8
    LineTracker_Init();
    uart_print("\r\n\r\n=== tracer-car LINE_FOLLOW v8 (5x grayscale, weighted-PD + gain scheduling) ===\r\n");
    uart_print("sensor: L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9\r\n");
    uart_print("motor : AIN1=PA13 AIN2=PA14 PWMA=PB2 | BIN1=PA17 BIN2=PA16 PWMB=PB3\r\n");
    uart_print("algo  : weighted_position + 5-gear PD + 6-state FSM (lowest speed for tuning)\r\n");
    uart_print("waiting for line...\r\n");
#elif LINE_FOLLOW_VERSION == 6
    uart_print("\r\n\r\n=== tracer-car LINE_FOLLOW v6 (5x grayscale, 5-err lookup table) ===\r\n");
    uart_print("sensor: L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9\r\n");
    uart_print("motor : AIN1=PA13 AIN2=PA14 PWMA=PB2 | BIN1=PA17 BIN2=PA16 PWMB=PB3\r\n");
    uart_print("algo  : 5-err lookup (err -2/-1/0/+1/+2 → vL/vR), lowest speed base=2\r\n");
    uart_print("waiting for line...\r\n");
#else
    uart_print("\r\n\r\n=== tracer-car LINE_FOLLOW v7 (5x grayscale, lookup table) ===\r\n");
    uart_print("sensor: L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9\r\n");
    uart_print("motor : AIN1=PA13 AIN2=PA14 PWMA=PB2 | BIN1=PA17 BIN2=PA16 PWMB=PB3\r\n");
#if MOTOR_TEST_DRIVE
    uart_print("*** MOTOR_TEST_DRIVE=1 — 测试模式, 两轮固定直行 ***\r\n");
#else
    uart_print("algo  : lookup table (8 patterns → duty pairs)\r\n");
    uart_print("waiting for line...\r\n");
#endif
#endif

    uint32_t boot_ms = g_system_ms;
    uint32_t btn_count = 0;     /* USER 按键按下次数 */

    while (1) {
        if (!s_ctrl_flag) {
            __WFI();
            continue;
        }
        s_ctrl_flag = 0;

        uint32_t now = g_system_ms;

        /* USER 按键: 检测到一次按下就计数 + 串口打印 */
        if (Button_Consume()) {
            btn_count++;
            uart_print("button press cnt=");
            uart_print_u32(btn_count);
            uart_print("\r\n");
        }

#if OLED_TEST
        /* OLED 自检: 每 200ms 把运行秒数刷到屏 (不阻断, 数码管/循迹照常) */
        {
            static uint32_t oled_last = 0;
            if (now - oled_last >= 200) {
                oled_last = now;
                uint32_t sec = now / 1000;
                char buf[12];
                int i = 11;
                buf[i--] = '\0';
                if (sec == 0) buf[i--] = '0';
                uint32_t v = sec;
                while (v && i >= 0) { buf[i--] = (char)('0' + (v % 10)); v /= 10; }
                OLED_DrawString(0, 2, "t =");
                OLED_DrawString(4, 2, &buf[i + 1]);
                OLED_DrawString(0, 4, "OLED running");
                /* 第 5 页: USER 按键计数 */
                OLED_DrawString(0, 5, "BTN:");
                {
                    int j = 11;
                    buf[j--] = '\0';
                    uint32_t w = btn_count;
                    if (w == 0) buf[j--] = '0';
                    while (w && j >= 0) { buf[j--] = (char)('0' + (w % 10)); w /= 10; }
                    OLED_DrawString(5, 5, &buf[j + 1]);
                }
                OLED_Refresh();
            }
        }
#endif

#if TM1637_TEST
        /* 数码管自检: 4 位同显 1→2→…→9→0, 每 500ms 切换; 电机停 */
        {
            static uint8_t  tm_d = 1;
            static uint32_t tm_last = 0;
            if (now - tm_last >= 500) {
                tm_last = now;
                tm_show_same(tm_d);
                uart_print("TM1637 show ");
                uart_print_u32(tm_d);
                uart_print("\r\n");
                tm_d = (uint8_t)((tm_d + 1) % 10);   /* 1→2→…→9→0→1 */
            }
            pwm_l_duty = 0; pwm_r_duty = 0;
            motor_apply_dir();
            continue;
        }
#endif

#if MOTOR_TEST_DRIVE
        {
            uint32_t elapsed = now - boot_ms;
            if (elapsed < 1500) {
                pwm_l_duty = 0; pwm_r_duty = 0;
            } else {
                pwm_l_duty = 2; pwm_r_duty = 2;
            }
            pwm_l_dir = 1; pwm_r_dir = 1;
            motor_apply_dir();

            uart_print("t=");
            uart_print_u32(now);
            uart_print(" [TEST] vL=");
            uart_print_u32(pwm_l_duty);
            uart_print(" vR=");
            uart_print_u32(pwm_r_duty);
            uart_print("\r\n");
            continue;
        }
#else

        uint8_t bits = read_line_sensors();
        s_line_raw = bits;

#if LINE_FOLLOW_VERSION == 8
        /* ====================================================
         *  LINE_FOLLOW v8 — 加权位置法 + 增益调度 PD + 状态机
         * ==================================================== */

        /* boot 后 1.5s 静止 (boot 延迟由 main.c 接管, 与算法解耦) */
        if (now - boot_ms < 1500) {
            pwm_l_duty = 0; pwm_r_duty = 0;
            pwm_l_dir  = 1; pwm_r_dir  = 1;
            motor_apply_dir();
        } else {
            /* 喂给 line_tracker, 拿回控制输出 */
            LineTracker_Update(bits, now);
            pwm_l_duty = LineTracker_GetLeftDuty();
            pwm_r_duty = LineTracker_GetRightDuty();
            pwm_l_dir  = LineTracker_GetLeftDir();
            pwm_r_dir  = LineTracker_GetRightDir();
            motor_apply_dir();
        }

        /* v8 日志 (10Hz): state/gear/bits/pos×10/vL/vR/L1..R2 */
        uart_print("t=");
        uart_print_u32(now);
        uart_print(" st=");
        uart_print_u32((uint32_t)LineTracker_GetState());
        uart_print(" g=");
        uart_print_u32((uint32_t)LineTracker_GetGear());
        uart_print(" bits=");
        uart_print_u32(bits);
        uart_print(" pos=");
        {
            int pos_x10 = (int)(LineTracker_GetPosition() * 10.0f);
            if (pos_x10 < 0) { uart_putc('-'); pos_x10 = -pos_x10; }
            uart_print_u32((uint32_t)pos_x10);
        }
        uart_print(" vL=");
        uart_print_u32(pwm_l_duty);
        uart_print(" vR=");
        uart_print_u32(pwm_r_duty);
        uart_print(" L1="); uart_putc('0' + ((bits >> 0) & 1));
        uart_print(" L2="); uart_putc('0' + ((bits >> 1) & 1));
        uart_print(" M=" ); uart_putc('0' + ((bits >> 2) & 1));
        uart_print(" R1="); uart_putc('0' + ((bits >> 3) & 1));
        uart_print(" R2="); uart_putc('0' + ((bits >> 4) & 1));
        uart_print("\r\n");

#elif LINE_FOLLOW_VERSION == 6
        /* ====================================================
         *  LINE_FOLLOW v6 — 5-档 err 查表法 (2026-07-07 灰度初版)
         * ==================================================== */

        if (now - boot_ms < 1500) {
            pwm_l_duty = 0; pwm_r_duty = 0;
            pwm_l_dir  = 1; pwm_r_dir  = 1;
            motor_apply_dir();
        } else {
            line_control_v6(bits);
        }

        /* v6 日志 (10Hz): err/bits/vL/vR/L1..R2 */
        {
            int8_t err = (bits == 0) ? s_v6_last_err :
                         ((bits & 0b00001) ? -2 :
                          (bits & 0b10000) ? +2 :
                          (bits & 0b00010) ? -1 :
                          (bits & 0b01000) ? +1 : 0);
            uart_print("t=");
            uart_print_u32(now);
            uart_print(" err=");
            if (err < 0) { uart_putc('-'); err = -err; }
            uart_print_u32((uint32_t)err);
            uart_print(" bits=");
            uart_print_u32(bits);
            uart_print(" vL=");
            uart_print_u32(pwm_l_duty);
            uart_print(" vR=");
            uart_print_u32(pwm_r_duty);
            uart_print(" L1="); uart_putc('0' + ((bits >> 0) & 1));
            uart_print(" L2="); uart_putc('0' + ((bits >> 1) & 1));
            uart_print(" M=" ); uart_putc('0' + ((bits >> 2) & 1));
            uart_print(" R1="); uart_putc('0' + ((bits >> 3) & 1));
            uart_print(" R2="); uart_putc('0' + ((bits >> 4) & 1));
            uart_print("\r\n");
        }

#else
        /* ====================================================
         *  LINE_FOLLOW v7 — 旧查表法 (回退用, 调参对比)
         * ==================================================== */

        if (bits != 0) {
            s_line_last_seen_ms = now;
            s_last_bits = bits;
        }

        uint32_t lost_duration = now - s_line_last_seen_ms;

        /* 启动延迟: boot 后 1.5s 不动 */
        if (now - boot_ms < 1500) {
            if (s_state != ST_WAIT_LINE) enter_state(ST_WAIT_LINE);
            pwm_l_duty = 0; pwm_r_duty = 0;
            motor_apply_dir();
        }
        else if (s_line_last_seen_ms == 0) {
            /* 还没见过线 */
            if (s_state != ST_WAIT_LINE) {
                enter_state(ST_WAIT_LINE);
                pwm_l_duty = 0; pwm_r_duty = 0;
                motor_apply_dir();
            }
        }
        else if (lost_duration > 3000) {
            /* 持续丢线 > 3s, 才全停 */
            if (s_state != ST_ERROR) {
                enter_state(ST_ERROR);
                pwm_l_duty = 0; pwm_r_duty = 0;
                motor_apply_dir();
            }
        }
        else if (bits == 0) {
            /* 临时丢线 — 按 last_bits 方向缓行 (不全停) */
            if (s_state != ST_LOST) enter_state(ST_LOST);
            uint8_t lb = s_last_bits;
            if (lb == 0) lb = 0b00100;   /* 实在没历史就直行兜底 */
            uint8_t slow_l = (lb & 0b00011) ? SPEED_TURN_HARD :
                             (lb & 0b11000) ? SPEED_L_BASE    : SPEED_TURN_SOFT;
            uint8_t slow_r = (lb & 0b11000) ? SPEED_TURN_HARD :
                             (lb & 0b00011) ? SPEED_R_BASE    : SPEED_TURN_SOFT;
            pwm_l_duty = slow_l; pwm_r_duty = slow_r;
            pwm_l_dir = 1; pwm_r_dir = 1;
            motor_apply_dir();
        }
        else {
            /* 正常循迹 — 查表控制 */
            if (s_state != ST_TRACKING) enter_state(ST_TRACKING);
            line_control_simple(bits);
        }

        /* 10Hz 日志 */
        uart_print("t=");
        uart_print_u32(now);
        uart_print(" state=");
        uart_print(state_name(s_state));
        uart_print(" bits=");
        uart_print_u32(bits);
        uart_print(" vL=");
        uart_print_u32(pwm_l_duty);
        uart_print(" vR=");
        uart_print_u32(pwm_r_duty);
        uart_print(" lost=");
        uart_print_u32(lost_duration);
        uart_print(" L1=");
        uart_putc('0' + ((bits >> 0) & 1));
        uart_print(" L2=");
        uart_putc('0' + ((bits >> 1) & 1));
        uart_print(" M=");
        uart_putc('0' + ((bits >> 2) & 1));
        uart_print(" R1=");
        uart_putc('0' + ((bits >> 3) & 1));
        uart_print(" R2=");
        uart_putc('0' + ((bits >> 4) & 1));
        uart_print("\r\n");
#endif /* USE_V8_ADVANCED */
#endif
    }
}

/* 空实现 — 防止 Module/ 里某些被链接器拉进来的符号找不到入口 */
void Port_OnTick1kHz(void)  {}
void Port_OnTick100Hz(void) {}
