/**
 * @file main.c
 * @brief tracer-car 主循环 — 循迹控制 + 状态机 + 测试编排
 *
 * 架构 (重构后, 硬件驱动已拆到 Module/):
 *   硬件驱动: motor_drive(软件PWM+方向) / line_sensor(灰度) / tm1637(数码管)
 *             / oled / button / uart_debug
 *   循迹算法: line_tracker(v8, Algorithm/) + v6/v7 查表(本文件)
 *   本文件:   SysTick 调度 + v6/v7 查表控制 + v7 主状态机 + 测试开关编排
 *
 * 传感器: 5 路灰度 L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9 (TM1637 数码管已停用)
 * 算法切换: LINE_FOLLOW_VERSION (6/7/8)
 */

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "config.h"
#include "line_tracker.h"
#include "oled.h"
#include "button.h"
#include "tm1637.h"
#include "motor_drive.h"
#include "line_sensor.h"
#include "uart_debug.h"

/* ============================================================
 *  LINE_FOLLOW 版本开关 (改一个值切算法)
 *    6 = v6 5-档 err 查表法 (最简单最稳)
 *    7 = v7 8-pattern 查表法 (加丢线超时全停)
 *    8 = v8 竞赛级 (加权位置法 + 5 档增益调度 PD + 状态机)
 * ============================================================ */
#ifndef LINE_FOLLOW_VERSION
#define LINE_FOLLOW_VERSION  6
#endif
#define USE_V8_ADVANCED  (LINE_FOLLOW_VERSION == 8)

/* ============================================================
 *  板载 LED + SysTick
 * ============================================================ */
#define LED_PIN         DL_GPIO_PIN_22   /* PB22 */
/* SysTick 2kHz @ 32MHz → reload = 16000-1 */
#define SYSTICK_LOAD    (16000u - 1u)

/* ============================================================
 *  测试模式开关
 * ============================================================ */
#define TM1637_TEST      0   /* =1 只跑数码管自检; =0 走循迹 (灰度已启用) */
#define OLED_TEST        1   /* =1 周期把运行秒数刷到屏 (不阻断主循环) */
#define MOTOR_TEST_DRIVE 0   /* =1 两轮固定直行 (验证电机用, 测完改 0) */

/* ============================================================
 *  查表控制 — 速度档位
 *  两轮有制造公差, 直行档拆左右两个, 实测后微调补偿机械差异:
 *    车往左偏 → 左轮快 → 减 SPEED_L_BASE 或加 SPEED_R_BASE
 *    车往右偏 → 右轮快 → 减 SPEED_R_BASE 或加 SPEED_L_BASE
 * ============================================================ */
#define SPEED_L_BASE      8u    /* 左轮直行基础档 (40%) */
#define SPEED_R_BASE      8u    /* 右轮直行基础档 (40%) */
#define SPEED_TURN_SOFT   6u    /* 小转内侧档 (30%) */
#define SPEED_TURN_HARD   4u    /* 大转内侧档 (20%) */

/* ============================================================
 *  全局状态
 * ============================================================ */
volatile uint32_t g_system_ms = 0;   /* 毫秒时钟 (其他模块 extern 引用) */

static volatile uint8_t s_ctrl_flag;       /* 100Hz 主循环就绪标志 */
static volatile uint8_t s_tick_100ms;

static volatile uint8_t  s_line_raw;        /* v7: 最近一次灰度原始读数 */
static volatile uint32_t s_line_last_seen_ms;
static uint8_t s_last_bits = 0;             /* v7: 上一次有效 bits (丢线时复用方向) */

/* ============================================================
 *  SysTick 2kHz — 系统 ms + 软件 PWM + 10Hz tick + LED
 * ============================================================ */
void SysTick_Handler(void)
{
    static uint8_t ms_prescaler = 0;
    if (++ms_prescaler >= 2) {
        ms_prescaler = 0;
        g_system_ms++;
    }

    MotorDrive_Tick();   /* 软件 PWM (模块内, 仅 GPIO 翻转, 中断安全) */

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
 *  v7 查表控制 — 5 路 bits → (duty_l, duty_r)
 *
 *  bits 约定: bit0=L1(最左) bit1=L2 bit2=M bit3=R1 bit4=R2(最右)
 *  转向约定: err<0(线在左) → vL<vR 左转追线; err>0(线在右) → vL>vR 右转追线
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

        /* 小左转 — 线偏左, 左轮慢 */
        case 0b00010:
        case 0b00110:
            dl = SPEED_TURN_SOFT; dr = SPEED_R_BASE; break;

        /* 小右转 — 线偏右, 右轮慢 */
        case 0b01000:
        case 0b01100:
            dl = SPEED_L_BASE; dr = SPEED_TURN_SOFT; break;

        /* 大左转 — 线大偏左, 左轮更慢 */
        case 0b00001:
        case 0b00011:
            dl = SPEED_TURN_HARD; dr = SPEED_R_BASE; break;

        /* 大右转 — 线大偏右, 右轮更慢 */
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

    MotorDrive_Set(dl, 1, dr, 1);
}

/* ============================================================
 *  v6.3 查表法 + 夹角修正 + 方向锁 + 直角原地旋转
 *
 *  NORMAL: bits→err(-2..+2) 最外优先; M单亮直行 / 丢线追线 / 次外夹角修正 / 查表
 *  CORNER_SPIN: 最外两路凑齐触发, 两轮等速反向原地旋转, M单亮或超时退出
 * ============================================================ */
#define V6_CORNER_WINDOW_MS  (50u)
#define V6_CORNER_TIMEOUT_MS (1500u)
#define V6_SPIN_DUTY         (3u)
#define V6_DT_FAST_MS        (50u)
#define V6_DT_SLOW_MS        (200u)
#define V6_CORRECT_ESCALATE  (3u)
#define V6_TURN_PULSE_MS     (50u)  /* 场景4 转弯脉冲: err变化后内侧强打方向持续时间 */

static int8_t   s_v6_last_err = 0;
static uint32_t s_v6_m_only_tick = 0;
static bool     s_v6_has_m_baseline = false;
static uint8_t  s_v6_correct_count = 0;
static int8_t   s_v6_correct_dir = 0;
static int8_t   s_v6_prev_err = 0;       /* 场景4: 上次 err (检测变化触发脉冲) */
static uint32_t s_v6_err_change_ms = 0;  /* 场景4: err 最近变化时刻 */

typedef enum {
    V6_PHASE_NORMAL = 0,
    V6_PHASE_CORNER_SPIN,
} V6_Phase_t;
static V6_Phase_t s_v6_phase = V6_PHASE_NORMAL;
static uint32_t   s_v6_spin_start_tick = 0;
static int8_t     s_v6_spin_dir = 0;

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
        if (bits == 0b00100) {                 /* M 单亮 = 转够 90° */
            s_v6_phase = V6_PHASE_NORMAL;
            s_v6_correct_dir = 0;
            s_v6_correct_count = 0;
            s_v6_m_only_tick = now;
            /* 落入下面 NORMAL */
        } else if (now - s_v6_spin_start_tick > V6_CORNER_TIMEOUT_MS) {  /* 超时兜底 */
            s_v6_phase = V6_PHASE_NORMAL;
            s_v6_correct_dir = 0;
            /* 落入下面 NORMAL */
        } else {                              /* 继续原地旋转 */
            if (s_v6_spin_dir > 0) MotorDrive_Set(V6_SPIN_DUTY, 1, V6_SPIN_DUTY, 0);
            else                   MotorDrive_Set(V6_SPIN_DUTY, 0, V6_SPIN_DUTY, 1);
            return;
        }
    }

    /* 更新各传感器命中时间戳 (位分配: bit0=L1 bit1=L2 bit2=M bit3=R1 bit4=R2) */
    if (bits & 0b00001) s_v6_l1_tick = now;
    if (bits & 0b00010) s_v6_l2_tick = now;
    if (bits & 0b01000) s_v6_r1_tick = now;
    if (bits & 0b10000) s_v6_r2_tick = now;

    /* 直角检测: 最外两路同帧亮 或 50ms 窗口内都命中过 */
    bool r1_r2_now = (bits & 0b11000) == 0b11000;
    bool l1_l2_now = (bits & 0b00011) == 0b00011;
    bool right_corner = r1_r2_now ||
                        (s_v6_r2_tick > 0 && (now - s_v6_r2_tick) <= V6_CORNER_WINDOW_MS &&
                         s_v6_r1_tick > 0 && (now - s_v6_r1_tick) <= V6_CORNER_WINDOW_MS);
    bool left_corner = l1_l2_now ||
                       (s_v6_l2_tick > 0 && (now - s_v6_l2_tick) <= V6_CORNER_WINDOW_MS &&
                        s_v6_l1_tick > 0 && (now - s_v6_l1_tick) <= V6_CORNER_WINDOW_MS);

    if (0 && (right_corner || left_corner)) {   /* 直角禁用 (不考虑直角道, 6 路位重映射后待适配) */
        s_v6_phase = V6_PHASE_CORNER_SPIN;
        s_v6_spin_start_tick = now;
        s_v6_spin_dir = right_corner ? +1 : -1;
        s_v6_l2_tick = s_v6_l1_tick = s_v6_r1_tick = s_v6_r2_tick = 0;
        s_v6_correct_dir = 0;
        s_v6_correct_count = 0;
        if (s_v6_spin_dir > 0) MotorDrive_Set(V6_SPIN_DUTY, 1, V6_SPIN_DUTY, 0);
        else                   MotorDrive_Set(V6_SPIN_DUTY, 0, V6_SPIN_DUTY, 1);
        return;
    }

    /* 场景 1: 中间 S3/S4 命中且最外无 = 直行 */
    if ((bits & 0b001100) && !(bits & 0b110011)) {
        s_v6_m_only_tick = now;
        s_v6_has_m_baseline = true;
        s_v6_last_err = 0;
        s_v6_correct_count = 0;
        s_v6_correct_dir = 0;
        MotorDrive_Set(5, 1, 5, 1);
        return;
    }

    /* 场景 2: 丢线 → 按 last_err 方向追线 */
    if (bits == 0) {
        err = s_v6_last_err;
        if      (err < 0) { dl = 1; dr = 5; }
        else if (err > 0) { dl = 5; dr = 1; }
        else              { dl = 5; dr = 5; }
        MotorDrive_Set(dl, 1, dr, 1);
        return;
    }

    /* bits → err (6 路, 最外优先; bit0=S1最左 .. bit5=S6最右) */
    if      (bits & 0b000001) err = -2;   /* S1 最左 */
    else if (bits & 0b100000) err = +2;   /* S6 最右 */
    else if (bits & 0b000010) err = -1;   /* S2 次左 */
    else if (bits & 0b010000) err = +1;   /* S5 次右 */
    else if (bits & 0b001100) err =  0;   /* S3/S4 中间 = 直行 */
    else                      err =  0;

    err = (int8_t)(-err);   /* 转向修正 (左碰→右转, 右碰→左转) */

    s_v6_last_err = err;

    /* 场景 3: 次外命中 (err=±1) 且 M 也命中 且 有基准 → 夹角修正 + 方向锁 */
    bool m_active = (bits & 0b001100) != 0;   /* S3/S4 中间任一活跃 */
    if (m_active && (err == -1 || err == +1) && s_v6_has_m_baseline) {
        int8_t needed_dir = (err < 0) ? +1 : -1;

        if (s_v6_correct_dir != 0 && s_v6_correct_dir != needed_dir) {
            MotorDrive_Set(5, 1, 5, 1);   /* 反向请求 → 直行等回正 */
            return;
        }

        s_v6_correct_dir = needed_dir;
        s_v6_correct_count++;
        uint32_t dt = now - s_v6_m_only_tick;
        if (dt < 1) dt = 1;
        bool escalate = (s_v6_correct_count >= V6_CORRECT_ESCALATE);

        if (escalate || dt < V6_DT_FAST_MS) {
            if (err < 0) { dl = 3; dr = 5; }
            else         { dl = 5; dr = 3; }
        } else if (dt < V6_DT_SLOW_MS) {
            if (err < 0) { dl = 4; dr = 5; }
            else         { dl = 5; dr = 4; }
        } else {
            dl = 5; dr = 5;
        }
    }
    /* 场景 4: 大偏离 → 脉冲+阻尼 (err变化瞬间内侧强打, 之后阻尼温和修正) */
    else {
        if (err != s_v6_prev_err) {            /* err 变化 → 重置脉冲计时 */
            s_v6_err_change_ms = now;
            s_v6_prev_err = err;
        }
        /* 脉冲(< PULSE_MS)内侧1强打; 阻尼期: 大弯内侧2(略急), 小弯内侧3(温和) */
        uint8_t damp = (err == -2 || err == +2) ? 3 : 4;
        uint8_t inner = ((now - s_v6_err_change_ms) < V6_TURN_PULSE_MS) ? 2 : damp;
        switch (err) {
            case -2:  dl = inner; dr = 5; break;
            case -1:  dl = inner; dr = 5; break;
            case  0:  dl = 5; dr = 5; break;
            case +1:  dl = 5; dr = inner; break;
            case +2:  dl = 5; dr = inner; break;
            default:  dl = 5; dr = 5; break;
        }
    }

    MotorDrive_Set(dl, 1, dr, 1);
}

/* ============================================================
 *  v7 主状态机
 *    WAIT_LINE: 启动后还没见过线, 静止等
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

    MotorDrive_Init();          /* 电机引脚 (AIN/AIN2/BIN1/2 + PWMA/B) */

    LineSensor_Init();          /* 5 路灰度: L1=PB17 L2=PA12 M=PA22 R1=PA27 R2=PA9 */

#if 0  /* 数码管已停用 (PB17/PA12 还给灰度 L1/L2); 需要时改回 1 */
    TM1637_Init();
#endif

    /* OLED (SSD1306 软件 SPI): SCL=PA28 SDA=PA31 RES=PB14 DC=PB15, CS 接 GND */
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
    UART_Print("OLED init ok\r\n");

    /* USER 按键 (PB8) */
    Button_Init();
    UART_Print("button (PB8) init ok\r\n");

#if LINE_FOLLOW_VERSION == 8
    LineTracker_Init();
    UART_Print("\r\n=== tracer-car LINE_FOLLOW v8 (weighted-PD + gain scheduling) ===\r\n");
    UART_Print("waiting for line...\r\n");
#elif LINE_FOLLOW_VERSION == 6
    UART_Print("\r\n=== tracer-car LINE_FOLLOW v6 (5-err lookup) ===\r\n");
    UART_Print("waiting for line...\r\n");
#else
    UART_Print("\r\n=== tracer-car LINE_FOLLOW v7 (lookup) ===\r\n");
#if MOTOR_TEST_DRIVE
    UART_Print("*** MOTOR_TEST_DRIVE=1 — 两轮固定直行 ***\r\n");
#else
    UART_Print("waiting for line...\r\n");
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
            UART_Print("button press cnt=");
            UART_PrintU32(btn_count);
            UART_Print("\r\n");
        }

#if OLED_TEST
        /* OLED 自检: 每 200ms 把运行秒数 + 按键计数刷到屏 (不阻断) */
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
                TM1637_ShowSame(tm_d);
                UART_Print("TM1637 show ");
                UART_PrintU32(tm_d);
                UART_Print("\r\n");
                tm_d = (uint8_t)((tm_d + 1) % 10);
            }
            MotorDrive_Stop();
            continue;
        }
#endif

#if MOTOR_TEST_DRIVE
        {
            uint32_t elapsed = now - boot_ms;
            if (elapsed < 1500) MotorDrive_Stop();
            else                MotorDrive_Set(2, 1, 2, 1);

            UART_Print("t=");
            UART_PrintU32(now);
            UART_Print(" [TEST] vL=");
            UART_PrintU32(MotorDrive_GetLeftDuty());
            UART_Print(" vR=");
            UART_PrintU32(MotorDrive_GetRightDuty());
            UART_Print("\r\n");
            continue;
        }
#else

        uint8_t bits = LineSensor_Read();
        s_line_raw = bits;

#if LINE_FOLLOW_VERSION == 8
        /* ============ v8: 加权位置法 + 增益调度 PD + 状态机 ============ */
        if (now - boot_ms < 1500) {
            MotorDrive_Stop();                                  /* boot 后 1.5s 静止 */
        } else {
            LineTracker_Update(bits, now);
            MotorDrive_Set(LineTracker_GetLeftDuty(),  LineTracker_GetLeftDir(),
                           LineTracker_GetRightDuty(), LineTracker_GetRightDir());
        }

        UART_Print("t=");   UART_PrintU32(now);
        UART_Print(" st=");  UART_PrintU32((uint32_t)LineTracker_GetState());
        UART_Print(" g=");   UART_PrintU32((uint32_t)LineTracker_GetGear());
        UART_Print(" bits="); UART_PrintU32(bits);
        UART_Print(" pos=");
        {
            int pos_x10 = (int)(LineTracker_GetPosition() * 10.0f);
            if (pos_x10 < 0) { UART_Putc('-'); pos_x10 = -pos_x10; }
            UART_PrintU32((uint32_t)pos_x10);
        }
        UART_Print(" vL=");  UART_PrintU32(MotorDrive_GetLeftDuty());
        UART_Print(" vR=");  UART_PrintU32(MotorDrive_GetRightDuty());
        UART_Print(" L1=");  UART_Putc((char)('0' + ((bits >> 0) & 1)));
        UART_Print(" L2=");  UART_Putc((char)('0' + ((bits >> 1) & 1)));
        UART_Print(" M=");   UART_Putc((char)('0' + ((bits >> 2) & 1)));
        UART_Print(" R1=");  UART_Putc((char)('0' + ((bits >> 3) & 1)));
        UART_Print(" R2=");  UART_Putc((char)('0' + ((bits >> 4) & 1)));
        UART_Print("\r\n");

#elif LINE_FOLLOW_VERSION == 6
        /* ============ v6: 5-档 err 查表法 ============ */
        if (now - boot_ms < 1500) {
            MotorDrive_Stop();
        } else {
            line_control_v6(bits);
        }

        {
            int8_t err = (bits == 0) ? s_v6_last_err :
                         ((bits & 0b00001) ? -2 :
                          (bits & 0b10000) ? +2 :
                          (bits & 0b00010) ? -1 :
                          (bits & 0b01000) ? +1 : 0);
            UART_Print("t=");   UART_PrintU32(now);
            UART_Print(" err=");
            if (err < 0) { UART_Putc('-'); err = (int8_t)(-err); }
            UART_PrintU32((uint32_t)err);
            UART_Print(" bits="); UART_PrintU32(bits);
            UART_Print(" vL=");   UART_PrintU32(MotorDrive_GetLeftDuty());
            UART_Print(" vR=");   UART_PrintU32(MotorDrive_GetRightDuty());
            UART_Print(" S1=");   UART_Putc((char)('0' + ((bits >> 0) & 1)));
            UART_Print(" S2=");   UART_Putc((char)('0' + ((bits >> 1) & 1)));
            UART_Print(" S3=");   UART_Putc((char)('0' + ((bits >> 2) & 1)));
            UART_Print(" S4=");   UART_Putc((char)('0' + ((bits >> 3) & 1)));
            UART_Print(" S5=");   UART_Putc((char)('0' + ((bits >> 4) & 1)));
            UART_Print(" S6=");   UART_Putc((char)('0' + ((bits >> 5) & 1)));
            UART_Print("\r\n");
        }

#else
        /* ============ v7: 旧查表法 + 主状态机 (回退用) ============ */
        if (bits != 0) {
            s_line_last_seen_ms = now;
            s_last_bits = bits;
        }
        uint32_t lost_duration = now - s_line_last_seen_ms;

        if (now - boot_ms < 1500) {
            if (s_state != ST_WAIT_LINE) enter_state(ST_WAIT_LINE);
            MotorDrive_Stop();
        }
        else if (s_line_last_seen_ms == 0) {
            if (s_state != ST_WAIT_LINE) {
                enter_state(ST_WAIT_LINE);
                MotorDrive_Stop();
            }
        }
        else if (lost_duration > 3000) {
            if (s_state != ST_ERROR) {
                enter_state(ST_ERROR);
                MotorDrive_Stop();
            }
        }
        else if (bits == 0) {
            if (s_state != ST_LOST) enter_state(ST_LOST);
            uint8_t lb = s_last_bits;
            if (lb == 0) lb = 0b00100;
            uint8_t slow_l = (lb & 0b00011) ? SPEED_TURN_HARD :
                             (lb & 0b11000) ? SPEED_L_BASE    : SPEED_TURN_SOFT;
            uint8_t slow_r = (lb & 0b11000) ? SPEED_TURN_HARD :
                             (lb & 0b00011) ? SPEED_R_BASE    : SPEED_TURN_SOFT;
            MotorDrive_Set(slow_l, 1, slow_r, 1);
        }
        else {
            if (s_state != ST_TRACKING) enter_state(ST_TRACKING);
            line_control_simple(bits);
        }

        UART_Print("t=");     UART_PrintU32(now);
        UART_Print(" state=");  UART_Print(state_name(s_state));
        UART_Print(" bits=");   UART_PrintU32(bits);
        UART_Print(" vL=");     UART_PrintU32(MotorDrive_GetLeftDuty());
        UART_Print(" vR=");     UART_PrintU32(MotorDrive_GetRightDuty());
        UART_Print(" lost=");   UART_PrintU32(lost_duration);
        UART_Print(" L1=");     UART_Putc((char)('0' + ((bits >> 0) & 1)));
        UART_Print(" L2=");     UART_Putc((char)('0' + ((bits >> 1) & 1)));
        UART_Print(" M=");      UART_Putc((char)('0' + ((bits >> 2) & 1)));
        UART_Print(" R1=");     UART_Putc((char)('0' + ((bits >> 3) & 1)));
        UART_Print(" R2=");     UART_Putc((char)('0' + ((bits >> 4) & 1)));
        UART_Print("\r\n");
#endif /* LINE_FOLLOW_VERSION */
#endif /* MOTOR_TEST_DRIVE */
    }
}

/* 空实现 — 防止 Module/ 里某些被链接器拉进来的符号找不到入口 */
void Port_OnTick1kHz(void)  {}
void Port_OnTick100Hz(void) {}
