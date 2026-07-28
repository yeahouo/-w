# line_tracker v8 灰度竞赛级算法 (`Algorithm/line_tracker.c/h`)

当前最强的循迹算法：加权位置法 + 5 档增益调度 PD + 6 状态元素识别状态机。

> ⚠️ main.c 当前 `LINE_FOLLOW_VERSION = 6`（v6 查表），**v8 未默认启用**。切 v8：main.c 顶部改 `#define LINE_FOLLOW_VERSION 8`。

## 5 层管线

1. **输入**：5-bit 命中（来自 `LineSensor_Read`）
2. **加权位置法连续化**：5-bit → `position ∈ [-2, +2]` + 3 帧历史投票（防单帧误判）
3. **死区 + 滤波**：`|pos| < 0.2 → 0`；一阶低通滤波 `α=0.4`
4. **5 档增益调度 PD**（D1 直道 / D2 小弯 / D3 大弯 / D4 急弯 / D5 丢线）+ 0.1 迟滞带防乒乓
5. **6 状态元素识别状态机** + 丢线三段式
6. **输出**：差速 → `(duty_l, duty_r)`

## 状态机（LT_State_t）

| 状态 | 含义 |
|------|------|
| WAIT_LINE | 启动等线（还没见过线）|
| STRAIGHT | 直道（position 稳定 ±0.2 持续）|
| CURVE    | 弯道（PD 正常输出）|
| CROSS    | 十字（最左+最右同命中 ≥2 帧）|
| SHARP    | 急弯（最外传感器持续命中 ≥3 帧）|
| LOST     | 丢线（三段式处理）|
| ERROR    | 持续丢线 >3s 全停 |

**丢线三段式**：方向记忆（0–500ms 惯性续行）→ 扫掠搜索（500ms–2s）→ 全停（>3s）。

## 增益调度档（参数在 `config.h`）

| 档 | 进入条件 \|pos\| | Kp | Kd | 速度档 |
|----|-----------------|-----|-----|-------|
| D1 直道 | < 0.3   | 8  | 80  | 2/2 |
| D2 小弯 | 0.3–1.0 | 15 | 200 | 2/2 |
| D3 大弯 | 1.0–1.8 | 25 | 350 | 2/2 |
| D4 急弯 | > 1.8   | 40 | 500 | 2/1 (外/内) |
| D5 丢线 | 全白/全黑 | — | — | 1 + 方向 |

## API

```c
void LineTracker_Init(void);
void LineTracker_Update(uint8_t bits, uint32_t now_ms);   /* 100Hz 主循环调 */
uint8_t  LineTracker_GetLeftDuty/RightDuty(void);
uint8_t  LineTracker_GetLeftDir/RightDir(void);
LT_State_t LineTracker_GetState(void);
LT_Gear_t  LineTracker_GetGear(void);
float      LineTracker_GetPosition(void);    /* 滤波后 [-2,+2] */
int8_t     LineTracker_GetLastError(void);
```

## 依赖（`config.h` 灰度段参数）

`LINE_W_L1..R2`（权重）/ `LINE_VOTE_FRAMES` / `LINE_DEAD_ZONE` / `LINE_POS_FILTER_ALPHA` / `LINE_BAND_D1_D2..D3_D4` / `LINE_HYSTERESIS` / `D1..D4_KP/KD` / `SPEED_MIN_*` / `LINE_LOST_*_MS` / `CROSS/SHARP_CONFIRM_FRAMES` / `STRAIGHT_CONFIRM_MS` / `LINE_OUT_MAX_DUTY` / `LINE_DELTA_RATE_LIMIT`

## 调用约定

```c
LineTracker_Init();
while (1) {
    uint8_t bits = LineSensor_Read();
    LineTracker_Update(bits, now);
    MotorDrive_Set(LineTracker_GetLeftDuty(),  LineTracker_GetLeftDir(),
                   LineTracker_GetRightDuty(), LineTracker_GetRightDir());
}
```
- 100Hz 调用（当前主循环 ~10Hz，v8 在更低频率下仍可工作但响应慢）
- boot 后 1.5s 静止由 main.c 控制（与算法解耦）

## 排错

| 现象 | 原因 |
|------|------|
| 直线摆动 | D1 档不该加 D（直线段应纯 P，确认 `config.h` `D1_KD = 0`）|
| 弯道冲出 | D3/D4 的 Kp 不够，或整体速度太快 |
| 误判十字/急弯 | 调 `CROSS_CONFIRM_FRAMES` / `SHARP_CONFIRM_FRAMES` 加大确认帧数 |
| 丢线不恢复 | 看 `LINE_LOST_INERTIA_MS / SEARCH_MS / FATAL_MS` 三段超时 |
| 日志 pos 不变 | 滤波系数 `LINE_POS_FILTER_ALPHA` 太小（太相信旧值）|

## 启用 v8

main.c 顶部：
```c
#define LINE_FOLLOW_VERSION  8
```
