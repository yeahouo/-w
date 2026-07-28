# button USER 按键驱动 (`Module/button.c/h`)

USER 按键（PB8），时间戳去抖 + 按下事件（下降沿）。

## 硬件接口

| 信号 | 引脚 | 说明 |
|------|------|------|
| USER | PB8 | 一端接 PB8, 一端接 GND, 内部上拉 |

低电平有效（按下 = 低，上拉为高）。反了改 `button.c` 顶部 `BUTTON_ACTIVE_LEVEL`。

## API

```c
void Button_Init(void);        /* PB8 配输入+上拉, 建立去抖基线 */
bool Button_IsPressed(void);   /* 去抖后当前状态 (true=按下) */
bool Button_Consume(void);     /* 自上次 Consume 后是否有一次新按下; 读后清零 */
```

## 去抖原理（时间戳法）

原始电平持续稳定 ≥ `BUTTON_DEBOUNCE_MS` (20ms) 才采纳为新状态：
1. 原始电平翻转 → 记录变化时刻 `s_change`
2. 持续读原始 ≠ 去抖状态，且 `(now - s_change) ≥ 20ms` → 采纳
3. 若新采纳状态是"按下" → 产生一次事件（`Button_Consume` 返回 true）

比"连续 N 次采样一致"更省 CPU，去抖窗口精确。

## 依赖

- `g_system_ms`（`Button_IsPressed` 用）→ 必须 SysTick 启动后用

## 调用约定

- `Button_Init()` 在 main() SysTick 后调一次（建立基线，避免上电误触发）
- 主循环周期调 `Button_Consume()`（当前 ~10Hz / 100ms 够；人按键 100–300ms）
- ⚠️ **采样周期是灵敏度下限**：当前主循环 ~10Hz，最快 ~100ms 响应。要毫秒级（长按/连击检测）需把扫描挪进 2kHz 的 `SysTick_Handler`。

## 排错

| 现象 | 原因 |
|------|------|
| 按了没反应 | PB8 没接对 / 极性反（改 `BUTTON_ACTIVE_LEVEL`）/ SysTick 没启动 |
| 一次按计数多次 | 去抖太短（调大 `BUTTON_DEBOUNCE_MS`）|
| 上电误触发一次 | `Button_Init` 基线没建（已处理，若仍出现检查 Init 调用时机）|

## 示例

```c
Button_Init();
while (1) {
    if (Button_Consume()) {
        count++;            /* 检测到一次按下 */
        UART_Print("press\r\n");
    }
}
```
