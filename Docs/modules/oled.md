# OLED 显示驱动 (`Module/oled.c/h`)

SSD1306 128×64 OLED，4 线软件 SPI（CS 接 GND）。显示开机画面 / 调试信息 / 按键计数。

## 硬件接口

| 信号 | 引脚 | 方向 | 说明 |
|------|------|------|------|
| SCL (D0) | PA28 | 主控→OLED | 时钟 |
| SDA (D1) | PA31 | 主控→OLED | 数据, MSB first |
| RES      | PB14 | 主控→OLED | 复位, 低有效 |
| DC       | PB15 | 主控→OLED | 0=命令, 1=数据 |
| CS       | GND  | — | 永久片选 |
| VCC      | 3.3V | — | |
| GND      | GND  | — | |

⚠️ **引脚配置（`DL_GPIO_initDigitalOutput`）在 main.c 里，不在 `OLED_Init`**。调用者必须先配好 4 个引脚、再调 `OLED_Init`。

时序：软件 bit-bang（CLK 空闲低，上升沿采样），与 TM1637 同风格，不依赖 SysConfig 的 SPI 外设。

## 显存模型

SSD1306 页寻址：8 页 × 128 列 = 1024 字节。驱动在 RAM 维护副本 `s_gram[8][128]`，改完调 `OLED_Refresh()` 推到屏。

## API

```c
void OLED_Init(void);                                   /* RES复位+初始化命令+清屏 */
void OLED_Clear(void);                                  /* 清显存并刷新 */
void OLED_Refresh(void);                                /* 把 gram 整片推到屏 (~8ms) */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);   /* x:0-127, y:0-63 */
void OLED_DrawChar(uint8_t x, uint8_t y, char ch);      /* 6x8, x=字符列0-21, y=页0-7 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *s);
```

字库：6×8 ASCII（0x20–0x7E，95 字符，公开标准字模）。

## 依赖

- `g_system_ms`（`OLED_Init` 用它做 RES 复位延时）→ **必须 SysTick 启动后调**
- `ti_msp_dl_config.h`（`DL_GPIO_*`, `IOMUX_PINCM*`）

## 调用约定

```
1. main.c 配 4 个 OLED 引脚 (DL_GPIO_initDigitalOutput + enableOutput + setPins)
2. SysTick_Config(...)              ← g_system_ms 开始走
3. OLED_Init()                      ← 内部用 g_system_ms 延时, 必须在步骤2后
4. OLED_DrawString(...) 改 gram
5. OLED_Refresh()                   ← 推到屏, 改完必须 Refresh 才显示
```

满屏刷新 ~8ms，10Hz 无压力。

## SH1106 兜底

默认 SSD1306（0.96"）。若是 1.3" **SH1106**：`oled.c` 顶部 `OLED_COL_OFFSET` 改 `2`（列地址 +2 偏移，否则画面左侧 2 列空白）。

## 排错

| 现象 | 原因 |
|------|------|
| 全黑 | VCC 没接 / RES 时序 / 引脚没配 / Init 没跑（看串口有无 `OLED init ok`）|
| 全亮乱码 | DC 或 SCL/SDA 接反 / PINCM 配错 |
| 左侧 2 列空白 | 是 SH1106 → `OLED_COL_OFFSET = 2` |

## 示例

```c
/* main.c: 先配 PA28/PA31/PB14/PB15 引脚 */
SysTick_Config(SYSTICK_LOAD);
OLED_Init();
OLED_DrawString(0, 0, "tracer-car");
OLED_DrawString(0, 1, "OLED OK");
OLED_Refresh();
```
