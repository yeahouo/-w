# line_sensor 5 路灰度读取 (`Module/line_sensor.c/h`)

读 5 路数字灰度传感器，多次采样 + 多数表决，返回 5-bit bitmask（哪几路检测到黑线）。供 v6/v7/v8 循迹算法用。

## 硬件接口

| 传感器 | 引脚 | bitmask 位 | 端口 |
|--------|------|-----------|------|
| L1 (最左) | PB17 | bit0 | GPIOB |
| L2        | PA12 | bit1 | GPIOA |
| M  (中)   | PA22 | bit2 | GPIOA |
| R1        | PA27 | bit3 | GPIOA |
| R2 (最右) | PA9  | bit4 | GPIOA |

**极性**：`LINE_ACTIVE_LEVEL = 0` → 黑线时 GPIO 读到 0（常见接法）。反了改 `line_sensor.c` 顶部宏。

⚠️ **PB17/PA12 与 TM1637 复用**。当前灰度**未装**（引脚让给数码管），`LineSensor_Init()` 在 main.c 里被 `#if 0` 包住。
**装灰度时**：① 关数码管（`TM1637_TEST=0` + 不调 `TM1637_Init`）② 取消 main.c 里 `LineSensor_Init()` 的 `#if 0`。

## API

```c
void   LineSensor_Init(void);    /* 配 5 路为数字输入 + 内部上拉 (灰度装上后调) */
uint8_t LineSensor_Read(void);   /* 返回 bitmask */
```

**bitmask 约定**：`bit0=L1(最左) bit1=L2 bit2=M bit3=R1 bit4=R2(最右)`，**1 = 检测到黑线**。

| bits | 含义 |
|------|------|
| `0b00100` | M 单亮 → 直行 |
| `0b00010` | L2 亮 → 线偏左，小左转 |
| `0b00001` | L1 亮 → 线大偏左，大左转 |
| `0b01000` | R1 亮 → 线偏右，小右转 |
| `0b10000` | R2 亮 → 线大偏右，大右转 |
| `0b00000` | 丢线 |

## 多数表决（v6.2，抑抖动）

每次 `Read` 连读 `V6_SENSOR_SAMPLES` 次（间隔 NOP），每路累计命中数 ≥ `V6_SENSOR_THRESHOLD` 才算真命中。补偿硬件灵敏度不足（检测延迟 / 压线不触发 / 边缘模糊）。

调参（`line_sensor.c` 顶部）：
- `V6_SENSOR_SAMPLES` (5)：采样次数，越多越稳但越慢，奇数便于表决
- `V6_SENSOR_THRESHOLD` (3)：命中阈值，≥ 半数
- `V6_SENSOR_DELAY_NOP` (20)：采样间隔 NOP 数，硬件响应慢调大

## 依赖

- 无（纯 GPIO 读）

## 调用约定

- `LineSensor_Init()` 灰度装上后调一次
- `LineSensor_Read()` 主循环周期调（当前 ~10Hz）

## 排错

| 现象 | 原因 |
|------|------|
| 恒全 0 | 灰度没装 / 引脚没 `Init` / 极性反（改 `LINE_ACTIVE_LEVEL`）|
| 恒全 1 | 全压黑线 / 传感器常亮 / 极性反 |
| 抖动误判 | 调大 `V6_SENSOR_SAMPLES` 或 `V6_SENSOR_DELAY_NOP` |
| 某一路不响应 | 该路接线/传感器坏，看日志对应 L1..R2 字段 |

## 示例

```c
LineSensor_Init();
uint8_t bits = LineSensor_Read();
if (bits == 0b00100) {
    /* M 单亮 → 直行 */
}
```
