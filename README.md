# 寻迹小车 — MSPM0G3507 主控固件

基于**立创天猛星 MSPM0G3507 开发板**（TI MSPM0G3507, Cortex-M0+ 80MHz）的**两轮差速循迹小车**。
**5 路灰度传感器**循迹（v6/v7/v8 三套算法可切换），TM1637 数码管 + SSD1306 OLED 双显示，USER 按键交互。
当前灰度循迹已调通多版，**OpenMV 视觉待接入（灰度 + OpenMV 并存路线）**。

> 📖 **代码结构 / 引脚 / 接线 / 各驱动 API 全在 [`Docs/modules/README.md`](Docs/modules/README.md)**（模块地图）—— 组内协调看那一份。

## 硬件一句话

| 项 | 配置 |
|----|------|
| 主控 | MSPM0G3507 (Cortex-M0+ 80MHz, 128KB Flash / 32KB SRAM) |
| 驱动 | 两轮差速, TB6612FNG, 软件 PWM (SysTick 2kHz × 20 档 = 100Hz) |
| 传感器 | 5 路数字灰度 (当前未装, PB17/PA12 复用给数码管) |
| 显示 | SSD1306 OLED 128×64 (4 线软件 SPI) + TM1637 四位数码管 |
| 按键 | USER = PB8 (低电平有效, 去抖) |
| 调试 | UART0 TX = PA10, 460800 8N1, 接 USB-TTL |

## 目录结构（重构后）

```
Application/   主循环 + v6/v7 查表 + 状态机 + SysTick (main.c, config.h)
Algorithm/     line_tracker(v8 灰度, 活) + tracker/pid/motion(OpenMV/PID, 预留)
Module/        驱动层: motor_drive / line_sensor / tm1637 / uart_debug / oled / button (活)
                            motor / encoder / uart_vision / telemetry / log / ... (预留)
BSP/           port_min.c (Port stub, 活) + port_impl.c (预留)
Port/          port.h 移植接口 (预留)
Project/       Keil 工程 + ti_msp_dl_config (SysConfig 生成)
Docs/          文档: modules/(模块地图+各驱动) + 调试/调参/接线指南
tools/         上位机脚本 (log_viewer / serial_viewer / openmv_main)
```

**【活】** = main.c 实际调用，进最终固件；**【预留】** = OpenMV/PID 架构残留，编译但链接丢弃，OpenMV 上线时激活。详见 [`Docs/modules/_legacy.md`](Docs/modules/_legacy.md)。

## 模块速查

| 模块 | 状态 | 用途 | 文档 |
|------|------|------|------|
| `motor_drive` | 活 ★新 | 两轮软件 PWM + 方向 | [motor_drive.md](Docs/modules/motor_drive.md) |
| `line_sensor` | 活 ★新 | 5 路灰度读取 (多数表决) | [line_sensor.md](Docs/modules/line_sensor.md) |
| `tm1637` | 活 ★新 | 四位数码管 | [tm1637.md](Docs/modules/tm1637.md) |
| `uart_debug` | 活 ★新 | UART0 调试输出 | [uart_debug.md](Docs/modules/uart_debug.md) |
| `oled` | 活 | SSD1306 OLED (软件 SPI) | [oled.md](Docs/modules/oled.md) |
| `button` | 活 | USER 按键 (PB8 去抖) | [button.md](Docs/modules/button.md) |
| `line_tracker` | 活 | v8 灰度竞赛级算法 | [line_tracker.md](Docs/modules/line_tracker.md) |
| tracker/pid/motion/fsm/... | 预留 | OpenMV/PID 架构 | [_legacy.md](Docs/modules/_legacy.md) |

★新 = 本次重构新拆出的驱动（原都内联在 main.c）。

## 快速上手

1. **装环境**：Keil MDK 5.38+ / MSPM0 SDK 2.02+ / SysConfig / MSPM0G3507 DFP（详见 `Docs/sysconfig_guide.md`）
2. **编译**：Keil 打开 `Project/tracer-car.uvprojx` → `Rebuild all` → 0 Error
3. **烧录**：UniFlash 或 Keil 下载（XDS110 / JLink）
4. **看日志**：USB-TTL 接 PA10(TX)，串口工具 460800，复位后看 BOOT 日志

> 当前默认 `LINE_FOLLOW_VERSION = 6`（v6 查表法，最稳），`TM1637_TEST = 1` + `OLED_TEST = 1`（开机自检模式，电机停）。正式循迹前改回正常模式。

## 可观测性

| 工具 | 作用 |
|------|------|
| UART 日志 | 10Hz 帧（t/bits/vL/vR/L1..R2），`tools/serial_viewer.py` 画图 |
| OLED | 实时显示运行秒数 + 按键计数 |
| TM1637 | 自检（1→9→0 循环）|
| 板载 LED | 1Hz 心跳（SysTick 活着即闪）|

## OpenMV 接入路线（后续）

灰度 + OpenMV 并存：灰度做基础循迹（稳），OpenMV 识别复杂元素（环岛/坡道/起停线）。接入点已预留：
激活 `uart_vision.c`（帧解析）+ `tracker.c`（转向 PD）+ `motion.c` + `pid.c`，main.c 加 `SENSOR_MODE` 开关。
完整步骤见 [`Docs/modules/README.md`](Docs/modules/README.md) 的"OpenMV 接入路线" + [`Docs/modules/_legacy.md`](Docs/modules/_legacy.md)。

## 开发环境

- IDE: Keil MDK-ARM (主) / TI Code Composer Studio (备)
- 配置: TI SysConfig（图形化生成 `ti_msp_dl_config.c/h`）
- SDK: MSPM0 SDK 2.02（TI 官方 DriverLib）

## 参考资源

- [立创天猛星 wiki](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/download-center.html)
- [TI MSPM0G3507 数据手册](https://www.ti.com.cn/cn/lit/gpn/mspm0g3507)
- [MSPM0G3507 + OpenMV 循迹复现工程](https://blog.csdn.net/wowsunny0417/article/details/146977934)
