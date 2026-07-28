# tracer-car 模块地图

> 组内协调入口：看这一份就能知道"哪个模块干什么、接哪个引脚、依赖谁、谁在用谁"。
> 各驱动细节见同目录的 `*.md`。

## 项目一句话

TI **MSPM0G3507**（Cortex-M0+ 80MHz）主控的**两轮差速循迹小车**：5 路灰度传感器循迹，TM1637 数码管 + SSD1306 OLED 双显示，USER 按键交互。当前灰度循迹已通，**OpenMV 视觉待接入**（灰度 + OpenMV 并存路线）。

## 目录结构（重构后）

```
tracer-car/
├── Application/   应用层
│   ├── main.c          主循环 + v6/v7 查表 + v7 状态机 + SysTick 调度  【活】
│   ├── config.h        全局配置 (灰度参数 + OpenMV/PID 预留参数)        【活】
│   └── fsm.c/h         主状态机 (IDLE/TRACKING/..., OpenMV 架构预留)    【预留】
├── Algorithm/     算法层
│   ├── line_tracker.c/h  v8 灰度竞赛级算法 (加权位置法+增益调度PD)      【活】
│   ├── tracker.c/h       OpenMV 转向 PD 外环                            【预留】
│   ├── pid.c/h           通用 PID 控制器                                【预留】
│   └── motion.c/h        差速运动学解算 (v,ω)→轮速                      【预留】
├── Module/        模块驱动层
│   ├── motor_drive.c/h   两轮电机: 软件 PWM + 方向 GPIO                 【活】★新
│   ├── line_sensor.c/h   5 路灰度读取: 多次采样+多数表决                【活】★新
│   ├── tm1637.c/h        TM1637 四位数码管 (软件时序)                   【活】★新
│   ├── uart_debug.c/h    UART0 调试输出 (putc/print/print_u32)          【活】★新
│   ├── oled.c/h          SSD1306 OLED (4 线软件 SPI)                    【活】
│   ├── button.c/h        USER 按键 (PB8, 去抖+事件)                     【活】
│   ├── motor.c/h         PID 电机闭环 (编码器架构)                      【预留】
│   ├── encoder.c/h       编码器读速                                     【预留】
│   ├── uart_vision.c/h   OpenMV UART 帧解析                             【预留】
│   ├── telemetry.c/h     50Hz CSV 遥测                                  【预留】
│   ├── log.c/h           分级日志 (环形 buffer)                         【预留】
│   ├── errcode.c/h       错误码系统                                     【预留】
│   └── led_status.c/h    LED 状态指示                                   【预留】
├── Port/port.h    移植接口 (Port_*, OpenMV/PID 架构预留)                 【预留】
├── BSP/
│   ├── port_min.c        Port 最小 stub (仅 UART TX+LED+SysTick, 链接用) 【活】
│   └── port_impl.c       Port 完整实现 (电机/编码器/视觉, 预留)          【预留】
├── Project/       Keil 工程 + SysConfig 生成 (ti_msp_dl_config.c/h)
├── Docs/          文档 (本目录 modules/ + 调试/调参/接线指南)
└── tools/         上位机脚本 (log_viewer/serial_viewer/openmv_main)
```

- **【活】** = main.c 实际调用, 进最终固件
- **【预留】** = 当前未启用 (OpenMV/PID/编码器架构残留), 编译进工程但链接丢弃; OpenMV 上线时激活。详见 `_legacy.md`
- **★新** = 本次重构新拆出的驱动

## 模块依赖图

```
                        ┌─────────────┐
                        │   main.c    │  主循环 / v6·v7 查表 / v7 状态机 / SysTick
                        └──────┬──────┘
          ┌──────────┬─────────┼──────────┬──────────┬──────────┐
          ▼          ▼         ▼          ▼          ▼          ▼
    motor_drive  line_sensor  oled    tm1637     button    line_tracker
     (电机PWM)    (灰度读)   (显示)   (数码管)   (USER键)   (v8算法)
          │          │         │         │          │          │
          └──────────┴─────────┴─────────┴──────────┴──────────┘
                              ▼
                         g_system_ms   (main.c 定义, SysTick 累加)
                              ▲
                  uart_debug (UART0 TX)
                              ▲
                  ti_msp_dl_config (SysConfig 生成: 时钟/UART/GPIO)
```

- 所有驱动都依赖 `g_system_ms`（main.c 定义，SysTick 每 0.5ms 累加）做时序/超时
- `uart_debug` 依赖 `ti_msp_dl_config.h` 里的 `UART_0_INST`
- `line_tracker` 依赖 `config.h` 的 LINE_* / D1~D4_KP/KD 等灰度参数
- main.c 不直接碰任何寄存器（除 OLED 引脚配置 + LED 翻转，历史遗留）

## 引脚分配总表

| 引脚 | 功能 | 模块 | 备注 |
|------|------|------|------|
| PA5 / PA6 | HFXT 晶振 | ti_msp_dl_config | 系统主时钟源 |
| PA10 / PA11 | UART0 TX / RX | uart_debug | 460800 8N1, 接 USB-TTL |
| PA13 / PA14 | AIN1 / AIN2 (左轮方向) | motor_drive | TB6612 A |
| PA16 / PA17 | BIN2 / BIN1 (右轮方向) | motor_drive | TB6612 B |
| PB2 / PB3 | PWMA / PWMB (电机 PWM) | motor_drive | 软件 PWM 100Hz |
| PB8 | USER 按键 | button | 低电平有效, 内部上拉 |
| PB14 / PB15 | OLED RES / DC | oled | 软件 SPI |
| PA19 / PA20 | SWDIO / SWCLK | — | 调试, **禁止占用** |
| PA22 | 灰度 M (中) | line_sensor | 当前 #if 0 未装 |
| PA27 | 灰度 R1 | line_sensor | 当前 #if 0 未装 |
| PA28 / PA31 | OLED SCL / SDA | oled | 软件 SPI |
| PB17 | TM1637 CLK / 灰度 L1 | tm1637 / line_sensor | **复用** (灰度未装时给数码管) |
| PA12 | TM1637 DIO / 灰度 L2 | tm1637 / line_sensor | **复用** |
| PA9 | 灰度 R2 | line_sensor | 当前 #if 0 未装 |
| PB22 | 板载 LED | main.c (SysTick) | 1Hz 心跳 |

> ⚠️ PB17/PA12 是**复用脚**：灰度传感器未装时给 TM1637 数码管；装灰度时要先关数码管（`TM1637_TEST=0` + 不调 `TM1637_Init`）。

## 初始化顺序（main.c，不可乱序）

```
1. SYSCFG_DL_init()       时钟 80MHz + UART0 + GPIO (ti_msp_dl_config)
2. MotorDrive_Init()      电机 6 个引脚
3. (LineSensor_Init())    #if 0 — 灰度未装, 装上后启用
4. TM1637_Init()          数码管 CLK/DIO
5. OLED 引脚配置          4 个 GPIO (PA28/PA31/PB14/PB15)
6. SysTick_Config()       启动 2kHz 中断 → g_system_ms 开始走
7. OLED_Init()            需要 g_system_ms (复位延时)
8. Button_Init()          USER 按键基线
9. LineTracker_Init()     仅 LINE_FOLLOW_VERSION==8 时
```

**关键**：所有用 `g_system_ms` 做延时的模块（OLED/Button）必须在步骤 6 之后初始化。

## 调度时序（SysTick 2kHz）

SysTick_Handler 每 0.5ms 跑一次，内部做 4 件事：
1. 2 分频 → `g_system_ms` 每 1ms +1
2. `MotorDrive_Tick()` → 软件 PWM（每 tick 翻转 PWMA/PWMB，20 档相位 = 100Hz PWM）
3. 200 分频 → `s_ctrl_flag` 每 100ms 置位（主循环节奏 ≈ 10Hz）
4. 1000 分频 → LED 翻转（1Hz 心跳）

主循环 `__WFI` 睡眠等 `s_ctrl_flag`，10Hz 醒来跑：按键检测 → OLED 刷新 → 循迹算法（v6/v7/v8）→ 串口日志。

## OpenMV 接入路线（后续工作）

当前灰度循迹已通，OpenMV 上线时的接入点：

1. **通信**：激活 `Module/uart_vision.c`（OpenMV 帧解析）+ `Port/port.h` 的 `Port_VisionRegisterCb`；OpenMV 端脚本见 `tools/openmv_main.py`
2. **算法**：激活 `Algorithm/tracker.c`（OpenMV 转向 PD 外环，输入像素偏差，输出 (v,ω)）；配合 `Algorithm/motion.c`（差速解算）+ `Algorithm/pid.c`
3. **主控集成**：在 `main.c` 加一个 `SENSOR_MODE` 开关（或扩展 `LINE_FOLLOW_VERSION`），编译期选灰度 / OpenMV / 双传感器融合
4. **状态机**：激活 `Application/fsm.c`（IDLE/TRACKING/ELEMENT 元素子状态机）管理元素识别（十字/环岛/坡道）
5. **协议**：参考 `Docs/openmv_protocol.md`（11 字节帧定义）

> 灰度 + OpenMV 并存策略：灰度做基础循迹（稳），OpenMV 识别复杂元素（环岛/坡道/起停线）。两套驱动都保留正是为此。

## 相关文档

- 各驱动详细 API：`oled.md` `button.md` `tm1637.md` `motor_drive.md` `line_sensor.md` `uart_debug.md`
- v8 算法：`line_tracker.md`
- 预留模块（OpenMV/PID 架构）：`_legacy.md`
- 接线/调试/调参：`Docs/wiring_guide.md` `Docs/debug_guide.md` `Docs/tuning_log.md`
