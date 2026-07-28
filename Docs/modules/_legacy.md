# 预留模块总览（OpenMV / PID 架构，当前未启用）

这些模块 main.c **当前不调用**，但编译进工程（链接器丢弃未引用的函数体，最终固件不受影响）。**保留是为 OpenMV 上线时复用**——项目原计划 OpenMV 视觉 + 四轮 PID 闭环，后转灰度两轮，这些是那套架构的产物。

## 模块清单

| 模块 | 文件 | 设计意图 | OpenMV 上线时 |
|------|------|----------|--------------|
| tracker | `Algorithm/tracker.c/h` | OpenMV 转向 PD 外环（输入像素偏差，输出 (v,ω)）| 接 OpenMV 后复活，替代/配合 line_tracker |
| pid | `Algorithm/pid.c/h` | 通用 PID 控制器（位置式/增量式）| 速度 PI 闭环、转向 PD 复用 |
| motion | `Algorithm/motion.c/h` | 差速运动学 (v,ω) → 轮速 | 配合 tracker 解算 |
| fsm | `Application/fsm.c/h` | 主状态机 IDLE/TRACKING/ELEMENT + 元素子状态 | OpenMV 元素识别时激活 |
| motor(PID版) | `Module/motor.c/h` | PID 电机闭环（编码器反馈）| 上编码器做速度闭环时激活 |
| encoder | `Module/encoder.c/h` | 编码器读速（EXTI 计数）| 上编码器时激活 |
| uart_vision | `Module/uart_vision.c/h` | OpenMV UART 帧解析（11 字节）| 接 OpenMV 时激活 |
| telemetry | `Module/telemetry.c/h` | 50Hz CSV 遥测（上位机画波形）| 调参时激活 |
| log | `Module/log.c/h` | 分级日志 DEBUG/INFO/WARN/ERROR（环形 buffer）| 需要分级日志时激活 |
| errcode | `Module/errcode.c/h` | 错误码分类计数 + 时戳 | 异常复盘时激活 |
| led_status | `Module/led_status.c/h` | LED 状态指示（IDLE/TRACKING/ERROR 闪烁）| 需要状态指示时激活 |
| port | `Port/port.h` + `BSP/port_impl.c` | 移植接口（Port_*）+ 完整实现 | 回归分层架构时激活 |

## 当前如何"活"着（不报错的原因）

- `BSP/port_min.c` 提供 `Port_*` 的最小 stub（仅 UART TX + LED + SysTick），让上述模块**编译 + 链接通过**
- main.c 末尾 `Port_OnTick1kHz / Port_OnTick100Hz` 空实现，满足被拉进来的符号引用
- 最终固件 Code ≈ 3.6 KB，这些模块的函数体被链接器 `--gc` 丢弃（未被 main 引用）

## 激活步骤（OpenMV 上线时）

1. **接 OpenMV**：UART RX 到 PB11（或改 `ti_msp_dl_config`），参考 `Docs/openmv_protocol.md`（11 字节帧）
2. **激活通信**：`Module/uart_vision.c`（帧解析）+ `Port/port.h` 的 `Port_VisionRegisterCb`
3. **激活算法**：`Algorithm/tracker.c`（转向 PD）+ `motion.c`（解算）+ `pid.c`
4. **主控集成**：main.c 加 `SENSOR_MODE` 编译开关，选灰度 / OpenMV / 双传感器融合
5. **状态机**：激活 `Application/fsm.c`（元素子状态机：十字/环岛/坡道）
6. **可选**：`telemetry/log/errcode/led_status` 按需激活
7. **Port 层**：用 `BSP/port_impl.c` 替换 `port_min.c`（或保留 port_min + 按需实现具体 Port_*）

详见 `Docs/modules/README.md` 的"OpenMV 接入路线"。

## 为什么不删

用户决策：**灰度 + OpenMV 并存**。tracker/port/uart_vision 等是 OpenMV 架构的核心，删了以后要重写。保留代价小（链接器丢弃，不进固件），收益大（OpenMV 上线时直接复用）。
