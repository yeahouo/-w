# 寻迹小车 — MSPM0G3507 主控固件

基于 **立创天猛星 MSPM0G3507 开发板**(TI MSPM0G3507,Cortex-M0+ 80MHz + FPU)的寻迹小车核心软件。
视觉前端为 **OpenMV**(自带 MCU 跑视觉算法),通过串口将赛道偏差与元素标志发给主控。
底盘为 **四轮独立驱动**,采用差速模型建模,串级 PID 控制(转向 PD 外环 + 速度 PI 内环)。

## 目录结构

```
tracer-car/
├── Application/   应用层:主循环、状态机、全局配置
├── Algorithm/     算法层:PID、运动学、循迹决策(与硬件无关,可 PC 单测)
├── Module/        模块驱动层:电机、编码器、串口、日志、遥测、错误码、LED
├── BSP/           硬件抽象层:
│                   ├─ ti_msp_dl_config.c/h  ← SysConfig 生成,不改
│                   └─ port_impl.c           ← Port 接口的 DriverLib 实现(模板)
├── Port/          移植接口:port.h 抽象边界,便于 PC 单测
├── Tests/         PC 端单元测试(待补)
├── Docs/          openmv_protocol.md / tuning_log.md / debug_guide.md
└── Project/       Keil/CCS 工程文件
```

## 可观测性体系(出问题时怎么排查)

详细见 [Docs/debug_guide.md](Docs/debug_guide.md)。简版:

| 工具 | 作用 | 文件 |
|------|------|------|
| 分级日志 | DEBUG/INFO/WARN/ERROR + 时间戳 + 模块标签,环形 buffer 异步输出 | `Module/log.c` |
| 实时遥测 | 50Hz CSV 帧发上位机画波形,PID 调参必备 | `Module/telemetry.c` |
| 错误码系统 | 分类计数 + 时戳,复盘神器 | `Module/errcode.c` |
| LED 状态指示 | 没屏幕也能看车状态(IDLE 慢闪/TRACKING 常亮/ERROR 急闪) | `Module/led_status.c` |

## 上手顺序

1. 先看 [Docs/debug_guide.md](Docs/debug_guide.md) 的 SysConfig 配置清单
2. 在 SysConfig 配好外设,生成 `ti_msp_dl_config.c/h`
3. 在 `BSP/port_impl.c` 顶部把宏名替换为实际生成的名字
4. 烧录,接调试串口(460800+ 波特率)
5. 按 [debug_guide.md](Docs/debug_guide.md) 的"调试 checklist"逐步验证

## 分层依赖

L4(应用)→ L3(算法)→ L2(模块驱动)→ L1(BSP/HAL)
**禁止反向依赖**,算法层不直接碰寄存器。

## 开发环境

- IDE: Keil MDK-ARM 或 TI Code Composer Studio
- 配置: TI SysConfig(图形化生成引脚与外设)
- SDK: MSPM0 SDK(TI 官方 DriverLib)

## 参考资源

- [立创天猛星 wiki](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/download-center.html)
- [TI MSPM0G3507 数据手册](https://www.ti.com.cn/cn/lit/gpn/mspm0g3507)
- [MSPM0G3507 + OpenMV 循迹复现工程](https://blog.csdn.net/wowsunny0417/article/details/146977934)

## 当前阶段

- [x] P0 基建(目录骨架 + 配置 + 接口文档)
- [ ] P1 电机层(PWM + 编码器 + PI 闭环)
- [ ] P2 运动学(差速解算)
- [ ] P3 通信(OpenMV 协议解析)
- [ ] P4 循迹(转向 PD 外环)
- [ ] P5 元素(状态机 + 元素子状态)
- [ ] P6 强化(异常恢复 + 调参文档化)
