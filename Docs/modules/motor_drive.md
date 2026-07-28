# motor_drive 两轮电机驱动 (`Module/motor_drive.c/h`)

两轮差速驱动：软件 PWM（SysTick bit-bang）+ 方向 GPIO，经 TB6612FNG 驱动两个直流电机。**模块拥有 duty/dir 状态**，主循环只管 `Set`，PWM 输出由 SysTick 自动完成。

## 硬件接口（TB6612FNG）

| 信号 | 引脚 | 端口 | 说明 |
|------|------|------|------|
| 左轮方向 AIN1 | PA13 | GPIOA | TB6612 A 通道 |
| 左轮方向 AIN2 | PA14 | GPIOA | TB6612 A 通道 |
| 左轮 PWM  PWMA | PB2  | GPIOB | 软件 PWM |
| 右轮方向 BIN1 | PA17 | GPIOA | TB6612 B 通道 |
| 右轮方向 BIN2 | PA16 | GPIOA | TB6612 B 通道 |
| 右轮 PWM  PWMB | PB3  | GPIOB | 软件 PWM |

TB6612 板上：**CS 接 GND**（永久片选）、**STBY 接 VCC**（解除待机）、VM 接电机电源（电池）、VCC 接 3.3V。

## API

```c
void MotorDrive_Init(void);                                                /* 配 6 引脚 */
void MotorDrive_Set(uint8_t ld, uint8_t ldir, uint8_t rd, uint8_t rdir);   /* 设左右轮, 立即应用方向 */
void MotorDrive_Tick(void);                                                /* 软件 PWM 一拍 — SysTick 调 */
void MotorDrive_Stop(void);                                                /* 两轮全停 */
uint8_t MotorDrive_GetLeftDuty(void);                                      /* 日志查询 */
uint8_t MotorDrive_GetRightDuty(void);
```

- **duty**: 0~20（PWM 档，`PWM_PHASE_MAX=20`）
- **dir**: 1=正转(物理前进), 0=反转

## PWM 原理（软件 bit-bang）

SysTick 2kHz，每 tick 调 `MotorDrive_Tick()`：
- `pwm_phase` 计数器 0~19 循环
- `phase < duty` 时对应 PWMA/B 拉高，否则拉低
- → PWM 频率 = 2kHz / 20 = **100Hz**，占空比 = duty / 20

⚠️ 100Hz 偏低（电机有可闻嗡声、低速扭矩一般）。调通后若要提质，迁到硬件 TimerA PWM（`ti_msp_dl_config` 加 PWM 外设，`Tick` 改成配占空比寄存器）。

## 极性

左轮接线反向，`LEFT_MOTOR_POLARITY = -1`（`apply_dir` 内异或翻转方向位）；右轮 `RIGHT_MOTOR_POLARITY = +1`。
两轮 `dir=1` 统一为物理前进。**实测仍不对**则改 `motor_drive.c` 顶部这两个宏（+1/-1 互换）。

## 依赖

- **SysTick**：`MotorDrive_Tick` 在 `SysTick_Handler`（2kHz 中断）里调，必须快速（仅 GPIO 翻转，无阻塞）
- duty/dir 为 `volatile`（中断读、主循环写）

## 调用约定

```
MotorDrive_Init();              // main() 初始化段调一次
// 主循环里:
MotorDrive_Set(8, 1, 8, 1);     // 两轮 40% 前进
MotorDrive_Set(4, 1, 8, 1);     // 左慢右快 = 右转追线
MotorDrive_Stop();              // 停车
// PWM 持续输出由 SysTick 自动完成, 主循环不用管
```

## 排错

| 现象 | 原因 |
|------|------|
| 完全不转 | duty=0 / 极性错 / TB6612 STBY 没拉高 / 电机电源不够 |
| 只转一个方向 | AIN/BIN 某只脚没接 / 对应方向 GPIO 坏 |
| 方向反了 | 极性宏反了 → 改 `LEFT/RIGHT_MOTOR_POLARITY` |
| 两轮转速不同 | 电机制造公差 → main.c 调 `SPEED_L_BASE / SPEED_R_BASE` 补偿 |
| `MotorDrive_Tick` 没被调 | SysTick_Handler 里漏调 → 检查 main.c |
