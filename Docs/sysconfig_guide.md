# SysConfig GUI 配置指南(MSPM0G3507 + MG310P20 + OpenMV + 单 UART)

> 主人按这份文档,在 SysConfig GUI 里**逐步点击**配置。SysConfig 会自动检查引脚冲突。
> 配置完成后,保存即可生成 `ti_msp_dl_config.c/h`。

## 硬件方案总览(先看这个!)

| 模块 | 配置 | 备注 |
|------|------|------|
| UART0 | TX=PA10 / RX=PA11,**460800**(锁定),8N1 | **共享**:RX 收 OpenMV,TX 发调试日志/遥测 |
| TIMG0 | 周期 1ms(1kHz),中断 | 电机 PI 闭环 + 系统时钟 |
| TIMG1 | 周期 10ms(100Hz),中断 | 编码器采样 + 遥测 + LED |
| TIMA0 | PWM 4 通道,20kHz,周期值 1000 | 4 路 MG310P20 电机速度 |
| GPIO_PWM_DIR × 4 | 输出,初始低 | TB6612 方向脚(每电机 1 个) |
| GPIO_ENC_A × 4 | 输入 + 下降沿 EXTI | 编码器 A 相(中断计数) |
| GPIO_ENC_B × 4 | 输入 | 编码器 B 相(只在 A 中断里读方向) |
| LED | PB22 输出 | 板载 LED |
| USER 按键 | 输入 + 上拉 + 下降沿 EXTI | 启动按键(查原理图确认引脚) |

> ⚠️ **关于引脚分配**:Nano 不能看引脚图,主人需要对照立创开发板原理图,确认推荐引脚在板子上**未被板载外设占用**且**已引出到排针**。SysConfig 也会自动报警告。

---

## Step 0:打开 SysConfig

### 在 Keil 里
1. 打开工程(.uvprojx)
2. 双击工程里的 `empty.syscfg`(或自己加一个)
3. SysConfig GUI 自动启动

### 在 CCS 里
1. 新建工程,选 MSPM0G3507
2. 双击 `empty.syscfg`

### 顶部参数(自动填好)
```
Device:    MSPM0G3507
Package:   LQFP-64 (PM)
Product:   mspm0_sdk@2.02.00.05 或更新
```

---

## Step 1:添加并配置 UART0(共享 OpenMV + 调试)

### 操作
1. 左侧 `MSPM0 DRIVERS` → `UART` → 点 `+ ADD`
2. 实例名改为 `UART_0`

### 参数设置
| 参数 | 值 | 说明 |
|------|-----|------|
| `$name` | `UART_0` | 代码里宏名 |
| `Basic` → `Mode` | `MODE_UART`(默认) | 标准异步 UART |
| `Basic` → `Baudrate` | `460800` 或 `921600` | 越高越能扛日志流量 |
| `Basic` → `Data Bits` | `8` | |
| `Basic` → `Parity` | `NONE` | |
| `Basic` → `Stop Bits` | `ONE` | |
| `UART Clk Src` | `BUSCLK` 或 `MFCLK` | 用 BUSCLK 更稳 |
| `Interrupts` → 勾选 `RX` | ✓ | OpenMV 帧到达触发中断 |

### 引脚分配(PinMux)
- `TX Pin` → `PA10`
- `RX Pin` → `PA11`

> 这两个引脚在立创例程 `05_uart` 里默认就是 PA10/PA11,直接用。

---

## Step 2:添加并配置 TIMG0(1kHz 系统节拍 + 电机 PI)

### 操作
1. `MSPM0 DRIVERS` → `TIMER` → `+ ADD`
2. 实例名改为 `TIMER_0`

### 参数
| 参数 | 值 |
|------|-----|
| `$name` | `TIMER_0` |
| `Timer Mode` | `PERIODIC`(周期) |
| `Timer Period` | `1 ms` |
| `Timer Clk Divider` | `1`(默认) |
| `Start Timer` | ✓ 勾选 |
| `Interrupts` → 勾选 `ZERO` | ✓ |

### 引脚
- 不需要引脚(纯内部定时)
- `Peripheral` → 自动选 `TIMG0`

---

## Step 3:添加并配置 TIMG1(100Hz 编码器采样 + LED + 遥测)

### 操作
1. `TIMER` → `+ ADD`
2. 实例名改为 `TIMER_1`

### 参数
| 参数 | 值 |
|------|-----|
| `$name` | `TIMER_1` |
| `Timer Mode` | `PERIODIC` |
| `Timer Period` | `10 ms` |
| `Start Timer` | ✓ |
| `Interrupts` → `ZERO` | ✓ |

### 引脚
- 不需要
- `Peripheral` → `TIMG1`

---

## Step 4:添加并配置 TIMA0(4 通道 PWM,电机速度控制)

### 操作
1. `MSPM0 DRIVERS` → `PWM` → `+ ADD`
2. 实例名改为 `PWM_MOT`

### 参数
| 参数 | 值 |
|------|-----|
| `$name` | `PWM_MOT` |
| `Peripheral` | `TIMA0`(高级 Timer,4 CC) |
| `Timer Clock Divider` | `1` |
| `PWM Frequency` | `20000 Hz`(20kHz,人耳听不到) |
| `PWM Period (counts)` | `1000`(对应 MOTOR_PWM_PERIOD) |
| `Start Timer` | ✓ |
| `CC Channels` | 添加 4 个 CCP:`CCP1`, `CCP2`, `CCP3`, `CCP4` |
| 每个 CC 的 duty | 初始 `0` |

### 引脚分配(主人查原理图,选 4 个排针引出且未被占用的)
推荐(占位,需主人确认):
- `CCP1 Pin` → `PA12`(电机 LF 的 PWM)
- `CCP2 Pin` → `PA13`(电机 LR 的 PWM)
- `CCP3 Pin` → `PA14`(电机 RF 的 PWM)
- `CCP4 Pin` → `PA15`(电机 RR 的 PWM)

> 如果引脚冲突,SysConfig 会标红;点击红区会提示可用引脚,选一个即可。

---

## Step 5:添加 4 个 GPIO 输出(TB6612 方向脚)

### 操作(重复 4 次)
1. `MSPM0 DRIVERS` → `GPIO` → `+ ADD`
2. 实例名分别为 `GPIO_DIR_LF` / `GPIO_DIR_LR` / `GPIO_DIR_RF` / `GPIO_DIR_RR`

### 每个实例的参数
| 参数 | 值 |
|------|-----|
| `Port` | `PORTA`(或 PORTB,看引脚) |
| `Pin` | 自选(查原理图) |
| `Direction` | `OUTPUT` |
| `Initial Value` | `LOW` |

> 推荐占位:`PA16` / `PA17` / `PA18` / `PA19`(主人按板子改)

> 💡 **TB6612 方向逻辑**:`AIN1=0,AIN2=1` 正转;`AIN1=1,AIN2=0` 反转;两者都 0 是空闲;两者都 1 是刹车。
> 简化做法:**AIN2 固定接 GND**,只用 `AIN1` 作方向:DIR=0 正转,DIR=1 反转。这样每电机只占 1 个 GPIO。

---

## Step 6:添加 4 个 GPIO + EXTI(编码器 A 相)

### 操作(重复 4 次)
1. `GPIO` → `+ ADD`
2. 实例名:`GPIO_ENC_A_LF` / `GPIO_ENC_A_LR` / `GPIO_ENC_A_RF` / `GPIO_ENC_A_RR`

### 每个实例的参数
| 参数 | 值 |
|------|-----|
| `Direction` | `INPUT` |
| `Internal Resistor` | `PULL_UP`(或 NONE,看编码器输出类型) |
| `Interrupt` | `RISING_EDGE` 或 `FALLING_EDGE`(看编码器,通常用双边沿 `BOTH_EDGES` 精度更高) |

> 推荐占位:`PB0` / `PB1` / `PB2` / `PB3`

> ⚠️ MSPM0 的 GPIO 中断是分组(`INT_0`~`INT_7`),同组的中断共享一个 ISR。主人需要把 4 个 A 相分配到不同组,或者在 ISR 里读 4 个 GPIO 引脚电平判断。SysConfig 会自动分组。

---

## Step 7:添加 4 个普通 GPIO 输入(编码器 B 相,只读不算)

### 操作(重复 4 次)
1. `GPIO` → `+ ADD`
2. 实例名:`GPIO_ENC_B_LF` 等

### 参数
| 参数 | 值 |
|------|-----|
| `Direction` | `INPUT` |
| `Internal Resistor` | `NONE` 或 `PULL_UP` |

> 推荐占位:`PB4` / `PB5` / `PB6` / `PB7`

> B 相只在 A 相中断里读一次,用来判断方向,不需要中断。

---

## Step 8:LED(板载 PB22,通常默认有)

### 操作
1. 如果 LED 实例已存在(从模板创建时),跳过
2. 否则 `GPIO` → `+ ADD`,实例名 `LED1`
   - `Port` = `PORTB`,`Pin` = `22`
   - `Direction` = `OUTPUT`
   - `Initial Value` = `LOW`

---

## Step 9:USER 按键(启动按键)

### 操作
1. `GPIO` → `+ ADD`
2. 实例名 `KEY_USER`

### 参数
| 参数 | 值 |
|------|-----|
| `Direction` | `INPUT` |
| `Internal Resistor` | `PULL_UP`(默认上拉) |
| `Interrupt` | `FALLING_EDGE`(按下接 GND,下降沿触发) |

### 引脚
- ⚠️ **查原理图确认 USER 按键接的 GPIO**!Nano 不知道具体引脚。常见可能:`PA18` 或 `PB3` 之类,主人查一下。

---

## Step 10:保存并生成代码

1. **File → Save**(Ctrl+S),文件名 `empty.syscfg`
2. SysConfig 自动生成(在 Keil 里点击 Build 前会先 build SysConfig)
3. 生成结果:
   - `ti_msp_dl_config.c`(初始化函数 `SYSCFG_DL_init()`)
   - `ti_msp_dl_config.h`(所有外设宏,如 `UART_0_INST`、`TIMER_0_INST`、`PWM_MOT_INST`、`LED1_PORT/PIN`、`KEY_USER_PORT/PIN` 等)
4. 把这两个文件放到 `BSP/` 目录

---

## Step 11:对接 BSP/port_impl.c

打开 `BSP/port_impl.c`,把顶部的宏替换为 `ti_msp_dl_config.h` 实际生成的名字:

```c
#define UART_DEBUG_INST        UART_0_INST    /* 调试 + OpenMV 共用 */
#define UART_VISION_INST       UART_0_INST    /* 同一个 UART */
#define TIMER_PI_INST          TIMER_0_INST   /* 1kHz */
#define TIMER_PI_IRQN          TIMER_0_INST_INT_IRQN
#define TIMER_TICK_INST        TIMER_1_INST   /* 100Hz */
#define TIMER_TICK_IRQN        TIMER_1_INST_INT_IRQN
#define PWM_MOT_INST           PWM_MOT_INST
#define LED_PORT               LED1_PORT
#define LED_PIN                LED1_PIN_22_PIN

/* PWM 通道索引(SysConfig 生成的 CC 通道号) */
#define PWM_CC_LF              PWM_MOT_CC1_IDX  /* 名称按 ti_msp_dl_config.h 改 */
#define PWM_CC_LR              PWM_MOT_CC2_IDX
#define PWM_CC_RF              PWM_MOT_CC3_IDX
#define PWM_CC_RR              PWM_MOT_CC4_IDX
```

---

## 常见问题

### Q1:SysConfig 报"Pin Conflict"
点红区 → 选 SysConfig 推荐的可用引脚,或者手动改另一个。

### Q2:`PWM_MOT_INST` 找不到
`ti_msp_dl_config.h` 里实际名字可能带后缀,如 `PWM_MOT_INST` 是你在 SysConfig 里填的 `$name`。

### Q3:GPIO 中断进不去
- 确认 NVIC 里使能了对应 IRQ
- 确认 SysConfig 里勾选了中断
- ISR 函数名必须匹配 SysConfig 生成的 `GPIO_<INSTANCE>__IRQHandler`

### Q4:UART RX 中断不触发
- 检查 RX 引脚是否接对(OpenMV TX → 主控 RX)
- 检查共地
- 检查 SysConfig 勾选了 RX 中断

### Q5:生成的代码编译报错"未定义符号"
确保 MSPM0 SDK 版本 ≥ 2.02.00,SysConfig ≥ 1.21.1。

---

## 完整外设清单(自检表)

配完之后,`ti_msp_dl_config.h` 应该有这些宏:

- [ ] `UART_0_INST`,`UART_0_INST_INT_IRQN`
- [ ] `TIMER_0_INST`,`TIMER_0_INST_INT_IRQN`
- [ ] `TIMER_1_INST`,`TIMER_1_INST_INT_IRQN`
- [ ] `PWM_MOT_INST`,`PWM_MOT_*_IDX`(CC 通道索引)
- [ ] `GPIO_DIR_LF/LR/RF/RR_PORT/PIN`
- [ ] `GPIO_ENC_A_LF/LR/RF/RR_PORT/PIN`
- [ ] `GPIO_ENC_B_LF/LR/RF/RR_PORT/PIN`
- [ ] `LED1_PORT`,`LED1_PIN_22_PIN`(或你实际命名的)
- [ ] `KEY_USER_PORT/PIN`(USER 按键)

全部 ✓ 后,代码就能编译通过。
