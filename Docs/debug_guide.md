# 调试与排查指南

> 寻迹小车的"可观测性总入口"。出问题时先看这份文档,按流程定位。

---

## 1. 日志体系(文本输出)

### 1.1 日志级别

代码里所有日志调用形如 `LOG_I("FSM", "...")`,四档级别运行时可调:

| 级别 | 宏 | 字符 | 用途 |
|------|-----|------|------|
| DEBUG | `LOG_D` | `[D]` | 最详细(状态切换、PI 输出),上线后建议关 |
| INFO  | `LOG_I` | `[I]` | 关键事件(初始化完成、元素变化、状态切换) |
| WARN  | `LOG_W` | `[W]` | 异常但可恢复(丢帧、丢线) |
| ERROR | `LOG_E` | `[E]` | 严重故障(通信断、堵转、PI 发散) |

**运行时切换**(在代码里调用,或日后通过调试命令):
```c
Log_SetLevel(LOG_LEVEL_INFO);    /* 关掉 DEBUG 减少日志量 */
Log_SetLevel(LOG_LEVEL_WARN);    /* 比赛实战用,只留 WARN/ERROR */
```

### 1.2 日志格式

```
[12345][I][FSM] TRACKING -> ELEMENT
└─ms─┘ └级┘ └tag┘ └─── 消息 ────────────┘
```

- 时间戳来自 SysTick 毫秒计数
- 模块标签:`BOOT` `FSM` `TRACK` `MOT` `VIS` `PID` 等

### 1.3 异步输出原理

`LOG_xxx` 调用时:**格式化 + memcpy 进环形 buffer(8KB),不阻塞**(中断里也能用)。
`Log_Poll()` 在主循环调用:从 buffer 抽出,经 UART 发送。

**buffer 满时会丢最旧字节**,统计看 `Log_GetDroppedCount()`。如果这个值持续增长,说明:
- 日志级别太低(DEBUG 全开)
- UART 波特率太低
- 主循环被某个慢任务卡住

---

## 2. 实时遥测(波形输出)

### 2.1 输出格式(CSV)

`Telemetry_Poll()` 在主循环按设定频率(默认 50Hz)输出一行 CSV,字段顺序:

```
ms,err,elem,state,last_err,vframes,vdrops,
tlf,tlr,trf,trr,alf,alr,arf,arr,plf,plr,prf,prr
```

| 字段 | 含义 |
|------|------|
| `ms` | 时间戳(ms) |
| `err` | 循迹偏差(像素,-80~80,32767=丢线) |
| `elem` | 元素枚举(0=UNKNOWN ... 7=STOP_LINE,见 `config.h`) |
| `state` | 循迹状态(0=OK 1=LOST 2=BLIND) |
| `last_err` | 最近错误码(见 `errcode.h`) |
| `vframes` | OpenMV 累计接收帧数 |
| `vdrops` | OpenMV 累计丢帧数 |
| `tXX` | 四轮目标速度(mm/s,顺序 LF/LR/RF/RR) |
| `aXX` | 四轮实测速度(mm/s) |
| `pXX` | 四轮 PWM 输出(-1000~1000) |

### 2.2 上位机配置

#### SerialPlot(推荐,免费开源)

1. 下载 https://serialplot.sourceforge.io/
2. 打开串口,波特率匹配调试串口(如 460800 或 921600,越高越好)
3. 配置:
   - Frame format: **CSV**
   - Number of channels: **19**(对应上表 19 个数值字段)
   - 自动识别 `#` 开头的行为表头

#### VOFA+

1. 协议选 **RawData**
2. 分隔符 `,`
3. 帧尾 `\r\n`

#### Python 自写示例

```python
import serial, csv, time
ser = serial.Serial('COM5', 460800, timeout=1)
with open('log.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['ms','err','elem','state','last_err','vframes','vdrops',
                'tlf','tlr','trf','trr','alf','alr','arf','arr',
                'plf','plr','prf','prr'])
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line.startswith('#'): continue
        if line.count(',') == 18:
            w.writerow(line.split(','))
            f.flush()
```

### 2.3 遥测频率运行时调整

```c
Telemetry_SetRate(100);   /* 100Hz 全速(波特率要够) */
Telemetry_SetRate(10);    /* 10Hz 省带宽 */
Telemetry_SetFmt(TEL_FMT_OFF);   /* 关闭遥测 */
```

### 2.4 关键调参场景

#### PID 调参
画图:`target_lf` 和 `actual_lf` 同图,看 PI 跟随。给阶跃目标(0→300 mm/s),看:
- **响应慢** → 加 Kp
- **过冲** → 减 Kp 或加 Kd
- **稳态误差大** → 加 Ki
- **持续震荡** → Kp 太大,减半再加

#### 循迹 PD 调参
画图:`err` 单图,加 `state` 标志位。看偏差是否收敛到 0、丢线率(`vdrops/vframes`)。

#### PWM 饱和诊断
画图:`plf/plr/prf/prr` 是否长时间打满 ±1000 → 堵转或 PI 发散。

---

## 3. LED 状态指示

板载 LED 在不同 FsmState_t 下的闪烁模式:

| 状态 | LED 模式 | 视觉效果 |
|------|---------|----------|
| IDLE | 慢闪 1Hz | 慢节奏闪(等启动) |
| START | 快闪 5Hz | 短促闪烁(加速起步) |
| TRACKING | 常亮 | 一直亮(正常循迹) |
| ELEMENT | 双闪 | 短-短-长间歇 |
| FINISH | 长亮 2 秒后灭 | 表示已到终点 |
| ERROR | 急闪 10Hz | 快速闪(报警,马上看日志!) |

**故障快速判断**:看 LED 闪烁频率就知道车在什么状态。出问题直接看 LED → 对照表 → 查日志。

---

## 4. 错误码速查表

| 码 | 名 | 含义 | 触发位置 | 排查建议 |
|----|----|------|---------|---------|
| 1 | `VISION_TIMEOUT` | OpenMV 200ms 没来帧 | `tracker.c` | 检查 OpenMV TX→主控 RX 接线 / 波特率 / OpenMV 是否在跑 |
| 2 | `VISION_CRC_FAIL` | XOR 校验或 TAIL 校验失败 | `uart_vision.c` | 波特率不匹配 / 线材干扰 / OpenMV 端协议写错 |
| 3 | `VISION_NO_HDR` | 长时间未对齐帧头 | (预留) | OpenMV 端没在发数据 |
| 4 | `VISION_LOST_LINE` | 偏差字段 = 0x7FFF | `tracker.c` | OpenMV 找不到线 → 视觉算法/光照/赛道对比度问题 |
| 5 | `MOTOR_STALL` | 目标>50 但实测<10 持续 300ms | `motor.c` | 卡住、电机线断、H 桥烧、PI 反极性 |
| 6 | `MOTOR_OVERCURRENT` | 过流 | (预留) | 加电流采样后启用 |
| 7 | `PI_DIVERGE` | 输出打满 800ms | `motor.c` | 编码器反极性 / PI 参数过大 / 死区设错 |
| 8 | `BATTERY_LOW` | 电池电压低 | (预留) | 加 ADC 采样后启用 |
| 9 | `TASK_OVERRUN` | 周期任务超时 | (预留) | 减小 PI/Tracker 计算量,或提高主频 |

日志里会看到:`lastErr=VISION_TIMEOUT(5)` — 第一个字段是错误名,括号里是累计错误总数。

---

## 5. 常见故障排查流程

### 故障 A:开机后 LED 不亮,日志没有任何输出

1. 检查电源、烧录是否成功
2. 用 JLink/ST-Link 单步调试,看是否进入 `main()`
3. 检查 `SYSCFG_DL_init()` 是否卡死(外设配置错)
4. 检查 UART 调试串口的 TX/RX 是否接反

### 故障 B:LED 一直急闪(卡在 ERROR)

查日志,看 ERROR 是从哪儿转过来的:
- 从 TRACKING → ERROR 且 `lastErr=VISION_TIMEOUT`:OpenMV 没通信
- 从 ELEMENT → ERROR:元素处理中丢线
- 启动后立刻进 ERROR:PI 反极性导致 `PI_DIVERGE`

### 故障 C:循迹时车左右抖

波形上看 `err` 来回摆:
- Kp 太大 → 减半 Kp
- 没加滤波 → 检查 `TRACK_E_FILTER_ALPHA` 是否生效
- PD 频率不够 → 确认 `Tracker_Update` 真的跑在 50Hz

### 故障 D:弯道冲出赛道

- 看波形 `elem` 是否识别到 LEFT/RIGHT
- 看 `target` 速度,弯道时应该降到 `SPEED_TARGET_CURVE`
- PD 输出 `omega` 限幅是否太小(`TRACK_PD_OUT_MAX`)

### 故障 E:电机不转但有 PWM 输出

- H 桥 EN 脚是否拉高
- 方向脚是否正确
- 编码器线是否接对(报 `MOTOR_STALL` 是典型)
- PWM 极性反了(改 `motor_invert[]`)

### 故障 F:OpenMV 帧率不对

波形看 `vframes` 增长率,正常应该 50Hz(每 20ms +1):
- 增长慢:OpenMV 端处理不过来
- 不增长:OpenMV 没发,或主控 UART 接线错
- `vdrops` 比例高:协议写错或线材干扰

### 故障 G:CPU 跑不过来(周期任务超时)

预留 `ERR_TASK_OVERRUN`,日后通过测量两次 `Port_OnTick1kHz` 之间的实际间隔判断。

---

## 6. BSP 移植对接清单(SysConfig 配置)

主人在 SysConfig 里需配置以下外设,然后在 `BSP/port_impl.c` 顶部把宏名替换成实际生成的名字:

| 外设 | 用途 | 关键参数 |
|------|------|---------|
| UART0 | 调试 + 日志 + 遥测 + **OpenMV RX 共享** | **460800**(锁定),8N1,RX 中断使能 |
> 注: 不再使用独立 UART1。OpenMV TX → UART0 RX (PA11),与调试日志共享同一 UART。波特率必须 460800。
| TIMER0 | 1kHz tick(PI + 系统时钟) | 周期 1ms,中断使能 |
| TIMER1 | 100Hz tick(编码器+遥测+LED) | 周期 10ms,或复用 TIMER0 分频 |
| PWM | 电机 PWM 4 通道 | 频率 20kHz,周期 1000 |
| TIM_ENCODER × 4 | 编码器正交解码(可选) | AB 相 |
| GPIO_OUT | LED | 板载 LED |
| GPIO_IN | 启动按键(可选) | 上拉输入,EXTI 下降沿 |

### 配置步骤

1. **打开 SysConfig**(Keil 内置或独立工具)
2. 逐个添加上述外设,在右侧设参数,引脚根据板子原理图分配
3. **保存** → 生成 `ti_msp_dl_config.c/h`
4. 把 `BSP/port_impl.c` 顶部的宏(如 `UART_DEBUG_INST`、`TIMER_PI_INST`、`PWM_MOT_INST` 等)替换为 `ti_msp_dl_config.h` 里实际生成的名字
5. 编译下载

### 接线建议

- OpenMV RX ← 主控 TX(不需要,主控只接收)
- OpenMV TX → 主控 UART1 RX
- 共地(GND 必须接)
- 主控 UART0 TX → USB-TTL → PC(调试)

---

## 7. 调试 checklist(每次上车前过一遍)

- [ ] 烧录成功,LED 慢闪(IDLE)
- [ ] 调试串口能看到 `[BOOT] === tracer-car firmware v0.1 boot ===`
- [ ] 串口能看到所有模块 `init ok` 日志
- [ ] 用串口工具发一帧假数据 `0xAA 0x55 0x00 0x00 0x01 0x00 0x00 0x00 0x00 0x0D <xor>`,看日志 `first frame received`
- [ ] 遥测波形能正常显示
- [ ] 启动按键(或命令)触发后,LED 切换到 TRACKING 常亮
- [ ] 用手转电机轮子,波形上 `actual` 应有变化(编码器 OK)
- [ ] 给一个目标速度,看 PI 闭环能跟得上
- [ ] 上赛道实测
