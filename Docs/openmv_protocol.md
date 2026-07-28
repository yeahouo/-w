# OpenMV ↔ MSPM0G3507 串口通信协议 v1.0

> 本文档面向**视觉端(OpenMV)开发同学**:按此协议实现 OpenMV 端的帧封装与发送,主控就能正确解析。

## 1. 物理层

| 项 | 值 |
|----|----|
| 接口 | UART,TTL 电平(3.3V) |
| 波特率 | **460800**(与主控 `config.h:UART_SHARED_BAUD` 和 `ti_msp_dl_config.c` UART0 配置一致;改动需同步三方) |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |

OpenMV TX → MSPM0G3507 RX
> 单向通信即可(主控不需要回传)。如需双向,预留 GND 共地。

## 2. 帧格式(定长 11 字节)

```
字节号:  0     1     2-3     4      5-6       7-8       9       10
字段:  HDR1  HDR2  ERR  ELEM  DIST_MM  SPEED_MM  TAIL   XOR_CHK
```

| 字段 | 类型 | 说明 |
|------|------|------|
| HDR1 | u8 = 0xAA | 帧头 1 |
| HDR2 | u8 = 0x55 | 帧头 2 |
| ERR  | int16(signed) | **偏差**:赛道中线与画面中心水平线偏移,像素单位。负=左偏,正=右偏。范围建议 [-80, 80]。找不到线时填 `0x7FFF`(32767) |
| ELEM | u8 | 元素枚举(见下表) |
| DIST | u16 | 元素距车头前瞻距离(mm),0 表示未测到 |
| SPEED| u16 | OpenMV 建议速度(mm/s),0 表示用主控默认 |
| TAIL | u8 = 0x0D | 帧尾 |
| XOR_CHECK | u8 | 前 10 字节(字节 0~9)的按位异或 |

### 字节序: **小端**(int16/u16 低字节在前)

## 3. 元素枚举 ELEM

| 值 | 含义 |
|----|------|
| 0  | UNKNOWN(未知/无法识别) |
| 1  | STRAIGHT(直道) |
| 2  | LEFT(左弯) |
| 3  | RIGHT(右弯) |
| 4  | CROSS(十字路口) |
| 5  | ROUNDABOUT(环岛) |
| 6  | SLOPE(坡道) |
| 7  | STOP_LINE(起跑线/停车线) |

## 4. 发送频率

**50 Hz**(每 20ms 发一帧),与主控转向 PD 周期匹配。
OpenMV 处理不过来时降到 25Hz,但**必须稳定**;抖动比慢更可怕。

## 5. OpenMV 端伪代码示例(MicroPython)

```python
from pyb import UART
import struct, time

uart = UART(3, 460800, bits=8, parity=None, stop=1)  # 必须与主控 config.h:UART_SHARED_BAUD 一致

HDR1, HDR2, TAIL = 0xAA, 0x55, 0x0D

def send_frame(err, elem, dist_mm=0, speed_mm=0):
    # int16 -> '<h'  ;  u16 -> '<H'  ;  u8 -> 'B'
    payload = struct.pack('<BBhBHHi'.replace('i','B'),  # 见下方修正
                          HDR1, HDR2,
                          max(-32768, min(32767, err)),
                          elem & 0xFF,
                          dist_mm & 0xFFFF,
                          speed_mm & 0xFFFF,
                          TAIL)
    # 注: struct 格式应写为 '<BBhBHBB'
    xor = 0
    for b in payload[:10]:
        xor ^= b
    uart.write(payload + bytes([xor]))

while True:
    err, elem, dist = vision_pipeline()   # 你的视觉算法
    send_frame(err, elem, dist)
    time.sleep_ms(20)
```

> struct.pack 格式串: `'<BBhBHBB'`
> B=HDR1, B=HDR2, h=ERR(int16), B=ELEM, H=DIST(u16), H=SPEED(u16), B=TAIL。共 10 字节,XOR 单独追加。

## 6. 主控侧容错行为

| 情况 | 主控行为 |
|------|---------|
| XOR 校验失败 | 丢帧,丢失计数 +1 |
| 帧头对不上 | 滑动窗口直到对齐 |
| 200ms 未收到有效帧 | 进入 `TRACKER_LOST` 子状态,降速到 0 并停车 |
| ERR == 0x7FFF | 视为丢线,进入降速策略 |

## 7. 联调建议

1. OpenMV 端先固定发 `err=0, elem=1` 直道帧,主控能看到稳定偏差 0
2. 再让 OpenMV 跑视觉算法,主控打印原始 `err` 字段,验证符号方向
3. 最后联调元素识别,逐个元素场景测试
