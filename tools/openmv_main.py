"""
OpenMV 视觉感知端 — tracer-car 主从架构
======================================
功能:
    摄像头取图 → 重心法找线 → 通过 UART3 发 11 字节帧给 MSPM0

!! 本文件只做感知,不做控制!!
    - 不驱动电机 (MSPM0 那边跑模糊控制)
    - 不跑 PD/Motor 类 (原版那套删掉了)

接线:
    OpenMV PB10 (UART3 TX) ──→ MSPM0 PA11 (UART0 RX)
    OpenMV GND             ──→ MSPM0 GND         (必须共地!)

帧格式 (11 字节, LSB first):
    AA 55 ERR_LO ERR_HI ELEM DIST_LO DIST_HI SPEED_LO SPEED_HI 0D XOR
    err  : int16, = center_x - line_x, 范围 -80..+80, 丢线=0x7FFF
    elem : 1=STRAIGHT 2=LEFT 3=RIGHT (暂未做元素识别,统一发 1)
    dist : mm, 暂发 0
    speed: mm/s, 暂发 0
    XOR  : 字节 0..9 异或 (含 HDR/TAIL)

波特率: 115200 (跟 MSPM0 配置一致)
"""

import sensor
import image
import time
from pyb import UART, LED

# ============================================================
#  配置
# ============================================================
THRESHOLD = (0, 64)          # 黑线阈值 (灰度 0..64)
ROI = (0, 30, 160, 90)       # 感兴趣区域 (x, y, w, h)
FRAMESIZE = sensor.QQVGA      # 160 x 120
UART_BAUD = 115200            # 必须跟 MSPM0 ti_msp_dl_config.c UART0 波特率一致

# ============================================================
#  协议常量
# ============================================================
HDR1, HDR2, TAIL = 0xAA, 0x55, 0x0D
ERR_LOST = 0x7FFF             # 丢线标志
ELEM_STRAIGHT = 1


def build_frame(err, elem=ELEM_STRAIGHT, dist=0, speed=0):
    """构造 11 字节帧"""
    err_u16 = err & 0xFFFF
    body = bytes([
        HDR1, HDR2,
        err_u16 & 0xFF, (err_u16 >> 8) & 0xFF,
        elem & 0xFF,
        dist & 0xFF, (dist >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF,
        TAIL,
    ])
    xor = 0
    for b in body:
        xor ^= b
    return body + bytes([xor])


# ============================================================
#  摄像头初始化
# ============================================================
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(FRAMESIZE)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.skip_frames(time=500)  # 等曝光稳定

CENTER_X = sensor.width() // 2   # 80
IMG_H    = sensor.height()       # 120

# ============================================================
#  UART3 — 默认就是 PB10 (TX) / PB11 (RX)
#  在 OpenMV Cam H7 上:
#     PB10 = USART3_TX = 丝印 "P4"
#     PB11 = USART3_RX = 丝印 "P5"
#  即焊盘 PB10 跟丝印 P4 是同一个物理脚,主人接 PB10 就对
# ============================================================
uart = UART(3, UART_BAUD, timeout=100, timeout_char=10)

# ============================================================
#  LED 状态指示
# ============================================================
led = LED(1)
led.off()
# 启动闪 3 次表示初始化完成
for _ in range(3):
    led.on();  time.sleep_ms(80)
    led.off(); time.sleep_ms(80)

print("=== OpenMV vision tx ===")
print("baud=%d  center_x=%d  ROI=%s" % (UART_BAUD, CENTER_X, str(ROI)))
print("frame: AA 55 ERR_LO ERR_HI ELEM DIST_LO DIST_HI SPEED_LO SPEED_HI 0D XOR")

# ============================================================
#  重心法找线 (沿用原版 LineDetector.detect 的逻辑)
# ============================================================
def detect_line_x(img):
    """返回 line_x (像素), 或 None (丢线)"""
    img.binary([THRESHOLD], invert=True)
    x0, y0, w, h = ROI
    wx_sum = 0
    w_sum  = 0
    step = 2
    for y in range(y0, y0 + h, step):
        weight = y / IMG_H  # 越靠下的行权重越大
        for x in range(x0, x0 + w, step):
            if img.get_pixel(x, y) == 0:  # 0 = 前景(线条)
                wx_sum += x * weight
                w_sum  += weight
    if w_sum < 2:
        return None
    return int(wx_sum / w_sum)


# ============================================================
#  主循环
# ============================================================
clock = time.clock()
frame_cnt = 0
lost_cnt  = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    line_x = detect_line_x(img)

    if line_x is not None:
        err = CENTER_X - line_x           # 线在左 → line_x<80 → err>0
        err = max(-80, min(80, err))      # 限幅
        elem = ELEM_STRAIGHT
        frame_cnt += 1
        led.off()
    else:
        err  = ERR_LOST
        elem = 0
        lost_cnt += 1
        led.on()  # 丢线时亮灯

    frame = build_frame(err, elem=elem)
    uart.write(frame)

    # 每 50 帧打印一次状态(只到 IDE, 不影响 UART3)
    if (frame_cnt + lost_cnt) % 50 == 0:
        print("FPS:%.1f line=%s err=%d fc=%d lc=%d" % (
            clock.fps(),
            str(line_x) if line_x is not None else "LOST",
            err if err != ERR_LOST else 9999,
            frame_cnt, lost_cnt))
