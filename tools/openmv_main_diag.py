"""
OpenMV 视觉感知端 — 诊断版
================================
功能: 跟 openmv_main.py 一样,但加大量 print + try/except,
      让主人在 OpenMV IDE 里能看到代码到底卡在哪一步

用法:
    1. OpenMV IDE 连上 OpenMV
    2. 打开本文件
    3. 点左下角绿色 ▶ 运行
    4. 看 IDE 底部 Serial Terminal 输出
"""

import sensor
import time
from pyb import UART, LED

print("=" * 50)
print("[STEP 1] boot ok, basic import done")
print("=" * 50)

# ============================================================
#  配置
# ============================================================
THRESHOLD = (0, 64)
ROI = (0, 30, 160, 90)
UART_BAUD = 460800            # 必须跟 MSPM0 config.h UART_SHARED_BAUD 一致

# ============================================================
#  协议
# ============================================================
HDR1, HDR2, TAIL = 0xAA, 0x55, 0x0D
ERR_LOST = 0x7FFF
ELEM_STRAIGHT = 1


def build_frame(err, elem=ELEM_STRAIGHT, dist=0, speed=0):
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
#  STEP 2: 摄像头
# ============================================================
print("[STEP 2] init camera...")
try:
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(False)
    sensor.skip_frames(time=500)
    CENTER_X = sensor.width() // 2
    IMG_H    = sensor.height()
    print("      camera ok: %dx%d, center_x=%d" % (sensor.width(), sensor.height(), CENTER_X))
except Exception as e:
    print("      [ERR] camera init failed:", e)
    raise

# ============================================================
#  STEP 3: UART
# ============================================================
print("[STEP 3] init UART3 @%d..." % UART_BAUD)
try:
    uart = UART(3, UART_BAUD, timeout=100, timeout_char=10)
    print("      UART3 ok")
    # 立即发一串测试字节,验证 UART 能 write
    test_frame = build_frame(0)
    uart.write(test_frame)
    print("      UART3 write test ok, sent bytes:", len(test_frame))
    print("      bytes:", " ".join("%02X" % b for b in test_frame))
except Exception as e:
    print("      [ERR] UART3 init failed:", e)
    raise

# ============================================================
#  STEP 4: LED
# ============================================================
print("[STEP 4] init LED...")
led = LED(1)
led.off()
for _ in range(3):
    led.on();  time.sleep_ms(100)
    led.off(); time.sleep_ms(100)

print("=" * 50)
print("[BOOT] all init done, entering main loop")
print("frame: AA 55 ERR_LO ERR_HI ELEM DIST_LO DIST_HI SPEED_LO SPEED_HI 0D XOR")
print("=" * 50)

# ============================================================
#  重心法
# ============================================================
def detect_line_x(img):
    img.binary([THRESHOLD], invert=True)
    x0, y0, w, h = ROI
    wx_sum = 0
    w_sum  = 0
    step = 2
    for y in range(y0, y0 + h, step):
        weight = y / IMG_H
        for x in range(x0, x0 + w, step):
            if img.get_pixel(x, y) == 0:
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
err_cnt   = 0
last_print_ms = time.ticks_ms()

try:
    while True:
        clock.tick()
        img = sensor.snapshot()
        line_x = detect_line_x(img)

        if line_x is not None:
            err = CENTER_X - line_x
            err = max(-80, min(80, err))
            elem = ELEM_STRAIGHT
            frame_cnt += 1
            led.off()
        else:
            err  = ERR_LOST
            elem = 0
            lost_cnt += 1
            led.on()

        frame = build_frame(err, elem=elem)
        uart.write(frame)

        # 每 1 秒打印一次状态 + UART 发送的字节数
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) > 1000:
            last_print_ms = now
            print("FPS:%.1f line=%s err=%d fc=%d lc=%d uart_write=%dB" % (
                clock.fps(),
                str(line_x) if line_x is not None else "LOST",
                err if err != ERR_LOST else 9999,
                frame_cnt, lost_cnt, (frame_cnt + lost_cnt) * 11))

except Exception as e:
    print("[ERR] main loop crashed:", e)
    err_cnt += 1
    # 闪 LED 报错
    while True:
        led.on();  time.sleep_ms(50)
        led.off(); time.sleep_ms(50)
