#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracer-car 串口调试工具
用法:
    python serial_viewer.py                 # 自动找 COM 口
    python serial_viewer.py COM6            # 指定端口
    python serial_viewer.py COM6 460800     # 指定端口 + 波特率
    python serial_viewer.py --hex           # HEX 模式显示
    python serial_viewer.py --no-log        # 不写日志文件
按 Ctrl+C 退出
"""
import sys
import time
import datetime
import argparse
from pathlib import Path

# Windows 控制台默认 cp1252,中文会崩 — 强制 UTF-8
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("[ERR] 缺 pyserial,先装: python -m pip install pyserial")
    sys.exit(1)

DEFAULT_BAUD = 115200
DEFAULT_PORT = "COM6"
LOG_DIR = Path(__file__).resolve().parent / "logs"


def find_port(hint: str = "CH340|CH9102|CP210|USB-Enhanced-SERIAL|USB Serial|UART|FT232|MSP|MSPM0|J-Link|STLink"):
    """自动找一个看起来像 USB 串口的端口,排除蓝牙等"""
    ports = list(list_ports.comports())
    if not ports:
        return None
    import re
    pat = re.compile(hint, re.IGNORECASE)
    skip = re.compile(r"bluetooth|modem|linemode|proprietary", re.IGNORECASE)
    # 优先按 hint 匹配 description,排除蓝牙
    for p in ports:
        if skip.search(p.description):
            continue
        if pat.search(p.description) or pat.search(p.product or ""):
            return p.device
    # 退化:取第一个非蓝牙的
    for p in ports:
        if not skip.search(p.description):
            return p.device
    return None


def open_log(port: str) -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    safe_port = port.replace("/", "_").replace("\\", "_")
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fn = LOG_DIR / f"{ts}_{safe_port}.log"
    return fn


def main():
    ap = argparse.ArgumentParser(description="tracer-car serial viewer")
    ap.add_argument("port", nargs="?", default=None, help=f"COM 端口 (默认自动找,找不到用 {DEFAULT_PORT})")
    ap.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD, help=f"波特率 (默认 {DEFAULT_BAUD})")
    ap.add_argument("--hex", action="store_true", help="HEX 模式显示")
    ap.add_argument("--no-log", action="store_true", help="不写日志文件")
    args = ap.parse_args()

    port = args.port or find_port() or DEFAULT_PORT

    print(f"[viewer] port={port} baud={args.baud} hex={args.hex} log={'off' if args.no_log else 'on'}")
    print(f"[viewer] Ctrl+C 退出\n")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2,
                            bytesize=8, parity='N', stopbits=1)
    except serial.SerialException as e:
        print(f"[ERR] 打不开 {port}: {e}")
        print("      - 关掉占用端口的程序(PuTTY/BSL 工具/其他 viewer)")
        print("      - 拔插 USB")
        sys.exit(1)

    log_file = None if args.no_log else open_log(port)
    if log_file:
        print(f"[viewer] 日志: {log_file}\n")

    start = time.time()
    rx_total = 0
    buf = bytearray()

    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            rx_total += len(chunk)

            if args.hex:
                # HEX 模式:一行 16 字节
                line = " ".join(f"{b:02X}" for b in chunk)
                print(line)
                if log_file:
                    log_file.write_bytes(chunk)
                continue

            # ASCII 模式:按 \n 切行,加时间戳
            buf.extend(chunk)
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(buf[:idx + 1])
                buf = buf[idx + 1:]
                # 去 \r\n
                text = raw.rstrip(b"\r\n").decode(errors="replace")
                ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                line = f"[{ts}] {text}"
                print(line)
                if log_file:
                    log_file.write_bytes(raw)

            # 缓冲区过大(没换行的乱流)兜底刷出
            if len(buf) > 1024:
                text = buf.decode(errors="replace")
                ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{ts}] {text}")
                if log_file:
                    log_file.write_bytes(bytes(buf))
                buf.clear()

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start
        print(f"\n[viewer] 关闭 port={port} rx={rx_total}B  {elapsed:.1f}s")
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
