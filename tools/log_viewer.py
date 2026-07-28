#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracer-car LINE_FOLLOW v6 主控日志查看器
==================================================
解析 line_follow v6 main.c (5 路灰度) 输出的诊断日志,
带颜色高亮 + 5 位传感器可视化 + 状态告警。

用法:
    python log_viewer.py                 # 自动找 COM 口
    python log_viewer.py COM6            # 指定端口
    python log_viewer.py COM6 115200     # 指定端口 + 波特率

按 Ctrl+C 退出。

主控日志格式 (line_follow v6 main.c 末尾):
    t=100 state=WAIT_LINE bits=0 err=0 vL=0 vR=0 lost=100
        L1=0 L2=0 M=0 R1=0 R2=0
"""
import sys
import time
import datetime
import argparse
import re
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
    print("[ERR] 缺 pyserial, 先装: python -m pip install pyserial")
    sys.exit(1)

DEFAULT_BAUD = 115200
DEFAULT_PORT = "COM6"
LOG_DIR = Path(__file__).resolve().parent / "logs"

# ANSI 颜色代码(Windows 10+ 终端支持)
class C:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN    = "\033[36m"
    GRAY    = "\033[90m"

# 启用 Windows ANSI 颜色
if sys.platform == "win32":
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:
        pass


def find_port() -> str | None:
    """自动找一个看起来像 USB 串口的端口"""
    pat = re.compile(
        r"CH340|CH9102|CP210|USB-Enhanced-SERIAL|USB Serial|UART|FT232|MSP|J-Link|STLink",
        re.IGNORECASE,
    )
    skip = re.compile(r"bluetooth|modem|linemode|proprietary", re.IGNORECASE)
    ports = list(list_ports.comports())
    for p in ports:
        if skip.search(p.description):
            continue
        if pat.search(p.description) or pat.search(p.product or ""):
            return p.device
    for p in ports:
        if not skip.search(p.description):
            return p.device
    return None


def open_log(port: str) -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    safe_port = port.replace("/", "_").replace("\\", "_")
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"{ts}_{safe_port}.log"


# 状态颜色映射
STATE_COLOR = {
    "WAIT_LINE": C.YELLOW,
    "TRACKING":  C.GREEN,
    "LOST":      C.MAGENTA,
    "ERROR":     C.RED,
}

LINE_RE = re.compile(r"(\w+)=(-?\d+)")


def parse_line(text: str) -> dict:
    """解析一行日志,返回 dict"""
    return {k: int(v) for k, v in LINE_RE.findall(text)}


def color_state(s: str) -> str:
    return STATE_COLOR.get(s, C.GRAY) + s + C.RESET


def render_sensor_bar(bits: int) -> str:
    """渲染 5 位传感器可视化条 — 黑线=高亮, 白底=灰点"""
    # bits: bit0=L1 bit1=L2 bit2=M bit3=R1 bit4=R2
    labels = ["L1", "L2", "M ", "R1", "R2"]
    parts = []
    for i, lbl in enumerate(labels):
        on = (bits >> i) & 1
        if on:
            parts.append(f"{C.BOLD}{C.GREEN}{lbl}{C.RESET}")
        else:
            parts.append(f"{C.GRAY}.{lbl}{C.RESET}")
    return " ".join(parts)


def render_row(d: dict) -> str:
    """渲染一行漂亮的输出"""
    state = d.get("state_name", "?")
    err = d.get("err", 0)        # 主控打成 -99 表示丢线
    bits = d.get("bits", 0)
    vl = d.get("vL", 0)
    vr = d.get("vR", 0)
    lost = d.get("lost", 0)

    # err 着色: -99 = 丢线红, |err|>=2 红, |err|==1 黄, 0 绿
    if err == -99:
        err_str = f"{C.RED}{err:>4}{C.RESET} (丢线)"
    elif abs(err) >= 2:
        err_str = f"{C.RED}{err:>4}{C.RESET}"
    elif abs(err) == 1:
        err_str = f"{C.YELLOW}{err:>4}{C.RESET}"
    else:
        err_str = f"{C.GREEN}{err:>4}{C.RESET}"

    # duty 着色: 0=灰, >0=青
    def duty_str(v: int) -> str:
        return f"{C.GRAY}{v}{C.RESET}" if v == 0 else f"{C.CYAN}{v}{C.RESET}"
    vl_str = duty_str(vl)
    vr_str = duty_str(vr)

    # lost 颜色: <200 绿, <1000 黄, >1000 红
    if lost < 200:
        lost_str = f"{C.GREEN}{lost}{C.RESET}"
    elif lost < 1000:
        lost_str = f"{C.YELLOW}{lost}{C.RESET}"
    else:
        lost_str = f"{C.RED}{lost}{C.RESET}"

    sensor_bar = render_sensor_bar(bits)

    return (
        f"{C.GRAY}{d.get('t', 0):>6}{C.RESET}ms "
        f"{color_state(state):<22} "
        f"[{sensor_bar}]  "
        f"err={err_str:<18} "
        f"L={vl_str} R={vr_str}  "
        f"lost={lost_str}"
    )


def render_banner(port: str, baud: int, log_path: Path | None = None) -> None:
    print(f"{C.BOLD}{C.CYAN}")
    print("=" * 72)
    print("  tracer-car LINE_FOLLOW v6 (5x grayscale) 日志查看器")
    print(f"  port={port}  baud={baud}  日志={log_path or 'off'}")
    print("  Ctrl+C 退出")
    print("=" * 72)
    print(f"{C.RESET}")

    print(f"{C.DIM}字段说明:{C.RESET}")
    print(f"  {C.BOLD}state{C.RESET}  小车状态 (WAIT_LINE/TRACKING/LOST/ERROR)")
    print(f"  {C.BOLD}bits{C.RESET}   5 位传感器 bitmask (bit0=L1 .. bit4=R2)")
    print(f"  {C.BOLD}err{C.RESET}    偏差 (-2..+2, -99=丢线)")
    print(f"  {C.BOLD}vL/vR{C.RESET}  左右轮 PWM 占空比 (0-10 档)")
    print(f"  {C.BOLD}lost{C.RESET}   距上次见到线的 ms 数 (越小越好)")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description="tracer-car LINE_FOLLOW v6 日志查看器")
    ap.add_argument("port", nargs="?", default=None, help="COM 端口 (默认自动找)")
    ap.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--no-log", action="store_true", help="不写日志文件")
    args = ap.parse_args()

    port = args.port or find_port() or DEFAULT_PORT

    log_path = None if args.no_log else open_log(port)
    render_banner(port, args.baud, log_path)
    if log_path:
        log_fp = log_path.open("wb")
    else:
        log_fp = None

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2,
                            bytesize=8, parity='N', stopbits=1)
    except serial.SerialException as e:
        print(f"{C.RED}[ERR] 打不开 {port}: {e}{C.RESET}")
        print("      - 关掉占用端口的程序 (PuTTY/BSL/其他 viewer)")
        print("      - 拔插 USB")
        sys.exit(1)

    buf = bytearray()
    boot_seen = False

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                if log_fp:
                    log_fp.write(chunk)
                buf.extend(chunk)

            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(buf[:idx + 1])
                buf = buf[idx + 1:]
                text = raw.rstrip(b"\r\n").decode(errors="replace")

                # 启动横幅直接打印
                if not boot_seen:
                    if ("tracer-car" in text or "===" in text
                            or "waiting" in text):
                        print(f"{C.BOLD}{C.GREEN}<< {text}{C.RESET}")
                        if "waiting" in text:
                            boot_seen = True
                        continue
                    if any(k in text for k in
                           ("sensor:", "motor :", "algo  :")):
                        print(f"{C.GREEN}<< {text}{C.RESET}")
                        continue

                # 解析 t=... 字段
                d = parse_line(text)
                if "t" not in d:
                    print(f"{C.GRAY}{text}{C.RESET}")
                    continue

                # state 是字符串(line_follow v6 输出 state=WAIT_LINE 而非数字),
                # LINE_RE 只匹配数字, 所以单独提取
                m = re.search(r"state=(\w+)", text)
                d["state_name"] = m.group(1) if m else "?"

                print(render_row(d))

                # 异常告警
                if d["state_name"] == "ERROR":
                    print(f"{C.RED}{C.BOLD}  [告警] ERROR 状态! 丢线已超过 2s{C.RESET}")
                elif d["state_name"] == "LOST" and d.get("lost", 0) > 1000:
                    print(f"{C.YELLOW}  [警告] LOST 状态, lost={d['lost']}ms 接近 ERROR 阈值{C.RESET}")

            # 缓冲过大兜底
            if len(buf) > 1024:
                text = buf.decode(errors="replace")
                print(f"{C.GRAY}{text}{C.RESET}")
                buf.clear()

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{C.GRAY}[viewer] 关闭{C.RESET}")
        try:
            ser.close()
        except Exception:
            pass
        if log_fp:
            log_fp.close()
            print(f"{C.GRAY}[viewer] 日志保存到 {log_path}{C.RESET}")


if __name__ == "__main__":
    main()
