#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracer-car 麦克风录音工具

把麦克风声音录到 txt (每行一个采样值) 和 wav (方便回放)。
默认: 16 kHz / 单声道, 持续录音直到按任意键停止。

用法:
    python record_mic.py                      # 默认: 一直录, 按任意键停止
    python record_mic.py --duration 5         # 固定录 5 秒
    python record_mic.py --rate 44100         # 44.1 kHz
    python record_mic.py --output my_rec      # 自定义文件名 (生成 .wav + .txt)
    python record_mic.py --list-devices       # 列出可用音频设备
    python record_mic.py --device 2           # 指定设备索引
    python record_mic.py --txt-only           # 只生成 txt
    python record_mic.py --no-txt             # 只生成 wav

按键停止模式下, 按任意键或 Ctrl+C 都能停止。

依赖:
    pip install sounddevice numpy
"""
import sys
import argparse
import datetime
import time
from pathlib import Path

# Windows 控制台默认 cp1252,中文会崩 — 强制 UTF-8
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import sounddevice as sd
    import numpy as np
    import wave
except ImportError as e:
    print(f"[缺库] {e}")
    print("请安装: python -m pip install sounddevice numpy")
    sys.exit(1)

# Windows 用 msvcrt 检测按键 (任意键停止)
if sys.platform == "win32":
    import msvcrt


def list_input_devices() -> None:
    """列出可用输入设备"""
    print("可用输入设备:")
    devices = sd.query_devices()
    for i, d in enumerate(devices):
        if d['max_input_channels'] > 0:
            print(f"  [{i}] {d['name']}  (输入通道: {d['max_input_channels']}, "
                  f"默认采样率: {d['default_samplerate']:.0f} Hz)")


def key_pressed() -> bool:
    """检查是否有按键 (Windows)"""
    if sys.platform == "win32":
        return msvcrt.kbhit()
    # 非 Windows 暂不支持按键停止, 只能 Ctrl+C
    return False


def record_fixed(duration: float, sample_rate: int, channels: int, device: int | None) -> np.ndarray:
    """固定时长录音"""
    print(f"[录音] 固定 {duration:.1f}s @ {sample_rate} Hz, {channels} 通道"
          f"{f', 设备 [{device}]' if device is not None else ', 默认设备'}")
    print("[录音] 开始 — 请说话...")
    audio = sd.rec(int(duration * sample_rate),
                   samplerate=sample_rate,
                   channels=channels,
                   dtype='float32',
                   device=device)
    sd.wait()
    print("\n[录音] 完成")
    return audio


def record_until_key(sample_rate: int, channels: int, device: int | None) -> np.ndarray:
    """一直录音, 直到按任意键停止 (Windows 用 msvcrt)"""
    print(f"[录音] 持续录音 @ {sample_rate} Hz, {channels} 通道"
          f"{f', 设备 [{device}]' if device is not None else ', 默认设备'}")
    print("[录音] 开始 — 按任意键停止 (或 Ctrl+C)...")
    print("[录音] 提示: 终端需保持焦点才能捕获按键\n")

    chunks: list[np.ndarray] = []
    overflow_count = 0
    stream = sd.InputStream(samplerate=sample_rate,
                            channels=channels,
                            dtype='float32',
                            device=device)
    stream.start()
    start = time.time()

    try:
        while True:
            # 检测按键
            if key_pressed():
                # 清空键盘缓冲
                while msvcrt.kbhit():
                    msvcrt.getch()
                break

            # 读取一块
            data, overflowed = stream.read(1024)
            if data.shape[0] > 0:
                chunks.append(data.copy())
            if overflowed:
                overflow_count += 1

            # 进度显示
            elapsed = time.time() - start
            total_samples = sum(c.shape[0] for c in chunks)
            sys.stdout.write(
                f"\r[录音] {elapsed:6.1f}s | {total_samples} 采样 | "
                f"chunks: {len(chunks)} | 溢出: {overflow_count}"
            )
            sys.stdout.flush()
            # 减少进度刷屏频率
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[录音] Ctrl+C 中止")
    finally:
        stream.stop()
        stream.close()

    elapsed = time.time() - start
    print(f"\n[录音] 完成, 实际录了 {elapsed:.2f}s")

    if not chunks:
        return np.zeros((0, channels), dtype='float32')
    return np.concatenate(chunks, axis=0)


def save_wav(path: Path, audio: np.ndarray, sample_rate: int, channels: int) -> None:
    """保存 WAV (16-bit PCM, 小端)"""
    pcm = np.clip(audio, -1.0, 1.0)
    pcm = (pcm * 32767).astype('<i2')
    with wave.open(str(path), 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm.tobytes())


def save_txt(path: Path, audio_flat: np.ndarray) -> None:
    """保存 txt: 每行一个 float 采样值, 范围 [-1, 1]"""
    np.savetxt(str(path), audio_flat, fmt='%.6f')


def main() -> int:
    parser = argparse.ArgumentParser(description="录麦克风声音到 txt/wav (默认按键停止)")
    parser.add_argument("--duration", type=float, default=None,
                        help="固定录音时长 (秒), 不指定则按键停止")
    parser.add_argument("--rate", type=int, default=16000,
                        help="采样率 (Hz), 默认 16000")
    parser.add_argument("--channels", type=int, default=1, choices=[1, 2],
                        help="通道数, 默认 1 (单声道)")
    parser.add_argument("--output", type=str, default=None,
                        help="输出文件名 (不带扩展名), 默认按时间戳")
    parser.add_argument("--device", type=int, default=None,
                        help="音频设备索引 (用 --list-devices 查询)")
    parser.add_argument("--txt-only", action="store_true",
                        help="只生成 txt")
    parser.add_argument("--no-txt", action="store_true",
                        help="只生成 wav, 不生成 txt")
    parser.add_argument("--list-devices", action="store_true",
                        help="列出可用音频设备后退出")
    args = parser.parse_args()

    if args.list_devices:
        list_input_devices()
        return 0

    if args.duration is not None and args.duration <= 0:
        print("[错误] --duration 必须 > 0")
        return 1

    # 输出目录: tools/recordings/
    out_dir = Path(__file__).parent / "recordings"
    out_dir.mkdir(exist_ok=True)

    # 文件名
    base = args.output if args.output else \
        f"recording_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # 录音
    try:
        if args.duration is not None:
            audio = record_fixed(args.duration, args.rate, args.channels, args.device)
        else:
            audio = record_until_key(args.rate, args.channels, args.device)
    except sd.PortAudioError as e:
        print(f"[错误] 录音失败: {e}")
        print("[提示] 检查麦克风是否连接, 或用 --list-devices 查看设备, --device N 指定")
        return 2

    if audio.shape[0] == 0:
        print("[警告] 没有录到数据")
        return 3

    # 统计
    samples = audio.shape[0]
    print(f"[信息] 采样点数: {samples}")
    print(f"[信息] 实际时长: {samples / args.rate:.3f}s")
    print(f"[信息] 幅度范围: [{audio.min():.4f}, {audio.max():.4f}]")
    print(f"[信息] RMS: {float(np.sqrt(np.mean(audio ** 2))):.4f}")

    # 保存
    saved: list[Path] = []

    if not args.txt_only:
        wav_path = out_dir / f"{base}.wav"
        save_wav(wav_path, audio, args.rate, args.channels)
        saved.append(wav_path)

    if not args.no_txt:
        # txt 只保存单通道 (如果是立体声, 取左声道)
        audio_flat = audio[:, 0] if audio.ndim == 2 else audio
        txt_path = out_dir / f"{base}.txt"
        save_txt(txt_path, audio_flat)
        saved.append(txt_path)

    print("[完成] 已保存:")
    for p in saved:
        size_kb = p.stat().st_size / 1024
        print(f"  {p}  ({size_kb:.1f} KB)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
