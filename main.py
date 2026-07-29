"""
激光打靶系统 — 单文件完整实现
==============================================================
硬件平台: OpenMV Cam H7 Plus
舵机云台: SG92R × 2 (Pan 水平 / Tilt 俯仰)
激光模块: 红色激光 (650nm), 固定在摄像头模组上
靶纸规格: 黑框边 + 浅红色同心圆环 (类标准 10 环靶)

工作原理:
  1. 摄像头搜索靶纸 (黑框 + 红环)
  2. 检测到靶心后, PD 控制器驱动云台将靶心移至画面中心
  3. 激光模块随摄像头同轴转动, 因此靶心对准画面中心 = 激光对准靶心
  4. 连续稳定锁定后自动击发激光
  5. 冷却后继续追踪, 可连发

接线说明:
  P7  → Pan 舵机信号线 (水平旋转)
  P8  → Tilt 舵机信号线 (俯仰)
  P9  → 激光模块控制 (高电平触发)
  GND → 舵机 GND + 激光 GND (共地)
  VIN → 舵机电源 (推荐独立 5V/2A 供电, 勿从 OpenMV 3.3V 取电!)

使用方法:
  1. OpenMV IDE 打开此文件
  2. 点击左下角绿色 ▶ 运行
  3. 帧缓冲区可看到实时叠加层
  4. 串行终端输出调试信息
  5. 将靶纸置于摄像头前方 1-3 米处
  6. 系统自动搜索→锁定→击发

可调参数:
  所有关键参数集中在顶部 CONFIG 区域, 标有 [TUNE] 的需根据实际环境调整
"""

import sensor
import image
import time
import math
from pyb import Pin, Timer, LED

# ===========================================================================
#                                CONFIG 配置区
# ===========================================================================
# 所有可调参数集中在此, 修改后重新运行即可生效
# ===========================================================================

class Config:
    """集中配置类 — 修改这里的参数适配不同环境和需求"""

    # ------------------------------------------------------------------
    # 摄像头参数
    # ------------------------------------------------------------------
    FRAME_WIDTH  = 320       # QVGA, 帧率和精度的平衡点
    FRAME_HEIGHT = 240
    PIXFORMAT    = sensor.RGB565
    AUTO_WHITEBAL = False    # 关闭自动白平衡, 颜色阈值才稳定
    AUTO_GAIN    = False     # 关闭自动增益, 避免亮度波动
    AUTO_EXPOSURE = False    # 关闭自动曝光
    EXPOSURE_US  = 12000     # 手动曝光 (微秒), [TUNE] 根据环境光调整
    GAIN_DB      = 6         # 手动增益 (dB)

    # ------------------------------------------------------------------
    # 靶面检测 — 颜色阈值 (LAB 色彩空间)
    # [TUNE] 用 OpenMV IDE 的 工具→机器视觉→阈值编辑器 选取最佳值
    # ------------------------------------------------------------------

    # 黑色边框: 低亮度 L, A/B 接近零 (中性黑)
    # L: 0-100, A: -128~+127, B: -128~+127
    BLACK_THRESHOLD = (0, 30,    # L: 极暗
                       -15, 15,  # A: 接近中性
                       -15, 15)  # B: 接近中性

    # 浅红色环: 中等亮度 + 正A(偏红) + 低B(略暖)
    # 浅红 = 不太暗的不饱和红, L 偏高, A 中等正
    RED_THRESHOLD = (35, 75,    # L: 不要最亮(排除白底) 不要最暗(排除黑框)
                     15, 55,    # A: 弱红~中等红
                     -5, 40)    # B: 偏中~略暖

    # 红色备选阈值 (不同光线下可能更准)
    RED_THRESHOLD_ALT = (30, 80,    # L: 更宽亮度范围
                         12, 60,    # A: 更宽的红色范围
                         -10, 45)   # B

    # ------------------------------------------------------------------
    # 靶面几何约束
    # ------------------------------------------------------------------
    MIN_BLACK_AREA    = 300    # 黑框最小面积 (像素^2)
    MAX_BLACK_AREA    = 20000  # 黑框最大面积
    MIN_RED_AREA      = 50     # 红环色块最小面积
    MIN_RED_BLOBS     = 2      # 最少检测到的红色块数量才能确认靶面
    BORDER_ROUNDNESS_MAX = 0.6  # 黑框圆度上限 (矩形不圆)

    # ------------------------------------------------------------------
    # 舵机参数 (SG92R: 500-2500us → 0-180°)
    # ------------------------------------------------------------------
    SERVO_FREQ     = 50       # PWM 频率, 舵机标准 50Hz
    SERVO_MIN_US   = 500      # 0° 脉宽
    SERVO_MAX_US   = 2500     # 180° 脉宽
    SERVO_CENTER_US = 1500    # 90° 中位脉宽

    # 角度限位 (防止舵机撞机械限位)
    PAN_MIN  = 5              # 水平最小角度 [TUNE]
    PAN_MAX  = 175            # 水平最大角度 [TUNE]
    TILT_MIN = 20             # 俯仰最小角度 (避免看到地面) [TUNE]
    TILT_MAX = 160            # 俯仰最大角度 [TUNE]

    # 归中角度
    PAN_HOME  = 90
    TILT_HOME = 90

    # 舵机平滑: 每帧最大角度变化 (°). 越小越平滑但响应越慢
    SERVO_SMOOTH_STEP = 2.0   # [TUNE]

    # ------------------------------------------------------------------
    # PD 追踪参数
    # [TUNE] KP 过大会震荡, 过小响应慢; KD 用于抑制震荡
    # ------------------------------------------------------------------
    PAN_KP  = 0.10            # 水平 P 增益 (°/像素)
    PAN_KD  = 0.06            # 水平 D 增益
    TILT_KP = 0.10            # 俯仰 P 增益
    TILT_KD = 0.06            # 俯仰 D 增益

    TRACK_DEAD_ZONE = 5       # 死区 (像素), 偏差 ≤ 此值认为已对准
    TRACK_MAX_STEP  = 4.0     # 每帧最大步进 (°), 安全保护

    # ------------------------------------------------------------------
    # 激光偏移校准
    # 激光模块与摄像头光轴有物理偏移, 需补偿
    # offset_x > 0: 激光在镜头右侧 → 需向左瞄准
    # offset_y > 0: 激光在镜头下方 → 需向上瞄准
    # [TUNE] 在 1.5m 距离打一枪, 量测偏差, 调整这两个值
    # ------------------------------------------------------------------
    LASER_OFFSET_X = 0        # 水平补偿 (像素), 正=激光偏右
    LASER_OFFSET_Y = 15       # 垂直补偿 (像素), 正=激光偏下

    # ------------------------------------------------------------------
    # 击发逻辑
    # ------------------------------------------------------------------
    LOCK_HOLD_TIME  = 0.6     # 稳定锁定多久后击发 (秒)
    FIRE_DURATION   = 0.3     # 激光点亮时长 (秒)
    COOLDOWN_TIME   = 2.0     # 击发后冷却时间 (秒)
    MAX_LOST_FRAMES = 15      # 丢失靶面多少帧后回到搜索

    # ------------------------------------------------------------------
    # 搜索模式
    # ------------------------------------------------------------------
    SEARCH_PAN_RANGE  = 60    # 搜索时水平扫描范围 (±°) [TUNE]
    SEARCH_TILT_RANGE = 30    # 搜索时俯仰扫描范围 (±°) [TUNE]
    SEARCH_STEP_DEG   = 5     # 搜索步长 (°)
    SEARCH_DELAY_MS   = 50    # 每步停顿 (ms)

    # ------------------------------------------------------------------
    # 调试选项
    # ------------------------------------------------------------------
    SHOW_OVERLAY    = True     # 在帧缓冲区绘制叠加层
    SHOW_FPS        = True     # 终端打印 FPS
    FPS_PRINT_EVERY = 30      # 每 N 帧打印一次状态


# 全局配置实例
cfg = Config()


# ===========================================================================
#                            硬件初始化
# ===========================================================================

def angle_to_us(angle):
    """角度 → 脉宽映射 (SG92R: 0°=500us, 180°=2500us)"""
    angle = max(0, min(180, angle))
    return int(cfg.SERVO_MIN_US + (cfg.SERVO_MAX_US - cfg.SERVO_MIN_US) * (angle / 180.0))


def init_camera():
    """初始化摄像头传感器"""
    sensor.reset()
    sensor.set_pixformat(cfg.PIXFORMAT)
    sensor.set_framesize(sensor.QVGA)          # 320 × 240
    sensor.set_auto_whitebal(cfg.AUTO_WHITEBAL)
    sensor.set_auto_gain(cfg.AUTO_GAIN)
    sensor.set_auto_exposure(cfg.AUTO_EXPOSURE)
    if not cfg.AUTO_EXPOSURE:
        sensor.set_auto_exposure(False, exposure_us=cfg.EXPOSURE_US)
    sensor.skip_frames(time=800)
    print(f"[CAM] OK  {cfg.FRAME_WIDTH}x{cfg.FRAME_HEIGHT} RGB565")


def init_servos():
    """初始化双轴舵机, 返回 (pan_timer, pan_ch, tilt_ch)"""
    # Timer 4: Ch1=P7(Pan), Ch2=P8(Tilt), 50Hz
    tim = Timer(4, freq=cfg.SERVO_FREQ)
    pan_ch  = tim.channel(1, Timer.PWM, pin=Pin('P7'))
    tilt_ch = tim.channel(2, Timer.PWM, pin=Pin('P8'))

    # 输出中位, 让舵机先稳定
    pan_ch.pulse_width(cfg.SERVO_CENTER_US)
    tilt_ch.pulse_width(cfg.SERVO_CENTER_US)
    time.sleep_ms(400)

    print(f"[SERVO] OK  P7=Pan P8=Tilt  {cfg.SERVO_FREQ}Hz "
          f"{cfg.SERVO_MIN_US}-{cfg.SERVO_MAX_US}us")
    return tim, pan_ch, tilt_ch


def init_laser():
    """初始化激光控制引脚"""
    laser = Pin('P9', Pin.OUT_PP)
    laser.low()
    print("[LASER] OK  P9 GPIO Output, initially OFF")
    return laser


# ===========================================================================
#                            靶面检测器
# ===========================================================================

class TargetDetector:
    """
    靶面检测: 黑框 + 浅红同心环 → 计算靶心坐标

    检测策略 (多级回退):
      Level 1 — 先找黑框, 框内找红环, 红环质心 → 靶心
      Level 2 — 直接找全局红色色块, 质心 → 靶心
      Level 3 — 扩大黑色阈值找暗色区域中心
    """

    def __init__(self):
        self.target_cx = None      # 靶心 X (像素)
        self.target_cy = None      # 靶心 Y (像素)
        self.target_radius = None  # 最外环半径 (像素)
        self.confidence = 0.0      # 置信度 0~1
        self.rings_found = 0       # 检测到的红环数
        self._border_rect = None   # 黑框矩形 (调试绘制用)
        self._red_blobs = []       # 红环色块列表 (调试绘制用)

        # 丢失保持: 短暂丢失时沿用上一帧结果
        self._last_valid = None
        self._lost_frames = 0
        self._max_hold_frames = 5

    def detect(self, img):
        """
        执行靶面检测。

        返回:
            dict: {
                'cx': int|None,      # 靶心 X
                'cy': int|None,      # 靶心 Y
                'radius': int|None,  # 最外层环半径
                'conf': float,       # 置信度 0~1
                'rings': int,        # 检测到的环数
                'found': bool,       # 是否检测到
            }
        """
        # ---- Level 1: 黑框 + 红环 ----
        result = self._detect_with_border(img)
        if result['found'] and result['conf'] >= 0.4:
            self._update_state(result)
            return result

        # ---- Level 2: 全局红色色块质心 ----
        result = self._detect_red_only(img)
        if result['found'] and result['conf'] >= 0.25:
            self._update_state(result)
            return result

        # ---- Level 3: 宽阈值暗色区域 ----
        result = self._detect_dark_region(img)
        if result['found']:
            self._update_state(result)
            return result

        # ---- 全部失败: 丢失保持 ----
        return self._handle_lost()

    # ------------------------------------------------------------------
    def _detect_with_border(self, img):
        """Level 1: 黑框内找红环"""
        # 找黑色矩形框
        black_blobs = img.find_blobs(
            [cfg.BLACK_THRESHOLD],
            pixels_threshold=cfg.MIN_BLACK_AREA,
            area_threshold=cfg.MIN_BLACK_AREA,
            merge=True, margin=15
        )

        best_border = self._pick_best_border(black_blobs)
        self._border_rect = best_border

        if best_border is None:
            return self._empty_result()

        # 在边框区域内找红色环
        border_cx = best_border.cx()
        border_cy = best_border.cy()
        # 搜索区域: 黑框中心 ± 框宽的一半
        search_radius = max(best_border.w(), best_border.h()) // 2 + 10
        roi = (
            max(0, border_cx - search_radius),
            max(0, border_cy - search_radius),
            min(cfg.FRAME_WIDTH  - max(0, border_cx - search_radius), search_radius * 2),
            min(cfg.FRAME_HEIGHT - max(0, border_cy - search_radius), search_radius * 2),
        )

        # 尝试两种红色阈值
        red_blobs = img.find_blobs(
            [cfg.RED_THRESHOLD],
            roi=roi,
            pixels_threshold=cfg.MIN_RED_AREA,
            area_threshold=cfg.MIN_RED_AREA,
            merge=True, margin=8
        )
        if not red_blobs or len(red_blobs) < cfg.MIN_RED_BLOBS:
            red_blobs = img.find_blobs(
                [cfg.RED_THRESHOLD_ALT],
                roi=roi,
                pixels_threshold=cfg.MIN_RED_AREA,
                area_threshold=cfg.MIN_RED_AREA,
                merge=True, margin=8
            )

        self._red_blobs = red_blobs if red_blobs else []

        if red_blobs and len(red_blobs) >= cfg.MIN_RED_BLOBS:
            # 用红环质心估计靶心
            cx, cy = self._compute_red_center(red_blobs)
            # 最外层红环半径作为参考 (blob 没有 .r(), 用包围盒估算)
            max_radius = max(max(b.w(), b.h()) // 2 for b in red_blobs)
            conf = min(1.0, 0.4 + len(red_blobs) * 0.12)
            return {'cx': cx, 'cy': cy, 'radius': max_radius,
                    'conf': conf, 'rings': len(red_blobs), 'found': True}

        # 有黑框但无红环 → 黑框中心即近似靶心
        radius = min(best_border.w(), best_border.h()) // 2
        return {'cx': border_cx, 'cy': border_cy, 'radius': radius,
                'conf': 0.35, 'rings': 0, 'found': True}

    # ------------------------------------------------------------------
    def _detect_red_only(self, img):
        """Level 2: 全局搜索红色色块"""
        for threshold in [cfg.RED_THRESHOLD, cfg.RED_THRESHOLD_ALT]:
            red_blobs = img.find_blobs(
                [threshold],
                pixels_threshold=cfg.MIN_RED_AREA * 2,
                area_threshold=cfg.MIN_RED_AREA * 2,
                merge=True, margin=12
            )
            if red_blobs and len(red_blobs) >= cfg.MIN_RED_BLOBS:
                self._red_blobs = red_blobs
                cx, cy = self._compute_red_center(red_blobs)
                max_radius = max(max(b.w(), b.h()) // 2 for b in red_blobs)
                conf = min(1.0, 0.25 + len(red_blobs) * 0.10)
                return {'cx': cx, 'cy': cy, 'radius': max_radius,
                        'conf': conf, 'rings': len(red_blobs), 'found': True}

        return self._empty_result()

    # ------------------------------------------------------------------
    def _detect_dark_region(self, img):
        """Level 3: 宽范围暗色区域"""
        # 用更宽松的黑色阈值
        wide_black = (0, 40, -25, 25, -25, 25)
        dark_blobs = img.find_blobs(
            [wide_black],
            pixels_threshold=cfg.MIN_BLACK_AREA * 3,
            area_threshold=cfg.MIN_BLACK_AREA * 3,
            merge=True, margin=20
        )
        if dark_blobs:
            best = max(dark_blobs, key=lambda b: b.area())
            radius = min(best.w(), best.h()) // 2
            return {'cx': best.cx(), 'cy': best.cy(), 'radius': radius,
                    'conf': 0.2, 'rings': 0, 'found': True}

        return self._empty_result()

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------
    def _pick_best_border(self, blobs):
        """从黑色色块中选出最像靶框的 (矩形、大小适中)"""
        if not blobs:
            return None

        best, best_score = None, 0.0
        for b in blobs:
            w, h = b.w(), b.h()

            # 大小过滤
            if not (cfg.MIN_BLACK_AREA <= b.area() <= cfg.MAX_BLACK_AREA):
                continue

            # 矩形度: 边框应有较低圆度
            rect_score = 1.0 - b.roundness()

            # 宽高比: 接近正方形
            aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0

            # 密度: 空心框密度适中 (实心块密度高)
            density = b.density()
            hollow_score = 1.0 - abs(density - 0.25)  # 理想边框密度约 0.25

            score = rect_score * 0.45 + aspect * 0.30 + hollow_score * 0.25

            if score > best_score:
                best_score = score
                best = b

        return best

    def _compute_red_center(self, red_blobs):
        """
        从红色色块列表计算靶心。
        策略: 按面积排序, 取面积最小 (最内环) 的几个色块的质心加权平均。
        小的红环更接近靶心, 圆度高的更可信。
        """
        # 按面积升序 (小的在前, 通常是内环)
        sorted_blobs = sorted(red_blobs, key=lambda b: b.area())

        # 取前 5 个最小的、圆度较好的
        candidates = []
        for b in sorted_blobs[:8]:
            if b.roundness() > 0.3:  # 至少有点圆
                weight = b.roundness() / max(b.area(), 1)
                candidates.append((b.cx(), b.cy(), weight))

        if candidates:
            total_wx = sum(c[0] * c[2] for c in candidates)
            total_wy = sum(c[1] * c[2] for c in candidates)
            total_w  = sum(c[2] for c in candidates)
            if total_w > 0:
                return (int(total_wx / total_w), int(total_wy / total_w))

        # 回退: 简单平均
        return (int(sum(b.cx() for b in sorted_blobs) / len(sorted_blobs)),
                int(sum(b.cy() for b in sorted_blobs) / len(sorted_blobs)))

    def _update_state(self, result):
        """更新内部状态"""
        self.target_cx    = result['cx']
        self.target_cy    = result['cy']
        self.target_radius = result['radius']
        self.confidence   = result['conf']
        self.rings_found  = result['rings']
        self._last_valid  = result.copy()
        self._lost_frames = 0

    def _handle_lost(self):
        """丢失处理: 短暂保持, 超时清空"""
        self._lost_frames += 1
        self._red_blobs = []

        if self._last_valid and self._lost_frames <= self._max_hold_frames:
            d = self._last_valid
            return {'cx': d['cx'], 'cy': d['cy'], 'radius': d['radius'],
                    'conf': max(0.0, d['conf'] - 0.15), 'rings': d['rings'],
                    'found': False}

        return self._empty_result()

    def _empty_result(self):
        return {'cx': None, 'cy': None, 'radius': None,
                'conf': 0.0, 'rings': 0, 'found': False}

    def is_valid(self):
        """当前靶面是否可信"""
        return self.confidence >= 0.2

    # ------------------------------------------------------------------
    # 调试绘制
    # ------------------------------------------------------------------
    def draw_overlay(self, img):
        """在图像上绘制检测叠加层"""
        # 黑框矩形
        if self._border_rect is not None:
            img.draw_rectangle(self._border_rect.rect(),
                               color=(255, 80, 80), thickness=2)

        # 红色色块
        for b in self._red_blobs:
            img.draw_rectangle(b.rect(), color=(255, 200, 0), thickness=1)

        # 靶心
        if self.target_cx is not None and self.target_cy is not None:
            c = (0, 255, 0) if self.confidence >= 0.4 else (0, 180, 255)
            img.draw_cross(self.target_cx, self.target_cy,
                           color=c, size=16, thickness=2)
            img.draw_circle(self.target_cx, self.target_cy, 6,
                            color=c, thickness=2)

        # 最外层环 (如果有)
        if self.target_radius and self.target_cx:
            img.draw_circle(self.target_cx, self.target_cy,
                            self.target_radius,
                            color=(0, 200, 0), thickness=2)

        # 状态文字
        s = f"T:{self.confidence:.2f} R:{self.rings_found}"
        img.draw_string(5, 5, s, color=(255, 255, 255),
                        mono_space=False, scale=1)


# ===========================================================================
#                            云台控制器
# ===========================================================================

class Gimbal:
    """
    双轴云台: Pan (水平) + Tilt (俯仰)
    PD 控制, 角度限位, 平滑过渡
    """

    def __init__(self, pan_ch, tilt_ch):
        self._pan_ch  = pan_ch
        self._tilt_ch = tilt_ch

        # 当前角度
        self.pan_angle  = float(cfg.PAN_HOME)
        self.tilt_angle = float(cfg.TILT_HOME)

        # PD 误差记忆
        self._last_err_x = 0.0
        self._last_err_y = 0.0
        self._last_time  = time.ticks_ms()

        # 写入初始脉宽
        self._write_pan()
        self._write_tilt()

    # ------------------------------------------------------------------
    # 底层 PWM 写入
    # ------------------------------------------------------------------
    def _write_pan(self):
        self._pan_ch.pulse_width(angle_to_us(self.pan_angle))

    def _write_tilt(self):
        self._tilt_ch.pulse_width(angle_to_us(self.tilt_angle))

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ------------------------------------------------------------------
    # 绝对角度设置
    # ------------------------------------------------------------------
    def set_pan(self, angle):
        self.pan_angle = self._clamp(angle, cfg.PAN_MIN, cfg.PAN_MAX)
        self._write_pan()

    def set_tilt(self, angle):
        self.tilt_angle = self._clamp(angle, cfg.TILT_MIN, cfg.TILT_MAX)
        self._write_tilt()

    def set_both(self, pan, tilt):
        self.pan_angle  = self._clamp(pan,  cfg.PAN_MIN,  cfg.PAN_MAX)
        self.tilt_angle = self._clamp(tilt, cfg.TILT_MIN, cfg.TILT_MAX)
        self._write_pan()
        self._write_tilt()

    def home(self):
        """归中"""
        self.set_both(cfg.PAN_HOME, cfg.TILT_HOME)

    # ------------------------------------------------------------------
    # PD 追踪
    # ------------------------------------------------------------------
    def track(self, error_x, error_y):
        """
        PD 控制追踪靶心。
        error_x: 靶心 X - 画面中心 X (正=靶在右, 云台需右转)
        error_y: 靶心 Y - 画面中心 Y (正=靶在下, 云台需下俯)

        注意: 图像 Y 轴向下, 舵机 tilt 角度增大通常 = 下俯
              需要根据实际安装方向确定正负号
        """
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time) / 1000.0
        dt = max(0.01, min(dt, 0.2))  # 限制 10ms~200ms
        self._last_time = now

        # --- 水平 (Pan) ---
        # 靶在右 (error_x > 0) → 云台右转 → pan 角度增大
        deriv_x = (error_x - self._last_err_x) / dt
        pan_correction = cfg.PAN_KP * error_x + cfg.PAN_KD * deriv_x

        # 加入激光偏移补偿
        pan_correction += cfg.LASER_OFFSET_X * 0.05

        # 限幅
        pan_correction = self._clamp(pan_correction,
                                     -cfg.TRACK_MAX_STEP, cfg.TRACK_MAX_STEP)

        self.pan_angle = self._clamp(
            self.pan_angle + pan_correction,
            cfg.PAN_MIN, cfg.PAN_MAX
        )

        # --- 俯仰 (Tilt) ---
        # 靶在下 (error_y > 0) → 云台下俯 → tilt 角度增大
        deriv_y = (error_y - self._last_err_y) / dt
        tilt_correction = cfg.TILT_KP * error_y + cfg.TILT_KD * deriv_y

        # 加入激光偏移补偿 (激光偏下 → 需向上 → tilt减小)
        tilt_correction -= cfg.LASER_OFFSET_Y * 0.05

        tilt_correction = self._clamp(tilt_correction,
                                      -cfg.TRACK_MAX_STEP, cfg.TRACK_MAX_STEP)

        self.tilt_angle = self._clamp(
            self.tilt_angle + tilt_correction,
            cfg.TILT_MIN, cfg.TILT_MAX
        )

        # 写入硬件
        self._write_pan()
        self._write_tilt()

        # 更新误差记忆
        self._last_err_x = error_x
        self._last_err_y = error_y

    # ------------------------------------------------------------------
    # 搜索扫描
    # ------------------------------------------------------------------
    def search_step(self, step_index):
        """
        生成搜索扫描位置 (Z 字形)。
        返回 (pan, tilt) 角度元组。
        """
        half_pan  = cfg.SEARCH_PAN_RANGE / 2.0
        half_tilt = cfg.SEARCH_TILT_RANGE / 2.0

        # 计算网格尺寸
        cols = max(1, int(cfg.SEARCH_PAN_RANGE / cfg.SEARCH_STEP_DEG))
        rows = max(1, int(cfg.SEARCH_TILT_RANGE / cfg.SEARCH_STEP_DEG))

        row = (step_index // cols) % rows
        col = step_index % cols

        # Z 字形: 偶数行左→右, 奇数行右→左
        if row % 2 == 0:
            pan_offset = -half_pan + (col / max(cols - 1, 1)) * cfg.SEARCH_PAN_RANGE
        else:
            pan_offset = half_pan - (col / max(cols - 1, 1)) * cfg.SEARCH_PAN_RANGE

        tilt_offset = -half_tilt + (row / max(rows - 1, 1)) * cfg.SEARCH_TILT_RANGE

        pan  = cfg.PAN_HOME  + pan_offset
        tilt = cfg.TILT_HOME + tilt_offset

        self.set_both(pan, tilt)
        return (pan, tilt)


# ===========================================================================
#                            激光控制器
# ===========================================================================

class Laser:
    """激光模块控制"""

    def __init__(self, pin):
        self._pin = pin
        self._is_on = False
        self.shot_count = 0

    def on(self):
        self._pin.high()
        self._is_on = True

    def off(self):
        self._pin.low()
        self._is_on = False

    def fire(self, duration=None):
        """击发: 点亮指定时长后自动关闭"""
        if duration is None:
            duration = cfg.FIRE_DURATION
        self.on()
        time.sleep_ms(int(duration * 1000))
        self.off()
        self.shot_count += 1
        print(f"\n  >>>  LASER FIRED!  第 {self.shot_count} 发 <<<\n")

    def is_on(self):
        return self._is_on


# ===========================================================================
#                            状态机
# ===========================================================================

class StateMachine:
    """
    系统状态机

    SEARCH   — 扫描搜索靶面
    CHECK    — 疑似目标, 转向验证 (真靶→TRACKING, 假靶→SEARCH)
    TRACKING — 确认真靶, PD 追踪靶心
    LOCKED   — 稳定锁定, 计时准备击发
    FIRING   — 击发激光
    COOLDOWN — 冷却中, 继续慢速追踪
    """

    SEARCH   = 0
    CHECK    = 1
    TRACKING = 2
    LOCKED   = 3
    FIRING   = 4
    COOLDOWN = 5

    NAMES = {0: 'SEARCH', 1: 'CHECK', 2: 'TRACK', 3: 'LOCKED',
             4: 'FIRE', 5: 'COOL'}

    def __init__(self):
        self.state = self.SEARCH
        self._state_start_ms = time.ticks_ms()
        self._search_step = 0

    @property
    def name(self):
        return self.NAMES[self.state]

    def elapsed_ms(self):
        """当前状态已持续 ms"""
        return time.ticks_diff(time.ticks_ms(), self._state_start_ms)

    def transition(self, new_state):
        """切换状态"""
        if new_state != self.state:
            print(f"  [STATE] {self.NAMES[self.state]} → {self.NAMES[new_state]}")
        self.state = new_state
        self._state_start_ms = time.ticks_ms()

    def update(self, target_found, on_target, target_conf=0.0, ring_count=0):
        """
        状态机更新 (新增 CHECK 验证阶段)。

        参数:
            target_found: 是否检测到疑似靶面
            on_target:   靶心是否在死区内 (已对准)
            target_conf: 靶面置信度 (0-1)
            ring_count:  检测到的环数
        """
        # CHECK 阶段的验证条件: 多个环 + 足够置信度 = 真靶
        is_verified = (target_found and
                       ring_count >= cfg.MIN_RED_BLOBS and
                       target_conf >= 0.4)

        if self.state == self.SEARCH:
            if target_found:
                self.transition(self.CHECK)
                return 'check'
            return 'search'

        elif self.state == self.CHECK:
            if is_verified:
                # 验证通过 → 进入追踪
                self.transition(self.TRACKING)
                return 'track'
            elif not target_found:
                # 目标消失 → 回搜索
                self.transition(self.SEARCH)
                return 'search'
            elif self.elapsed_ms() > 1500:
                # 1.5秒验证超时 → 假靶, 回搜索
                print("  [CHECK] 验证失败, 疑似假靶")
                self.transition(self.SEARCH)
                self._search_step = 0
                return 'search'
            else:
                # 继续验证 (转向目标观察)
                return 'check'

        elif self.state == self.TRACKING:
            if not target_found:
                if self.elapsed_ms() > cfg.MAX_LOST_FRAMES * 33:
                    self.transition(self.SEARCH)
                    return 'search'
                return 'track'
            if on_target:
                self.transition(self.LOCKED)
                return 'hold'
            return 'track'

        elif self.state == self.LOCKED:
            if not target_found:
                self.transition(self.TRACKING)
                return 'track'
            if not on_target:
                self.transition(self.TRACKING)
                return 'track'
            if self.elapsed_ms() >= cfg.LOCK_HOLD_TIME * 1000:
                self.transition(self.FIRING)
                return 'fire'
            return 'hold'

        elif self.state == self.FIRING:
            self.transition(self.COOLDOWN)
            self._search_step = 0
            return 'cooldown'

        elif self.state == self.COOLDOWN:
            if self.elapsed_ms() >= cfg.COOLDOWN_TIME * 1000:
                self.transition(self.TRACKING)
                return 'track'
            return 'cooldown'

        return 'search'

    def next_search_step(self):
        """获取下一个搜索步进索引"""
        step = self._search_step
        self._search_step += 1
        return step


# ===========================================================================
#                            画面叠加层 (HUD)
# ===========================================================================

def draw_hud(img, state_name, target, gimbal, laser_shots, fps, on_target):
    """绘制完整的画面叠加层"""
    w, h = cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT
    cx, cy = w // 2, h // 2

    # ---- 画面中心十字 ----
    img.draw_cross(cx, cy, color=(0, 255, 0), size=10, thickness=1)

    # ---- 死区圆圈 ----
    dead_color = (0, 200, 0) if on_target else (80, 80, 80)
    img.draw_circle(cx, cy, cfg.TRACK_DEAD_ZONE, color=dead_color, thickness=1)

    # ---- 到靶心连线 ----
    if target['found'] and target['cx'] is not None:
        tx, ty = target['cx'], target['cy']
        img.draw_line(cx, cy, tx, ty, color=(255, 255, 0), thickness=1)
        img.draw_circle(tx, ty, 4, color=(0, 255, 255), thickness=1,
                        fill=True)

    # ---- 激光偏移补偿指示 ----
    if cfg.LASER_OFFSET_X != 0 or cfg.LASER_OFFSET_Y != 0:
        aim_x = cx - cfg.LASER_OFFSET_X
        aim_y = cy - cfg.LASER_OFFSET_Y
        img.draw_cross(int(aim_x), int(aim_y),
                       color=(255, 200, 100), size=5, thickness=1)

    # ---- 状态栏 (顶部) ----
    state_colors = {
        'SEARCH': (255, 60, 60),
        'CHECK':  (255, 180, 0),
        'TRACK':  (0, 220, 0),
        'LOCKED': (0, 255, 0),
        'FIRE':   (255, 0, 0),
        'COOL':   (120, 120, 255),
    }
    sc = state_colors.get(state_name, (255, 255, 255))

    # 半透明背景条
    img.draw_rectangle(0, 0, w, 52, color=(0, 0, 0), fill=True)

    img.draw_string(5, 2,  f"State: {state_name}", color=sc,
                    mono_space=False, scale=1)
    img.draw_string(5, 16, f"Pan:{gimbal.pan_angle:5.1f}  Tilt:{gimbal.tilt_angle:5.1f}",
                    color=(200, 200, 200), mono_space=False, scale=1)

    conf_str = f"Conf:{target['conf']:.2f}" if target['conf'] > 0 else "Conf:---"
    img.draw_string(5, 30, f"{conf_str}  Rings:{target['rings']}  Shots:{laser_shots}",
                    color=(200, 200, 200), mono_space=False, scale=1)

    # FPS
    if fps > 0:
        img.draw_string(w - 45, 2, f"{fps:.0f}fps",
                        color=(180, 180, 180), mono_space=False, scale=1)

    # ---- 底部状态条 ----
    bar_y = h - 14
    img.draw_rectangle(0, bar_y, w, 14, color=(0, 0, 0), fill=True)

    bottom_msgs = {
        'SEARCH': 'Scanning for target...',
        'CHECK':  'Verifying target...',
        'LOCKED': 'TARGET LOCKED!',
        'FIRE':   '>>> FIRING! <<<',
        'COOL':   'Cooldown...',
    }
    msg = bottom_msgs.get(state_name, '')
    msg_color = state_colors.get(state_name, (255, 255, 255))
    img.draw_string(3, bar_y + 1, msg, color=msg_color,
                    mono_space=False, scale=1)


# ===========================================================================
#                            主程序
# ===========================================================================

class LaserTargetSystem:
    """激光打靶系统主控"""

    def __init__(self):
        # ---- 硬件初始化 ----
        init_camera()
        _, pan_ch, tilt_ch = init_servos()
        laser_pin = init_laser()

        # ---- 模块初始化 ----
        self.gimbal  = Gimbal(pan_ch, tilt_ch)
        self.detector = TargetDetector()
        self.laser    = Laser(laser_pin)
        self.fsm      = StateMachine()

        # ---- LED 指示 ----
        self.led_r = LED(1)  # 红
        self.led_g = LED(2)  # 绿
        self.led_b = LED(3)  # 蓝
        self._led_off_all()

        # ---- 统计 ----
        self.frame_count = 0
        self.clock = time.clock()

        print("\n" + "=" * 55)
        print("   激光打靶系统 — Laser Targeting System")
        print("   平台: OpenMV H7 Plus + SG92R Gimbal")
        print("   激光: 固定于摄像头, 同轴瞄准")
        print("=" * 55)
        print(f"   分辨率:   {cfg.FRAME_WIDTH}x{cfg.FRAME_HEIGHT} RGB565")
        print(f"   死区:     ±{cfg.TRACK_DEAD_ZONE} px")
        print(f"   PD:       Pan(Kp={cfg.PAN_KP},Kd={cfg.PAN_KD})")
        print(f"             Tilt(Kp={cfg.TILT_KP},Kd={cfg.TILT_KD})")
        print(f"   锁定时间: {cfg.LOCK_HOLD_TIME}s")
        print(f"   激光时长: {cfg.FIRE_DURATION}s")
        print(f"   冷却时间: {cfg.COOLDOWN_TIME}s")
        print("=" * 55 + "\n")

    # ------------------------------------------------------------------
    def _led_off_all(self):
        for led in [self.led_r, self.led_g, self.led_b]:
            led.off()

    def _update_led(self, state_name):
        """根据状态更新 RGB LED"""
        self._led_off_all()
        if state_name == 'SEARCH':
            self.led_b.on()
        elif state_name == 'CHECK':
            self.led_b.on(); self.led_r.on()   # 蓝+红 = 紫, 验证中
        elif state_name == 'TRACK':
            self.led_b.on(); self.led_g.on()   # 蓝+绿 = 青, 追踪中
        elif state_name == 'LOCKED':
            self.led_g.on()                     # 绿, 已锁定
        elif state_name == 'FIRE':
            self.led_r.on()                     # 红, 击发
        elif state_name == 'COOLDOWN':
            self.led_r.on(); self.led_b.on()   # 紫, 冷却

    # ------------------------------------------------------------------
    def run(self):
        """主循环"""
        while True:
            self.clock.tick()
            self.frame_count += 1

            # ---- 1. 图像采集 ----
            img = sensor.snapshot()

            # ---- 2. 靶面检测 ----
            result = self.detector.detect(img)
            target_found = result['found'] and result['cx'] is not None
            target_cx = result['cx']
            target_cy = result['cy']

            # ---- 3. 计算偏差 ----
            frame_cx = cfg.FRAME_WIDTH  // 2
            frame_cy = cfg.FRAME_HEIGHT // 2

            if target_found:
                error_x = target_cx - frame_cx
                error_y = target_cy - frame_cy
                on_target = (abs(error_x) <= cfg.TRACK_DEAD_ZONE and
                            abs(error_y) <= cfg.TRACK_DEAD_ZONE)
            else:
                error_x = 0.0
                error_y = 0.0
                on_target = False

            # ---- 4. 状态机更新 (传入环数和置信度用于验证) ----
            ring_count = result.get('rings', 0)
            conf = result.get('conf', 0.0)
            action = self.fsm.update(target_found, on_target, conf, ring_count)

            # ---- 5. 执行动作 ----
            if action == 'search':
                step = self.fsm.next_search_step()
                self.gimbal.search_step(step)
                time.sleep_ms(cfg.SEARCH_DELAY_MS)

            elif action == 'check':
                # 疑似目标: 转向它验证 (用全增益, 尽快对准观察)
                self.gimbal.track(error_x, error_y)

            elif action == 'track':
                self.gimbal.track(error_x, error_y)

            elif action == 'hold':
                # 锁定: 微调
                self.gimbal.track(error_x * 0.4, error_y * 0.4)

            elif action == 'fire':
                self.laser.fire()

            elif action == 'cooldown':
                if target_found and not on_target:
                    self.gimbal.track(error_x * 0.2, error_y * 0.2)

            # ---- 6. 绘制叠加层 ----
            if cfg.SHOW_OVERLAY:
                self.detector.draw_overlay(img)
                draw_hud(img, self.fsm.name, result,
                         self.gimbal, self.laser.shot_count,
                         self.clock.fps(), on_target)

            # ---- 7. LED 更新 ----
            self._update_led(self.fsm.name)

            # ---- 8. 串口调试输出 ----
            if cfg.SHOW_FPS and self.frame_count % cfg.FPS_PRINT_EVERY == 0:
                fps = self.clock.fps()
                if target_found:
                    print(f"[{self.fsm.name:6s}] FPS:{fps:4.0f}  "
                          f"Target:({target_cx:3d},{target_cy:3d})  "
                          f"Err:({error_x:+4.0f},{error_y:+4.0f})  "
                          f"Gimbal:({self.gimbal.pan_angle:5.1f},{self.gimbal.tilt_angle:5.1f})  "
                          f"Conf:{result['conf']:.2f}")
                else:
                    print(f"[{self.fsm.name:6s}] FPS:{fps:4.0f}  No target  "
                          f"Gimbal:({self.gimbal.pan_angle:5.1f},{self.gimbal.tilt_angle:5.1f})")

            # ---- 9. 帧间间隔 ----
            # 给舵机 PWM 更新的时间, 也让图像处理管道不堵塞
            time.sleep_ms(5)


# ===========================================================================
#                            入口点
# ===========================================================================

if __name__ == "__main__":
    try:
        system = LaserTargetSystem()
        system.run()
    except KeyboardInterrupt:
        print("\n[EXIT] 用户停止程序")
        # 安全: 关闭激光
        try:
            Pin('P9', Pin.OUT_PP).low()
        except:
            pass
        # 舵机归中
        try:
            tim = Timer(4, freq=50)
            tim.channel(1, Timer.PWM, pin=Pin('P7')).pulse_width(1500)
            tim.channel(2, Timer.PWM, pin=Pin('P8')).pulse_width(1500)
        except:
            pass
        print("[EXIT] 激光已关闭, 舵机已归中")

    except Exception as e:
        print(f"\n[ERROR] 程序异常: {e}")
        # 紧急安全措施
        try:
            Pin('P9', Pin.OUT_PP).low()
        except:
            pass
        import sys
        sys.print_exception(e)
        raise
