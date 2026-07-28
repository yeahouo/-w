/**
 * @file line_sensor.h
 * @brief 5 路数字灰度传感器读取 — 多次采样 + 多数表决
 *
 * 接线 (灰度未装时引脚让给 TM1637 数码管, 见 LineSensor_Init 注释):
 *   L1=PB17  L2=PA12  M=PA22  R1=PA27  R2=PA9
 *   极性: LINE_ACTIVE_LEVEL=0 → 检测到黑线时 GPIO 读到 0 (常见接法)
 *
 * 输出 bitmask 约定:
 *   bit0=L1(最左) bit1=L2 bit2=M bit3=R1 bit4=R2(最右)
 *   1 = 检测到黑线
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

/**
 * @brief 配 5 路灰度引脚为数字输入 + 内部上拉
 * @note  当前灰度传感器未装 (PB17/PA12 复用给 TM1637 数码管),
 *        故 main.c 暂不调用本函数; 装上灰度后取消 main.c 里的条件编译即可
 */
void LineSensor_Init(void);

/**
 * @brief 读 5 路灰度, 多次采样 + 多数表决, 返回 bitmask
 * @return bit0=L1 .. bit4=R2, 1=黑线 (见文件头约定)
 * @note  每次调用连读 V6_SENSOR_SAMPLES 次 (间隔 NOP), 累计命中数
 *        ≥ V6_SENSOR_THRESHOLD 才算真命中 — 抑制硬件抖动/边缘模糊
 */
uint8_t LineSensor_Read(void);

#endif /* LINE_SENSOR_H */
