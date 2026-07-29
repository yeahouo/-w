/**
 * @file line_sensor.h
 * @brief 6 路数字灰度传感器读取 — 多次采样 + 多数表决
 *
 * 接线 (从左到右 S1..S6):
 *   S1=OUT7=PA12  S2=OUT6=PB17  S3=OUT5=PA22
 *   S4=OUT4=PB16  S5=OUT3=PA27  S6=OUT2=PA9
 *   极性: LINE_ACTIVE_LEVEL=0 → 检测到黑线时 GPIO 读到 0 (常见接法)
 *
 * 输出 bitmask 约定:
 *   bit0=S1(最左) bit1=S2 bit2=S3 bit3=S4 bit4=S5 bit5=S6(最右)
 *   1 = 检测到黑线
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

/**
 * @brief 配 6 路灰度引脚为数字输入 + 内部上拉
 */
void LineSensor_Init(void);

/**
 * @brief 读 6 路灰度, 多次采样 + 多数表决, 返回 bitmask
 * @return bit0=S1 .. bit5=S6, 1=黑线 (见文件头约定)
 * @note  每次调用连读 V6_SENSOR_SAMPLES 次 (间隔 NOP), 累计命中数
 *        ≥ V6_SENSOR_THRESHOLD 才算真命中 — 抑制硬件抖动/边缘模糊
 */
uint8_t LineSensor_Read(void);

#endif /* LINE_SENSOR_H */
