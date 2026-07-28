/**
 * @file tm1637.h
 * @brief TM1637 四位数码管驱动 — CLK/DIO 两线软件时序
 *
 * 当前接线 (灰度传感器未装, 复用其引脚):
 *   CLK = PB17 (原灰度 L1)   DIO = PA12 (原灰度 L2)
 *   CLK 在 GPIOB, DIO 在 GPIOA, 跨端口 — TM1637 协议不区分端口
 *
 * 用途: 调参阶段显示一个数字 (档位/计数/状态), 不占 UART
 */
#ifndef TM1637_H
#define TM1637_H

#include <stdint.h>

/* 配 CLK/DIO 为推挽输出 (空闲高) */
void TM1637_Init(void);

/**
 * @brief 4 位同显同一个数字 (自检用: 1111/2222/.../0000)
 * @param digit 0..9, 越界截到 9
 */
void TM1637_ShowSame(uint8_t digit);

#endif /* TM1637_H */
