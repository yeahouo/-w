/**
 * @file uart_debug.h
 * @brief UART0 调试输出 — 阻塞式 printf 风格 (putc/print/print_u32)
 *
 * 硬件: UART0 (TX=PA10, RX=PA11, 460800), 由 SYSCFG_DL_init() 配置
 *       本驱动只管发送, 不需单独 Init
 *
 * 用途: 串口日志/调试输出 (10Hz 帧), 接 USB-TTL 看
 */
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>

/* 阻塞发送一个字符 (等 UART 空闲) */
void UART_Putc(char c);

/* 阻塞发送以 '\0' 结尾的字符串 */
void UART_Print(const char *s);

/* 阻塞发送无符号十进制整数 */
void UART_PrintU32(uint32_t v);

#endif /* UART_DEBUG_H */
