/**
 * @file uart_debug.c
 * @brief UART0 调试输出实现 — 阻塞式
 *
 * UART0 由 SYSCFG_DL_init() 配置 (TX=PA10, 460800 8N1)
 */

#include "ti_msp_dl_config.h"
#include "uart_debug.h"

void UART_Putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST)) {}
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

void UART_Print(const char *s)
{
    while (*s) UART_Putc(*s++);
}

void UART_PrintU32(uint32_t v)
{
    char buf[12];
    int i = 11;
    buf[i--] = 0;
    if (v == 0) { UART_Putc('0'); return; }
    while (v && i >= 0) { buf[i--] = (char)('0' + (v % 10)); v /= 10; }
    UART_Print(&buf[i + 1]);
}
