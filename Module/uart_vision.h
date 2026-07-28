/**
 * @file uart_vision.h
 * @brief OpenMV 串口协议解析 — DMA+IDLE 接收,滑动窗口状态机
 *
 * 接收到完整且校验通过的帧后,调用 port 注册的 VisionFrameCallback。
 */
#ifndef UART_VISION_H
#define UART_VISION_H

#include <stdint.h>
#include <stdbool.h>

#define VISION_FRAME_LEN    (11)
#define VISION_HDR1         (0xAA)
#define VISION_HDR2         (0x55)
#define VISION_TAIL         (0x0D)

/* 初始化(注册 UART/DMA) */
void UART_Vision_Init(void);

/**
 * @brief 喂入一段原始字节(由 UART RX DMA 中断或 IDLE 中断调用)
 *   内部跑状态机,完整帧拼好后回调上层
 */
void UART_Vision_Feed(const uint8_t *data, uint16_t len);

/* 统计 */
uint32_t UART_Vision_GetFrameCount(void);
uint32_t UART_Vision_GetDropCount(void);
uint32_t UART_Vision_GetLastFrameMs(void);   /* 最近一帧的时戳 */

/* 单元测试辅助:直接给定一帧字节,返回是否接受 */
bool    UART_Vision_TestParseFrame(const uint8_t *frame, uint16_t len);

#endif /* UART_VISION_H */
