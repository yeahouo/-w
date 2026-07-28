/**
 * @file uart_vision.c
 * @brief OpenMV 串口协议解析 — 滑动窗口状态机 + XOR 校验
 *
 * 数据流:
 *   UART DMA/IDLE → UART_Vision_Feed(原始字节)
 *      → 内部状态机寻找 HDR1 HDR2 → 收满 11 字节
 *      → XOR 校验 + TAIL 校验 → 触发 VisionFrameCallback
 */
#include "uart_vision.h"
#include "port.h"
#include "log.h"
#include "errcode.h"
#include <string.h>

/* 统计 */
static uint32_t s_frame_cnt;
static uint32_t s_drop_cnt;
static uint32_t s_last_frame_ms;

/* 帧解析缓冲与状态 */
static uint8_t  s_buf[VISION_FRAME_LEN];
static uint8_t  s_buf_idx;
static bool     s_hdr_synced;   /* 已找到帧头 */

/* 上层回调(由 Port_VisionRegisterCb 设置) */
static VisionFrameCallback_t s_callback;

void UART_Vision_Init(void)
{
    s_frame_cnt    = 0;
    s_drop_cnt     = 0;
    s_last_frame_ms = 0;
    s_buf_idx      = 0;
    s_hdr_synced   = false;
    s_callback     = 0;
    LOG_I("VIS", "init ok, frame_len=%d", VISION_FRAME_LEN);
}

/* 由 port 层桥接注册 */
extern void Port_VisionRegisterCb(VisionFrameCallback_t cb);   /* 已在 port.h 声明 */

/* 内部接口:port 层会调用本函数把回调注入,但 port.h 没暴露 setter 实现,
   所以 Port_VisionRegisterCb 在 BSP 层实现时把 cb 存到模块外,
   再让本文件通过弱定义拿。简化做法:在此文件实现 Port_VisionRegisterCb。 */
void Port_VisionRegisterCb(VisionFrameCallback_t cb)
{
    s_callback = cb;
}

static uint8_t xor_checksum(const uint8_t *p, uint16_t n)
{
    uint8_t x = 0;
    for (uint16_t i = 0; i < n; ++i) x ^= p[i];
    return x;
}

static void emit_frame(void)
{
    /* 已收满 11 字节 → 校验 TAIL + XOR */
    if (s_buf[9] != VISION_TAIL) {
        s_drop_cnt++;
        Err_Report(ERR_VISION_CRC_FAIL);
        LOG_W("VIS", "drop(tail): got=0x%02X", s_buf[9]);
        return;
    }
    uint8_t expect_xor = xor_checksum(s_buf, VISION_FRAME_LEN - 1);
    if (expect_xor != s_buf[VISION_FRAME_LEN - 1]) {
        s_drop_cnt++;
        Err_Report(ERR_VISION_CRC_FAIL);
        LOG_W("VIS", "drop(xor): expect=0x%02X got=0x%02X cnt=%lu",
              expect_xor, s_buf[VISION_FRAME_LEN - 1],
              (unsigned long)s_drop_cnt);
        return;
    }
    /* OK */
    bool first_frame = (s_frame_cnt == 0);
    s_frame_cnt++;
    s_last_frame_ms = Port_NowMs();
    if (first_frame) {
        LOG_I("VIS", "first frame received, cnt=1");
    }
    if (s_callback) {
        s_callback(s_buf, VISION_FRAME_LEN);
    }
}

void UART_Vision_Feed(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t b = data[i];

        if (!s_hdr_synced) {
            /* 寻找帧头 HDR1=0xAA, HDR2=0x55 */
            if (s_buf_idx == 0) {
                if (b == VISION_HDR1) { s_buf[0] = b; s_buf_idx = 1; }
                /* 否则继续等 */
            } else if (s_buf_idx == 1) {
                if (b == VISION_HDR2) {
                    s_buf[1] = b;
                    s_buf_idx = 2;
                    s_hdr_synced = true;
                } else if (b == VISION_HDR1) {
                    /* 连续 0xAA,保持 idx=1 */
                    s_buf[0] = b;
                } else {
                    s_buf_idx = 0;   /* 假同步,重来 */
                }
            }
            continue;
        }

        /* 已对齐帧头,继续填 */
        s_buf[s_buf_idx++] = b;
        if (s_buf_idx >= VISION_FRAME_LEN) {
            emit_frame();
            /* 复位,准备下一帧 */
            s_buf_idx = 0;
            s_hdr_synced = false;
        }
    }
}

uint32_t UART_Vision_GetFrameCount(void)  { return s_frame_cnt; }
uint32_t UART_Vision_GetDropCount(void)   { return s_drop_cnt; }
uint32_t UART_Vision_GetLastFrameMs(void) { return s_last_frame_ms; }

bool UART_Vision_TestParseFrame(const uint8_t *frame, uint16_t len)
{
    if (len != VISION_FRAME_LEN) return false;
    if (frame[0] != VISION_HDR1) return false;
    if (frame[1] != VISION_HDR2) return false;
    if (frame[9] != VISION_TAIL) return false;
    uint8_t x = 0;
    for (uint16_t i = 0; i < VISION_FRAME_LEN - 1; ++i) x ^= frame[i];
    return x == frame[VISION_FRAME_LEN - 1];
}
