/**
 * @file tm1637.c
 * @brief TM1637 四位数码管驱动实现 — 软件 I2C-like 时序 (CLK/DIO)
 *
 * 引脚: CLK=PB17=PINCM43, DIO=PA12=PINCM34 (复用灰度 L1/L2)
 */

#include "ti_msp_dl_config.h"
#include "tm1637.h"

/* ============================================================
 *  引脚定义
 * ============================================================ */
#define TM_CLK_PORT   GPIOB
#define TM_CLK_PIN    DL_GPIO_PIN_17   /* PB17, 原灰度 L1 */
#define TM_CLK_PINCM  IOMUX_PINCM43
#define TM_DIO_PORT   GPIOA
#define TM_DIO_PIN    DL_GPIO_PIN_12   /* PA12, 原灰度 L2 */
#define TM_DIO_PINCM  IOMUX_PINCM34

/* 段码 0-9 (a-g, 共阴; dp=bit7 不点亮) */
static const uint8_t tm_seg7[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

/* ============================================================
 *  软件时序
 * ============================================================ */
static void tm_delay(void)
{
    /* ~几 us @ 80MHz; TM1637 时钟 <250kHz, 边沿间隔需 ≥1us */
    for (volatile int i = 0; i < 40; ++i) { __NOP(); }
}

static void tm_start(void)
{
    /* DIO 在 CLK 高时下降沿 = START */
    DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
}

static void tm_stop(void)
{
    /* DIO 在 CLK 高时上升沿 = STOP */
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN); tm_delay();
}

static void tm_write(uint8_t b)
{
    /* LSB first, 8 位数据 + 第 9 个 ACK 时钟(不读) */
    for (int i = 0; i < 8; ++i) {
        DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
        if (b & 0x01) DL_GPIO_setPins  (TM_DIO_PORT, TM_DIO_PIN);
        else          DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN);
        b >>= 1;
        tm_delay();
        DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();  /* 上升沿采样 */
    }
    /* ACK 周期: DIO 设低(让 TM1637 应答, 避免推挽冲突) */
    DL_GPIO_clearPins(TM_CLK_PORT, TM_CLK_PIN); tm_delay();
    DL_GPIO_clearPins(TM_DIO_PORT, TM_DIO_PIN); tm_delay();
    DL_GPIO_setPins  (TM_CLK_PORT, TM_CLK_PIN); tm_delay();
}

/* ============================================================
 *  公开 API
 * ============================================================ */
void TM1637_Init(void)
{
    /* CLK/DIO 推挽输出, 空闲高 */
    DL_GPIO_initDigitalOutput(TM_CLK_PINCM);
    DL_GPIO_initDigitalOutput(TM_DIO_PINCM);
    DL_GPIO_enableOutput(TM_CLK_PORT, TM_CLK_PIN);
    DL_GPIO_enableOutput(TM_DIO_PORT, TM_DIO_PIN);
    DL_GPIO_setPins(TM_CLK_PORT, TM_CLK_PIN);
    DL_GPIO_setPins(TM_DIO_PORT, TM_DIO_PIN);
}

void TM1637_ShowSame(uint8_t digit)
{
    if (digit > 9) digit = 9;
    uint8_t seg = tm_seg7[digit];
    tm_start();
    tm_write(0x40);       /* 数据命令: 写显示, 自动地址+1 */
    tm_stop();
    tm_start();
    tm_write(0xC0);       /* 起始地址: 第 0 位 */
    tm_write(seg);        /* 位 0 */
    tm_write(seg);        /* 位 1 */
    tm_write(seg);        /* 位 2 */
    tm_write(seg);        /* 位 3 */
    tm_stop();
    tm_start();
    tm_write(0x88 | 7);   /* 开显示, 亮度 7 (最亮) */
    tm_stop();
}
