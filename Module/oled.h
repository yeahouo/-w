/**
 * @file oled.h
 * @brief SSD1306 128x64 OLED 驱动 — 4 线软件 SPI (CS 接 GND)
 *
 * 接线 (主控 → OLED):
 *   SCL (D0) = PA28     时钟, 主控输出
 *   SDA (D1) = PA31     数据, 主控输出 (MSB first)
 *   RES      = PB14     复位, 主控输出 (低有效)
 *   DC       = PB15     数据/命令, 主控输出 (0=命令, 1=数据)
 *   CS       = GND      永久片选, 省一根线
 *
 * 时序: 软件 bit-bang (CLK 空闲低, 上升沿采样), 与 TM1637 驱动同风格
 *       不依赖 SysConfig 配 I2C/SPI 外设, 只用 GPIO 推挽输出
 *
 * 显存模型: SSD1306 页寻址, 8 页 x 128 列 = 1024 字节
 *           本驱动在 RAM 维护一份 gram 副本, 改完调 OLED_Refresh() 推到屏
 *
 * 兜底: 若实际是 SH1106 (1.3" 屏), 把 oled.c 顶部 OLED_COL_OFFSET 改 2
 *       (列地址 +2 偏移, 否则画面左侧 2 列空白)
 */
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* 屏幕 geometry */
#define OLED_WIDTH_PX    (128)
#define OLED_HEIGHT_PX   (64)
#define OLED_PAGES       (8)     /* 64 / 8 */

/**
 * @brief 复位 + 发 SSD1306 初始化命令序列 + 清屏
 * @note  必须在 SysTick 启动后调用 (内部用 g_system_ms 做复位延时)
 */
void OLED_Init(void);

/* 清显存 (全 0) 并刷新到屏 */
void OLED_Clear(void);

/* 把内部 gram 整片 push 到 OLED (约 8ms) */
void OLED_Refresh(void);

/**
 * @brief 设/清单个像素
 * @param x   列 0..127
 * @param y   行 0..63
 * @param on  1=点亮, 0=熄灭
 */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);

/**
 * @brief 画一个 6x8 ASCII 字符
 * @param x   字符列号 0..21 (每字符宽 6 像素)
 * @param y   页号    0..7  (每页高 8 像素)
 * @param ch  ASCII 0x20..0x7E, 越界按空格处理
 */
void OLED_DrawChar(uint8_t x, uint8_t y, char ch);

/**
 * @brief 画字符串 (6x8)
 * @param x,y 起始字符列/页
 * @param s   以 '\0' 结尾的 ASCII 串
 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *s);

#endif /* OLED_H */
