/**
 * @file port.h
 * @brief 移植接口 — 让算法层不直接依赖硬件,可在 PC 上做单元测试
 *
 * 规则:
 *   - 算法层(Algorithm/)与状态机(Application/)只调 port.h 提供的接口
 *   - 单片机环境: 由 Module/ 与 BSP/ 实现这些函数
 *   - PC 单测环境: 由 Tests/ 提供桩实现
 */
#ifndef PORT_H
#define PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ---------- 平台生命周期 ---------- */
void     Port_HalInit(void);    /* 时钟/外设/中断初始化(BSP SysConfig 已配置好) */
void     Port_Idle(void);       /* 主循环空闲钩子(可放喂狗/WFI) */

/* ---------- 电机控制 ---------- */
void     Port_MotorSetPWM(MotorChannel_t ch, int16_t pwm);     /* pwm: -1000~1000 */
int16_t  Port_MotorGetPWM(MotorChannel_t ch);

/* ---------- 编码器读数 ---------- */
int32_t  Port_EncoderRead(MotorChannel_t ch);  /* 返回自上次调用以来的累计计数,带方向 */
void     Port_EncoderResetAll(void);

/* ---------- 串口(视觉 + 调试) ---------- */
/* OpenMV 帧到达:数据指针 + 长度,已校验通过 */
typedef void (*VisionFrameCallback_t)(const uint8_t *frame, uint16_t len);
void     Port_VisionRegisterCb(VisionFrameCallback_t cb);
uint32_t Port_NowMs(void);   /* 系统毫秒时钟(用于超时) */

/* 调试输出(printf 风格,可能重定向到 UART 或虚拟串口) */
void     Port_DebugLog(const char *fmt, ...);

/* 非阻塞调试串口(供 log.c 环形 buffer 异步输出) */
bool     Port_DebugTxReady(void);                          /* UART DMA 是否空闲 */
void     Port_DebugSend(const uint8_t *data, uint16_t n);  /* 非阻塞 DMA 发送 */

/* 临界区(保护环形 buffer 多上下文访问,在 1kHz 中断里短暂使用) */
uint32_t Port_EnterCritical(void);                         /* 返回上一个 PRIMASK 状态 */
void     Port_ExitCritical(uint32_t prev);

/* 板载 LED(状态指示) */
void     Port_LED_Set(bool on);
bool     Port_LED_Get(void);

/* 系统计时(自由运行的 32-bit 计数器) */
uint32_t Port_NowUs(void);                                 /* 微秒(可选实现) */

/* 启动按键(USER 按键,低电平有效) */
bool     Port_StartTrigger(void);

/* 编码器 EXTI ISR 桥接(由 BSP GPIO 中断 handler 调用) */
void     Port_EncoderISR(MotorChannel_t ch, int8_t dir);

/* ---------- 时基回调(由 Timer/SysTick 中断调用) ---------- */
void     Port_OnTick1kHz(void);   /* 电机 PI 闭环驱动入口 */
void     Port_OnTick100Hz(void);  /* 编码器 / 状态机入口 */

#endif /* PORT_H */
