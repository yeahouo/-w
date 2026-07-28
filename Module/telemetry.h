/**
 * @file telemetry.h
 * @brief 实时遥测 — 把所有关键变量以 CSV 帧发上位机画波形
 *
 * 上位机推荐:
 *   - VOFA+(支持 CSV 数据流)
 *   - SerialPlot(支持 CSV)
 *   - Serial Studio(JSON/CSV)
 *   - 自写 Python(pyserial + matplotlib)
 *
 * 默认输出格式(每帧一行,\n 结束):
 *   ms,err,elem,state,last_err,vframes,vdrops,
 *   tlf,tlr,trf,trr,alf,alr,arf,arr,plf,plr,prf,prr
 *
 * 各字段含义:
 *   ms        时间戳(ms)
 *   err       循迹偏差(像素,-80~80,32767=丢线)
 *   elem      元素枚举(0~7)
 *   state     循迹状态(0=OK 1=LOST 2=BLIND)
 *   last_err  最近错误码(见 errcode.h)
 *   vframes   OpenMV 累计接收帧数
 *   vdrops    OpenMV 累计丢帧数
 *   tXX       四轮目标速度(mm/s,LF/LR/RF/RR)
 *   aXX       四轮实测速度(mm/s)
 *   pXX       四轮 PWM 输出(-1000~1000)
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TEL_FMT_CSV   = 0,   /* CSV 文本(默认,通用) */
    TEL_FMT_OFF   = 1,   /* 关闭遥测 */
} TelFmt_t;

void Telemetry_Init(void);

/**
 * @brief 100Hz 采样任务(由 Port_OnTick100Hz 调用)
 *   只更新快照,不发送
 */
void Telemetry_Capture(void);

/**
 * @brief 主循环调用 — 按设定频率输出 CSV 帧
 */
void Telemetry_Poll(void);

/* 运行时配置 */
void Telemetry_SetFmt(TelFmt_t f);
void Telemetry_SetRate(uint8_t hz);   /* 输出频率,默认 50Hz,范围 1~100 */

#endif /* TELEMETRY_H */
