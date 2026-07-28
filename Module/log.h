/**
 * @file log.h
 * @brief 分级日志系统 — DEBUG/INFO/WARN/ERROR + 环形 buffer 异步输出
 *
 * 使用:
 *   1. 启动时 Log_Init()
 *   2. 任意位置 LOG_I("TAG", "msg=%d", val)
 *   3. 主循环调用 Log_Poll() 异步输出到 UART
 *
 * 特性:
 *   - 中断上下文安全(写入环形 buffer 用临界区保护)
 *   - 运行时级别可调(Log_SetLevel)
 *   - 模块标签 + 时间戳前缀
 *   - buffer 满时丢弃最旧数据(避免反向阻塞)
 *
 * 性能预算(M0+ 80MHz):
 *   - LOG_xxx 入队:< 5μs(格式化 + memcpy + 临界区)
 *   - Log_Poll 出队:DMA 后台发送,主循环几乎零开销
 */
#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_ERROR = 3,
    LOG_LEVEL_NONE  = 4,   /* 完全静默 */
} LogLevel_t;

void       Log_Init(void);
void       Log_SetLevel(LogLevel_t lvl);
LogLevel_t Log_GetLevel(void);

/**
 * @brief 写入一条日志(自动加时间戳 + 级别 + 标签前缀)
 * @note  中断上下文也可调用,但建议避免在中断里频繁 LOG
 */
void       Log_Write(LogLevel_t lvl, const char *tag, const char *fmt, ...);

/**
 * @brief 主循环调用,把环形 buffer 的数据异步发到 UART
 */
void       Log_Poll(void);

/* 统计(给遥测/调试用) */
uint32_t   Log_GetDroppedCount(void);   /* 因 buffer 满被丢弃的字节数 */
uint32_t   Log_GetQueueUsed(void);      /* 当前 buffer 占用字节 */

/* 便捷宏 — tag 必须是字符串字面量,如 LOG_I("FSM", "state=%d", s) */
#define LOG_D(tag, ...)  Log_Write(LOG_LEVEL_DEBUG, tag, ##__VA_ARGS__)
#define LOG_I(tag, ...)  Log_Write(LOG_LEVEL_INFO,  tag, ##__VA_ARGS__)
#define LOG_W(tag, ...)  Log_Write(LOG_LEVEL_WARN,  tag, ##__VA_ARGS__)
#define LOG_E(tag, ...)  Log_Write(LOG_LEVEL_ERROR, tag, ##__VA_ARGS__)

#endif /* LOG_H */
