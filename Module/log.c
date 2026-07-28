/**
 * @file log.c
 * @brief 分级日志实现 — 环形 buffer + DMA 异步输出
 *
 * 数据流:
 *   LOG_I(...) ──> vsnprintf ──> 加前缀 ──> 临界区 memcpy 进环形 buffer
 *                                              │
 *                                              ↓
 *                          Log_Poll() ──> Port_DebugSend (DMA 后台)
 *
 * 设计要点:
 *   - 环形 buffer: 8KB,buffer 满时丢最旧字节
 *   - 单条日志最大 128 字节(超过截断)
 *   - 临界区极短(仅 memcpy + 索引更新,约几十个 cycle)
 */
#include "log.h"
#include "port.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define LOG_BUF_SIZE      (8192)
#define LOG_LINE_MAX      (160)

static uint8_t    s_buf[LOG_BUF_SIZE];
static volatile uint32_t s_head;   /* 写入位置(MOD BUF_SIZE) */
static volatile uint32_t s_tail;   /* 读取位置 */
static volatile uint32_t s_used;   /* 当前占用字节数 */

static LogLevel_t s_level = LOG_LEVEL_DEBUG;
static uint32_t   s_dropped;

/* 级别字符 */
static const char level_char[5] = {'D', 'I', 'W', 'E', '-'};

void Log_Init(void)
{
    s_head    = 0;
    s_tail    = 0;
    s_used    = 0;
    s_dropped = 0;
    s_level   = LOG_LEVEL_DEBUG;
}

void Log_SetLevel(LogLevel_t lvl)
{
    s_level = lvl;
}

LogLevel_t Log_GetLevel(void)
{
    return s_level;
}

uint32_t Log_GetDroppedCount(void) { return s_dropped; }
uint32_t Log_GetQueueUsed(void)    { return s_used; }

/* 把一段字节入队(已格式化好,带前缀) */
static void enqueue(const char *p, uint16_t n)
{
    uint32_t primask = Port_EnterCritical();

    if (n > LOG_BUF_SIZE) n = LOG_BUF_SIZE;   /* 防御 */

    /* 容量不足时丢弃最旧数据 */
    while (n > (LOG_BUF_SIZE - s_used)) {
        s_tail = (s_tail + 1) % LOG_BUF_SIZE;
        s_used--;
        s_dropped++;
    }

    for (uint16_t i = 0; i < n; ++i) {
        s_buf[s_head] = (uint8_t)p[i];
        s_head = (s_head + 1) % LOG_BUF_SIZE;
    }
    s_used += n;

    Port_ExitCritical(primask);
}

void Log_Write(LogLevel_t lvl, const char *tag, const char *fmt, ...)
{
    if (lvl < s_level) return;   /* 级别过滤 */
    if (lvl >= LOG_LEVEL_NONE) return;

    char line[LOG_LINE_MAX];
    int  len = 0;

    /* 前缀: [ms][L][TAG] */
    uint32_t ms = Port_NowMs();
    len += snprintf(line + len, LOG_LINE_MAX - len,
                    "[%lu][%c][%s] ",
                    (unsigned long)ms,
                    level_char[(int)lvl],
                    tag ? tag : "?");

    if (len < 0 || len >= LOG_LINE_MAX) return;

    /* 主体 */
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line + len, LOG_LINE_MAX - len, fmt, ap);
    va_end(ap);

    if (n < 0) { line[LOG_LINE_MAX - 1] = '\0'; n = LOG_LINE_MAX - len - 1; }
    len += n;
    if (len >= LOG_LINE_MAX) len = LOG_LINE_MAX - 1;

    /* 强制换行(便于上位机解析) */
    if (line[len - 1] != '\n') {
        if (len + 1 >= LOG_LINE_MAX) len = LOG_LINE_MAX - 2;
        line[len++] = '\r';
        line[len++] = '\n';
    }

    enqueue(line, (uint16_t)len);
}

void Log_Poll(void)
{
    /* 一次最多发 256 字节,避免主循环被 UART 阻塞 */
    static uint8_t tmp[256];

    if (s_used == 0) return;
    if (!Port_DebugTxReady()) return;

    uint16_t take = (s_used > sizeof(tmp)) ? (uint16_t)sizeof(tmp) : (uint16_t)s_used;

    /* 出队:从 tail 读 take 字节 */
    uint32_t primask = Port_EnterCritical();
    for (uint16_t i = 0; i < take; ++i) {
        tmp[i] = s_buf[s_tail];
        s_tail = (s_tail + 1) % LOG_BUF_SIZE;
    }
    s_used -= take;
    Port_ExitCritical(primask);

    Port_DebugSend(tmp, take);
}
