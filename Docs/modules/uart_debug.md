# uart_debug UART0 调试输出 (`Module/uart_debug.c/h`)

串口调试日志（10Hz 帧），接 USB-TTL 在 PC 看日志/画波形。阻塞式 printf 风格。

## 硬件接口

| 信号 | 引脚 | 说明 |
|------|------|------|
| TX | PA10 | 主控 → USB-TTL 的 RX |
| RX | PA11 | (当前未用, 预留 OpenMV/下行) |
| 波特率 | 460800 | 8N1 |

UART0 由 `SYSCFG_DL_init()`（`ti_msp_dl_config`）配置。**本驱动不管初始化**（UART0 已由 SysConfig 配好）。

## API

```c
void UART_Putc(char c);            /* 阻塞发一个字符 */
void UART_Print(const char *s);    /* 阻塞发以 '\0' 结尾的字符串 */
void UART_PrintU32(uint32_t v);    /* 阻塞发十进制无符号整数 */
```

## 依赖

- `ti_msp_dl_config.h`（`UART_0_INST`, `DL_UART_isBusy`, `DL_UART_Main_transmitData`）
- `SYSCFG_DL_init()` 已配 UART0

## 调用约定

- **阻塞式**：等 UART 空闲才发（`while (DL_UART_isBusy(...))`）。
- ⚠️ **别在 SysTick 中断里频繁调**（会堵中断、影响 PWM 时序）。日志放主循环 10Hz。
- 不需要 `Init`（UART0 由 `SYSCFG_DL_init` 配）。
- 想要格式化输出（如 `printf("%d", x)`）→ 用 `UART_Print` + `UART_PrintU32` 拼接，或自行加 `sprintf`。

## 排错

| 现象 | 原因 |
|------|------|
| 完全没输出 | TX/RX 接反 / USB-TTL 驱动没装 / `SYSCFG_DL_init` 没跑 |
| 乱码 | 波特率不匹配（PC 串口工具切 460800）/ 时钟没配对（HFXT）|
| 丢字符 | 中断里调了（改到主循环）/ 波特率太高线太长 |

## 示例

```c
UART_Print("t=");
UART_PrintU32(g_system_ms);
UART_Print(" bits=");
UART_PrintU32(bits);
UART_Print("\r\n");
```
