/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED1_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LED1_PIN_22_IOMUX                                        (IOMUX_PINCM50)


/* ============================================================
 *  软件 UART RX (PB11) — 接收 OpenMV TX @115200
 *    PB11 = PINCM29, 下降沿中断 + 内部上拉
 * ============================================================ */
#define SOFT_UART_RX_PORT                                                (GPIOB)
#define SOFT_UART_RX_PIN                                         (DL_GPIO_PIN_11)
#define SOFT_UART_RX_IOMUX                                        (IOMUX_PINCM29)


/* ============================================================
 *  电机方向 GPIO (TB6612 AIN1/AIN2/BIN1/BIN2)
 *    按立创天猛星 MSPM0G3507 官方标准
 *    PA12=BIN1  PA13=BIN2  PA14=AIN1  PA15=AIN2
 * ============================================================ */
#define MOTOR_DIR_PORT                                                   (GPIOA)
#define MOTOR_AIN1_PIN                                          (DL_GPIO_PIN_14)
#define MOTOR_AIN1_IOMUX                                         (IOMUX_PINCM36)
#define MOTOR_AIN2_PIN                                          (DL_GPIO_PIN_15)
#define MOTOR_AIN2_IOMUX                                         (IOMUX_PINCM37)
#define MOTOR_BIN1_PIN                                          (DL_GPIO_PIN_12)
#define MOTOR_BIN1_IOMUX                                         (IOMUX_PINCM34)
#define MOTOR_BIN2_PIN                                          (DL_GPIO_PIN_13)
#define MOTOR_BIN2_IOMUX                                         (IOMUX_PINCM35)
/* 方向引脚掩码,一次性写所有方向位 */
#define MOTOR_DIR_PINS  (MOTOR_AIN1_PIN | MOTOR_AIN2_PIN \
                        | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN)


/* ============================================================
 *  电机 PWM 引脚 (TB6612 PWMA/PWMB) — 当前阶段做普通 GPIO
 *    PA17 = PWMA   PA16 = PWMB
 *    复用为数字输出,拉高=100% 占空比,拉低=停转
 *    等需要调速时再换 SysConfig 生成 TimerA1 PWM
 * ============================================================ */
#define MOTOR_PWM_PORT                                                   (GPIOA)
#define GPIO_PWM_A_PIN                                        (DL_GPIO_PIN_17)
#define GPIO_PWM_A_IOMUX                                        (IOMUX_PINCM39)
#define GPIO_PWM_A_PORT                                             MOTOR_PWM_PORT
#define GPIO_PWM_B_PIN                                        (DL_GPIO_PIN_16)
#define GPIO_PWM_B_IOMUX                                        (IOMUX_PINCM38)
#define GPIO_PWM_B_PORT                                             MOTOR_PWM_PORT
#define MOTOR_PWM_PINS                                (GPIO_PWM_A_PIN | GPIO_PWM_B_PIN)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */， 
