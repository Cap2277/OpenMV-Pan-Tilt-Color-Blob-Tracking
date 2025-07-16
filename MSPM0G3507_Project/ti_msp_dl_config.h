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



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM */
#define PWM_INST                                                           TIMG7
#define PWM_INST_IRQHandler                                     TIMG7_IRQHandler
#define PWM_INST_INT_IRQN                                       (TIMG7_INT_IRQn)
#define PWM_INST_CLK_FREQ                                                  20000
/* GPIO defines for channel 0 */
#define GPIO_PWM_C0_PORT                                                   GPIOA
#define GPIO_PWM_C0_PIN                                           DL_GPIO_PIN_17
#define GPIO_PWM_C0_IOMUX                                        (IOMUX_PINCM39)
#define GPIO_PWM_C0_IOMUX_FUNC                       IOMUX_PINCM39_PF_TIMG7_CCP0
#define GPIO_PWM_C0_IDX                                      DL_TIMER_CC_0_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA1
#define PWM_1_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                                20000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOA
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_28
#define GPIO_PWM_1_C0_IOMUX                                       (IOMUX_PINCM3)
#define GPIO_PWM_1_C0_IOMUX_FUNC                      IOMUX_PINCM3_PF_TIMA1_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (31999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
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
#define UART_0_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_0_FBRD_4_MHZ_115200_BAUD                                       (11)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_1_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED1_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LED1_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for PIN_18: GPIOA.18 with pinCMx 40 on package pin 11 */
// pins affected by this interrupt request:["PIN_18"]
#define KEY_INT_IRQN                                            (GPIOA_INT_IRQn)
#define KEY_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define KEY_PIN_18_IIDX                                     (DL_GPIO_IIDX_DIO18)
#define KEY_PIN_18_PIN                                          (DL_GPIO_PIN_18)
#define KEY_PIN_18_IOMUX                                         (IOMUX_PINCM40)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for SDA: GPIOB.8 with pinCMx 25 on package pin 60 */
#define OLED_SDA_PIN                                             (DL_GPIO_PIN_8)
#define OLED_SDA_IOMUX                                           (IOMUX_PINCM25)
/* Defines for SCL: GPIOB.9 with pinCMx 26 on package pin 61 */
#define OLED_SCL_PIN                                             (DL_GPIO_PIN_9)
#define OLED_SCL_IOMUX                                           (IOMUX_PINCM26)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
