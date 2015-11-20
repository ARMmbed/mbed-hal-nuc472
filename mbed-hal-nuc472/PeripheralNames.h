/* mbed Microcontroller Library
 * Copyright (c) 2015-2016 Nuvoton
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STDIO_UART_TX  USBTX
#define STDIO_UART_RX  USBRX
#define STDIO_UART     UART_0

typedef enum {
    GPIO_A = (int) GPIOA_BASE,
    GPIO_B = (int) GPIOB_BASE,
    GPIO_C = (int) GPIOC_BASE,
    GPIO_D = (int) GPIOD_BASE,
    GPIO_E = (int) GPIOE_BASE,
    GPIO_F = (int) GPIOF_BASE,
    GPIO_G = (int) GPIOG_BASE,
    GPIO_H = (int) GPIOH_BASE,
    GPIO_I = (int) GPIOI_BASE
} GPIOName;

typedef enum {
    ADC_0 = (int)ADC_BASE,
    ADC_1 = (int)EADC_BASE
} ADCName;

typedef enum {
    UART_0 = (int)UART0_BASE,
    UART_1 = (int)UART1_BASE,
    UART_2 = (int)UART2_BASE,
    UART_3 = (int)UART3_BASE,
    UART_4 = (int)UART4_BASE,
    UART_5 = (int)UART5_BASE
} UARTName;

typedef enum {
    SPI_0 = (int)SPI0_BASE,
    SPI_1 = (int)SPI1_BASE,
    SPI_2 = (int)SPI2_BASE,
    SPI_3 = (int)SPI3_BASE
} SPIName;

typedef enum {
    I2C_0 = (int)I2C0_BASE,
    I2C_1 = (int)I2C1_BASE,
    I2C_2 = (int)I2C2_BASE,
    I2C_3 = (int)I2C3_BASE,
    I2C_4 = (int)I2C4_BASE
} I2CName;

typedef enum {
    PWM_0  = (int)PWM0_BASE,
    PWM_1  = (int)PWM1_BASE
} PWMName;

typedef enum {
    TIMER_0  = (int)TIMER0_BASE,
    TIMER_1  = (int)TIMER1_BASE,
    TIMER_2  = (int)TIMER2_BASE,
    TIMER_3  = (int)TIMER3_BASE
} TIMERName;

typedef enum {
    RTC_0 = (int)RTC_BASE
} RTCName;

typedef enum {
    DMA_0 = (int)PDMA_BASE
} DMAName;

#ifdef __cplusplus
}
#endif

#endif
