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

#include "PeripheralPins.h"

// =====
// Note: Commented lines are alternative possibilities which are not used per default.
//       If you change them, you will have also to modify the corresponding xxx_api.c file
//       for pwmout, analogin, analogout, ...
// =====

#if 0
//*** GPIO ***
const PinMap PinMap_GPIO[] = {
    // GPIO A MFPL
    {PA_0, GPIO_A, SYS_GPA_MFPL_PA0MFP_GPIO},
    {PA_1, GPIO_A, SYS_GPA_MFPL_PA1MFP_GPIO},
    {PA_2, GPIO_A, SYS_GPA_MFPL_PA2MFP_GPIO},
    {PA_3, GPIO_A, SYS_GPA_MFPL_PA3MFP_GPIO},
    {PA_4, GPIO_A, SYS_GPA_MFPL_PA4MFP_GPIO},
    {PA_5, GPIO_A, SYS_GPA_MFPL_PA5MFP_GPIO},
    {PA_6, GPIO_A, SYS_GPA_MFPL_PA6MFP_GPIO},
    {PA_7, GPIO_A, SYS_GPA_MFPL_PA7MFP_GPIO},
    // GPIO A MFPH
    {PA_8, GPIO_A, SYS_GPA_MFPH_PA8MFP_GPIO},
    {PA_9, GPIO_A, SYS_GPA_MFPH_PA9MFP_GPIO},
    {PA_10, GPIO_A, SYS_GPA_MFPH_PA10MFP_GPIO},
    {PA_11, GPIO_A, SYS_GPA_MFPH_PA11MFP_GPIO},
    {PA_12, GPIO_A, SYS_GPA_MFPH_PA12MFP_GPIO},
    {PA_13, GPIO_A, SYS_GPA_MFPH_PA13MFP_GPIO},
    {PA_14, GPIO_A, SYS_GPA_MFPH_PA14MFP_GPIO},
    {PA_15, GPIO_A, SYS_GPA_MFPH_PA15MFP_GPIO},
    
    // GPIO B MFPL
    {PB_0, GPIO_B, SYS_GPB_MFPL_PB0MFP_GPIO},
    {PB_1, GPIO_B, SYS_GPB_MFPL_PB1MFP_GPIO},
    {PB_2, GPIO_B, SYS_GPB_MFPL_PB2MFP_GPIO},
    {PB_3, GPIO_B, SYS_GPB_MFPL_PB3MFP_GPIO},
    {PB_4, GPIO_B, SYS_GPB_MFPL_PB4MFP_GPIO},
    {PB_5, GPIO_B, SYS_GPB_MFPL_PB5MFP_GPIO},
    {PB_6, GPIO_B, SYS_GPB_MFPL_PB6MFP_GPIO},
    {PB_7, GPIO_B, SYS_GPB_MFPL_PB7MFP_GPIO},
    // GPIO B MFPH
    {PB_8, GPIO_B, SYS_GPB_MFPH_PB8MFP_GPIO},
    {PB_9, GPIO_B, SYS_GPB_MFPH_PB9MFP_GPIO},
    {PB_10, GPIO_B, SYS_GPB_MFPH_PB10MFP_GPIO},
    {PB_11, GPIO_B, SYS_GPB_MFPH_PB11MFP_GPIO},
    {PB_12, GPIO_B, SYS_GPB_MFPH_PB12MFP_GPIO},
    {PB_13, GPIO_B, SYS_GPB_MFPH_PB13MFP_GPIO},
    {PB_14, GPIO_B, SYS_GPB_MFPH_PB14MFP_GPIO},
    {PB_15, GPIO_B, SYS_GPB_MFPH_PB15MFP_GPIO},
    
    // GPIO C MFPL
    {PC_0, GPIO_C, SYS_GPC_MFPL_PC0MFP_GPIO},
    {PC_1, GPIO_C, SYS_GPC_MFPL_PC1MFP_GPIO},
    {PC_2, GPIO_C, SYS_GPC_MFPL_PC2MFP_GPIO},
    {PC_3, GPIO_C, SYS_GPC_MFPL_PC3MFP_GPIO},
    {PC_4, GPIO_C, SYS_GPC_MFPL_PC4MFP_GPIO},
    {PC_5, GPIO_C, SYS_GPC_MFPL_PC5MFP_GPIO},
    {PC_6, GPIO_C, SYS_GPC_MFPL_PC6MFP_GPIO},
    {PC_7, GPIO_C, SYS_GPC_MFPL_PC7MFP_GPIO},
    // GPIO C MFPH
    {PC_8, GPIO_C, SYS_GPC_MFPH_PC8MFP_GPIO},
    {PC_9, GPIO_C, SYS_GPC_MFPH_PC9MFP_GPIO},
    {PC_10, GPIO_C, SYS_GPC_MFPH_PC10MFP_GPIO},
    {PC_11, GPIO_C, SYS_GPC_MFPH_PC11MFP_GPIO},
    {PC_12, GPIO_C, SYS_GPC_MFPH_PC12MFP_GPIO},
    {PC_13, GPIO_C, SYS_GPC_MFPH_PC13MFP_GPIO},
    {PC_14, GPIO_C, SYS_GPC_MFPH_PC14MFP_GPIO},
    {PC_15, GPIO_C, SYS_GPC_MFPH_PC15MFP_GPIO},
    
    // GPIO D MFPL
    {PD_0, GPIO_D, SYS_GPD_MFPL_PD0MFP_GPIO},
    {PD_1, GPIO_D, SYS_GPD_MFPL_PD1MFP_GPIO},
    {PD_2, GPIO_D, SYS_GPD_MFPL_PD2MFP_GPIO},
    {PD_3, GPIO_D, SYS_GPD_MFPL_PD3MFP_GPIO},
    {PD_4, GPIO_D, SYS_GPD_MFPL_PD4MFP_GPIO},
    {PD_5, GPIO_D, SYS_GPD_MFPL_PD5MFP_GPIO},
    {PD_6, GPIO_D, SYS_GPD_MFPL_PD6MFP_GPIO},
    {PD_7, GPIO_D, SYS_GPD_MFPL_PD7MFP_GPIO},
    // GPIO D MFPH
    {PD_8, GPIO_D, SYS_GPD_MFPH_PD8MFP_GPIO},
    {PD_9, GPIO_D, SYS_GPD_MFPH_PD9MFP_GPIO},
    {PD_10, GPIO_D, SYS_GPD_MFPH_PD10MFP_GPIO},
    {PD_11, GPIO_D, SYS_GPD_MFPH_PD11MFP_GPIO},
    {PD_12, GPIO_D, SYS_GPD_MFPH_PD12MFP_GPIO},
    {PD_13, GPIO_D, SYS_GPD_MFPH_PD13MFP_GPIO},
    {PD_14, GPIO_D, SYS_GPD_MFPH_PD14MFP_GPIO},
    {PD_15, GPIO_D, SYS_GPD_MFPH_PD15MFP_GPIO},
    
    // GPIO E MFPL
    {PE_0, GPIO_E, SYS_GPE_MFPL_PE0MFP_GPIO},
    {PE_1, GPIO_E, SYS_GPE_MFPL_PE1MFP_GPIO},
    {PE_2, GPIO_E, SYS_GPE_MFPL_PE2MFP_GPIO},
    {PE_3, GPIO_E, SYS_GPE_MFPL_PE3MFP_GPIO},
    {PE_4, GPIO_E, SYS_GPE_MFPL_PE4MFP_GPIO},
    {PE_5, GPIO_E, SYS_GPE_MFPL_PE5MFP_GPIO},
    {PE_6, GPIO_E, SYS_GPE_MFPL_PE6MFP_GPIO},
    {PE_7, GPIO_E, SYS_GPE_MFPL_PE7MFP_GPIO},
    // GPIO E MFPH
    {PE_8, GPIO_E, SYS_GPE_MFPH_PE8MFP_GPIO},
    {PE_9, GPIO_E, SYS_GPE_MFPH_PE9MFP_GPIO},
    {PE_10, GPIO_E, SYS_GPE_MFPH_PE10MFP_GPIO},
    {PE_11, GPIO_E, SYS_GPE_MFPH_PE11MFP_GPIO},
    {PE_12, GPIO_E, SYS_GPE_MFPH_PE12MFP_GPIO},
    {PE_13, GPIO_E, SYS_GPE_MFPH_PE13MFP_GPIO},
    {PE_14, GPIO_E, SYS_GPE_MFPH_PE14MFP_GPIO},
    {PE_15, GPIO_E, SYS_GPE_MFPH_PE15MFP_GPIO},
    
    // GPIO F MFPL
    {PF_0, GPIO_F, SYS_GPF_MFPL_PF0MFP_GPIO},
    {PF_1, GPIO_F, SYS_GPF_MFPL_PF1MFP_GPIO},
    {PF_2, GPIO_F, SYS_GPF_MFPL_PF2MFP_GPIO},
    {PF_3, GPIO_F, SYS_GPF_MFPL_PF3MFP_GPIO},
    {PF_4, GPIO_F, SYS_GPF_MFPL_PF4MFP_GPIO},
    {PF_5, GPIO_F, SYS_GPF_MFPL_PF5MFP_GPIO},
    {PF_6, GPIO_F, SYS_GPF_MFPL_PF6MFP_GPIO},
    {PF_7, GPIO_F, SYS_GPF_MFPL_PF7MFP_GPIO},
    // GPIO F MFPH
    {PF_8, GPIO_F, SYS_GPF_MFPH_PF8MFP_GPIO},
    {PF_9, GPIO_F, SYS_GPF_MFPH_PF9MFP_GPIO},
    {PF_10, GPIO_F, SYS_GPF_MFPH_PF10MFP_GPIO},
    {PF_11, GPIO_F, SYS_GPF_MFPH_PF11MFP_GPIO},
    {PF_12, GPIO_F, SYS_GPF_MFPH_PF12MFP_GPIO},
    {PF_13, GPIO_F, SYS_GPF_MFPH_PF13MFP_GPIO},
    {PF_14, GPIO_F, SYS_GPF_MFPH_PF14MFP_GPIO},
    {PF_15, GPIO_F, SYS_GPF_MFPH_PF15MFP_GPIO},
    
    // GPIO G MFPL
    {PG_0, GPIO_G, SYS_GPG_MFPL_PG0MFP_GPIO},
    {PG_1, GPIO_G, SYS_GPG_MFPL_PG1MFP_GPIO},
    {PG_2, GPIO_G, SYS_GPG_MFPL_PG2MFP_GPIO},
    {PG_3, GPIO_G, SYS_GPG_MFPL_PG3MFP_GPIO},
    {PG_4, GPIO_G, SYS_GPG_MFPL_PG4MFP_GPIO},
    {PG_5, GPIO_G, SYS_GPG_MFPL_PG5MFP_GPIO},
    {PG_6, GPIO_G, SYS_GPG_MFPL_PG6MFP_GPIO},
    {PG_7, GPIO_G, SYS_GPG_MFPL_PG7MFP_GPIO},
    // GPIO G MFPH
    {PG_8, GPIO_G, SYS_GPG_MFPH_PG8MFP_GPIO},
    {PG_9, GPIO_G, SYS_GPG_MFPH_PG9MFP_GPIO},
    {PG_10, GPIO_G, SYS_GPG_MFPH_PG10MFP_GPIO},
    {PG_11, GPIO_G, SYS_GPG_MFPH_PG11MFP_GPIO},
    {PG_12, GPIO_G, SYS_GPG_MFPH_PG12MFP_GPIO},
    {PG_13, GPIO_G, SYS_GPG_MFPH_PG13MFP_GPIO},
    {PG_14, GPIO_G, SYS_GPG_MFPH_PG14MFP_GPIO},
    {PG_15, GPIO_G, SYS_GPG_MFPH_PG15MFP_GPIO},
    
    // GPIO H MFPL
    {PH_0, GPIO_H, SYS_GPH_MFPL_PH0MFP_GPIO},
    {PH_1, GPIO_H, SYS_GPH_MFPL_PH1MFP_GPIO},
    {PH_2, GPIO_H, SYS_GPH_MFPL_PH2MFP_GPIO},
    {PH_3, GPIO_H, SYS_GPH_MFPL_PH3MFP_GPIO},
    {PH_4, GPIO_H, SYS_GPH_MFPL_PH4MFP_GPIO},
    {PH_5, GPIO_H, SYS_GPH_MFPL_PH5MFP_GPIO},
    {PH_6, GPIO_H, SYS_GPH_MFPL_PH6MFP_GPIO},
    {PH_7, GPIO_H, SYS_GPH_MFPL_PH7MFP_GPIO},
    // GPIO H MFPH
    {PH_8, GPIO_H, SYS_GPH_MFPH_PH8MFP_GPIO},
    {PH_9, GPIO_H, SYS_GPH_MFPH_PH9MFP_GPIO},
    {PH_10, GPIO_H, SYS_GPH_MFPH_PH10MFP_GPIO},
    {PH_11, GPIO_H, SYS_GPH_MFPH_PH11MFP_GPIO},
    {PH_12, GPIO_H, SYS_GPH_MFPH_PH12MFP_GPIO},
    {PH_13, GPIO_H, SYS_GPH_MFPH_PH13MFP_GPIO},
    {PH_14, GPIO_H, SYS_GPH_MFPH_PH14MFP_GPIO},
    {PH_15, GPIO_H, SYS_GPH_MFPH_PH15MFP_GPIO},
    
    // GPIO I MFPL
    {PI_0, GPIO_I, SYS_GPI_MFPL_PI0MFP_GPIO},
    {PI_1, GPIO_I, SYS_GPI_MFPL_PI1MFP_GPIO},
    {PI_2, GPIO_I, SYS_GPI_MFPL_PI2MFP_GPIO},
    {PI_3, GPIO_I, SYS_GPI_MFPL_PI3MFP_GPIO},
    {PI_4, GPIO_I, SYS_GPI_MFPL_PI4MFP_GPIO},
    {PI_5, GPIO_I, SYS_GPI_MFPL_PI5MFP_GPIO},
    {PI_6, GPIO_I, SYS_GPI_MFPL_PI6MFP_GPIO},
    {PI_7, GPIO_I, SYS_GPI_MFPL_PI7MFP_GPIO},
    // GPIO I MFPI
    {PI_8, GPIO_I, SYS_GPI_MFPH_PI8MFP_GPIO},
    {PI_9, GPIO_I, SYS_GPI_MFPH_PI9MFP_GPIO},
    {PI_10, GPIO_I, SYS_GPI_MFPH_PI10MFP_GPIO},
    {PI_11, GPIO_I, SYS_GPI_MFPH_PI11MFP_GPIO},
    {PI_12, GPIO_I, SYS_GPI_MFPH_PI12MFP_GPIO},
    {PI_13, GPIO_I, SYS_GPI_MFPH_PI13MFP_GPIO},
    {PI_14, GPIO_I, SYS_GPI_MFPH_PI14MFP_GPIO},
    {PI_15, GPIO_I, SYS_GPI_MFPH_PI15MFP_GPIO},
};
#endif

//*** ADC ***

const PinMap PinMap_ADC[] = {
    {PE_0, ADC_0_0, SYS_GPE_MFPL_PE0MFP_ADC0_0},  // ADC0_0
    {PE_1, ADC_0_1, SYS_GPE_MFPL_PE1MFP_ADC0_1},  // ADC0_1
    {PE_2, ADC_0_2, SYS_GPE_MFPL_PE2MFP_ADC0_2},  // ADC0_2
    {PE_3, ADC_0_3, SYS_GPE_MFPL_PE3MFP_ADC0_3},  // ADC0_3
    {PE_4, ADC_0_4, SYS_GPE_MFPL_PE4MFP_ADC0_4},  // ADC0_4
    {PE_5, ADC_0_5, SYS_GPE_MFPL_PE5MFP_ADC0_5},  // ADC0_5
    {PE_6, ADC_0_6, SYS_GPE_MFPL_PE6MFP_ADC0_6},  // ADC0_6
    {PE_7, ADC_0_7, SYS_GPE_MFPL_PE7MFP_ADC0_7},  // ADC0_7

    {PE_8, ADC_0_8, SYS_GPE_MFPH_PE8MFP_ADC1_0},  // ADC0_8/ADC1_0
    {PE_9, ADC_0_9, SYS_GPE_MFPH_PE9MFP_ADC1_1},  // ADC0_9/ADC1_1
    {PE_10, ADC_0_10, SYS_GPE_MFPH_PE10MFP_ADC1_2},  // ADC0_10/ADC1_2
    {PE_11, ADC_0_11, SYS_GPE_MFPH_PE11MFP_ADC1_3},  // ADC0_11/ADC1_3
    
    {NC,   NC,    0}
};

//*** I2C ***

const PinMap PinMap_I2C_SDA[] = {
    {PB_1,  I2C_4, SYS_GPB_MFPL_PB1MFP_I2C4_SDA},
    {PB_7,  I2C_2, SYS_GPB_MFPL_PB7MFP_I2C2_SDA},
    {PC_9, I2C_3, SYS_GPC_MFPH_PC9MFP_I2C3_SCL},
    {PD_3,  I2C_3, SYS_GPD_MFPL_PD3MFP_I2C3_SDA},
    {PD_9,  I2C_0, SYS_GPD_MFPH_PD9MFP_I2C0_SDA},
    {PD_12, I2C_4, SYS_GPD_MFPH_PD12MFP_I2C4_SDA},
    {PG_15, I2C_1, SYS_GPG_MFPH_PG15MFP_I2C1_SDA},
    {PH_1, I2C_1, SYS_GPH_MFPL_PH1MFP_I2C1_SDA},
    {PH_4, I2C_3, SYS_GPH_MFPL_PH4MFP_I2C3_SDA},
    {PI_8, I2C_2, SYS_GPI_MFPH_PI8MFP_I2C2_SDA},
    
    {NC,    NC,    0}
};

const PinMap PinMap_I2C_SCL[] = {
    {PA_15, I2C_0, SYS_GPA_MFPH_PA15MFP_I2C0_SCL},
    {PB_0, I2C_4, SYS_GPB_MFPL_PB0MFP_I2C4_SCL},
    {PB_6, I2C_2, SYS_GPB_MFPL_PB6MFP_I2C2_SCL},
    {PC_9, I2C_3, SYS_GPC_MFPH_PC9MFP_I2C3_SCL},
    {PD_2, I2C_3, SYS_GPD_MFPL_PD2MFP_I2C3_SCL},
    {PD_8, I2C_0, SYS_GPD_MFPH_PD8MFP_I2C0_SCL},
    {PD_10, I2C_4, SYS_GPD_MFPH_PD10MFP_I2C4_SCL},
    {PG_14, I2C_1, SYS_GPG_MFPH_PG14MFP_I2C1_SCL},
    {PH_0, I2C_1, SYS_GPH_MFPL_PH0MFP_I2C1_SCL},
    {PH_3, I2C_3, SYS_GPH_MFPL_PH3MFP_I2C3_SCL},
    {PI_7, I2C_2, SYS_GPI_MFPL_PI7MFP_I2C2_SCL},
    
    {NC,    NC,    0}
};

//*** PWM ***

const PinMap PinMap_PWM[] = {
    {PA_5, PWM_0_0, SYS_GPA_MFPL_PA5MFP_PWM0_CH0},
    {PA_6, PWM_0_1, SYS_GPA_MFPL_PA6MFP_PWM0_CH1},
    {PA_7, PWM_1_3, SYS_GPA_MFPL_PA7MFP_PWM1_CH3},
    //{PA_7, EPWM_0, SYS_GPA_MFPL_PA7MFP_EPWM0_CH5},
    //{PA_7, EPWM_0, SYS_GPA_MFPL_PA7MFP_EPWM0_CH5},
    {PA_8, PWM_1_2, SYS_GPA_MFPH_PA8MFP_PWM1_CH2},
    //{PA_8, EPWM_0, SYS_GPA_MFPH_PA8MFP_EPWM0_CH4},
    {PA_9, PWM_1_1, SYS_GPA_MFPH_PA9MFP_PWM1_CH1},
    //{PA_9, EPWM_0, SYS_GPA_MFPH_PA9MFP_EPWM0_CH3},
    {PA_10, PWM_1_0, SYS_GPA_MFPH_PA10MFP_PWM1_CH0},
    //{PA_10, EPWM_0, SYS_GPA_MFPH_PA10MFP_EPWM0_CH2},
    {PA_11, PWM_0_5, SYS_GPA_MFPH_PA11MFP_PWM0_CH5},
    //{PA_11, EPWM_0, SYS_GPA_MFPH_PA11MFP_EPWM0_CH1},
    {PA_12, PWM_0_4, SYS_GPA_MFPH_PA12MFP_PWM0_CH4},
    //{PA_12, EPWM_0, SYS_GPA_MFPH_PA12MFP_EPWM0_CH0},
    {PA_13, PWM_1_4, SYS_GPA_MFPH_PA13MFP_PWM1_CH4},
    {PA_14, PWM_1_5, SYS_GPA_MFPH_PA14MFP_PWM1_CH5},
    {PB_6, PWM_1_4, SYS_GPB_MFPL_PB6MFP_PWM1_CH4},
    //{PB_6, EPWM_1, SYS_GPB_MFPL_PB6MFP_EPWM1_CH0},
    {PB_7, PWM_1_5, SYS_GPB_MFPL_PB7MFP_PWM1_CH5},
    //{PB_7, EPWM_1, SYS_GPB_MFPL_PB7MFP_EPWM1_CH1},
    //{PB_8, EPWM_1, SYS_GPB_MFPH_PB8MFP_EPWM1_CH2},
    //{PB_9, EPWM_1, SYS_GPB_MFPH_PB9MFP_EPWM1_CH3},
    //{PB_10, EPWM_1, SYS_GPB_MFPH_PB10MFP_EPWM1_CH4},
    //{PB_11, EPWM_1, SYS_GPB_MFPH_PB11MFP_EPWM1_CH5},
    {PC_10, PWM_0_2, SYS_GPC_MFPH_PC10MFP_PWM0_CH2},
    {PC_11, PWM_0_3, SYS_GPC_MFPH_PC11MFP_PWM0_CH3},
    {PF_9, PWM_0_0, SYS_GPF_MFPH_PF9MFP_PWM0_CH0},
    {PF_10, PWM_0_1, SYS_GPF_MFPH_PF10MFP_PWM0_CH1},

    {NC,    NC,    0}
};

//*** SERIAL ***

const PinMap PinMap_UART_TX[] = {
    {PA_14, UART_0, SYS_GPA_MFPH_PA14MFP_UART0_TXD},
    {PB_3, UART_1, SYS_GPB_MFPL_PB3MFP_UART1_TXD},
    {PB_5, UART_4, SYS_GPB_MFPL_PB5MFP_UART4_TXD},
    {PB_10, UART_5, SYS_GPB_MFPH_PB10MFP_UART5_TXD},
    {PC_1, UART_4, SYS_GPC_MFPL_PC1MFP_UART4_TXD},
    {PC_11, UART_2, SYS_GPC_MFPH_PC11MFP_UART2_TXD},
    {PD_5, UART_3, SYS_GPD_MFPL_PD5MFP_UART3_TXD},
    {PD_15, UART_5, SYS_GPD_MFPH_PD15MFP_UART5_TXD},
    {PF_7, UART_2, SYS_GPF_MFPL_PF7MFP_UART2_TXD},
    {PF_13, UART_1, SYS_GPF_MFPH_PF13MFP_UART1_TXD},
    {PG_2, UART_0, SYS_GPG_MFPL_PG2MFP_UART0_TXD},
    {PH_1, UART_4, SYS_GPH_MFPL_PH1MFP_UART4_TXD},
    {PH_12, UART_3, SYS_GPH_MFPH_PH12MFP_UART3_TXD},
    
    {NC,    NC,     0}
};

const PinMap PinMap_UART_RX[] = {
    {PA_13, UART_0, SYS_GPA_MFPH_PA13MFP_UART0_RXD},
    {PB_2, UART_1, SYS_GPB_MFPL_PB2MFP_UART1_RXD},
    {PB_4, UART_4, SYS_GPB_MFPL_PB4MFP_UART4_RXD},
    {PB_11, UART_5, SYS_GPB_MFPH_PB11MFP_UART5_RXD},
    {PC_0, UART_4, SYS_GPC_MFPL_PC0MFP_UART4_RXD},
    {PC_10, UART_2, SYS_GPC_MFPH_PC10MFP_UART2_RXD},
    {PD_4, UART_3, SYS_GPD_MFPL_PD4MFP_UART3_RXD},
    {PF_0, UART_5, SYS_GPF_MFPL_PF0MFP_UART5_RXD},
    {PF_6, UART_2, SYS_GPF_MFPL_PF6MFP_UART2_RXD},
    {PF_14, UART_1, SYS_GPF_MFPH_PF14MFP_UART1_RXD},
    {PG_1, UART_0, SYS_GPG_MFPL_PG1MFP_UART0_RXD},
    {PH_0, UART_4, SYS_GPH_MFPL_PH0MFP_UART4_RXD},
    {PH_11, UART_3, SYS_GPH_MFPH_PH11MFP_UART3_RXD},
    
    {NC,    NC,     0}
};

//*** SPI ***

const PinMap PinMap_SPI_MOSI[] = {
    {PA_10, SPI_3, SYS_GPA_MFPH_PA10MFP_SPI3_MOSI0},
    {PA_12, SPI_3, SYS_GPA_MFPH_PA12MFP_SPI3_MOSI1},
    {PB_5, SPI_2, SYS_GPB_MFPL_PB5MFP_SPI2_MOSI0},
    {PB_13, SPI_2, SYS_GPB_MFPH_PB13MFP_SPI2_MOSI1},
    {PC_4, SPI_0, SYS_GPC_MFPL_PC4MFP_SPI0_MOSI0},
    {PC_7, SPI_0, SYS_GPC_MFPL_PC7MFP_SPI0_MOSI0},
    {PC_13, SPI_1, SYS_GPC_MFPH_PC13MFP_SPI1_MOSI1},
    {PC_15, SPI_1, SYS_GPC_MFPH_PC15MFP_SPI1_MOSI0},
    {PD_9, SPI_3, SYS_GPD_MFPH_PD9MFP_SPI3_MOSI1},
    {PE_3, SPI_0, SYS_GPE_MFPL_PE3MFP_SPI0_MOSI0},
    {PE_7, SPI_0, SYS_GPE_MFPL_PE7MFP_SPI0_MOSI0},
    {PE_11, SPI_0, SYS_GPE_MFPH_PE11MFP_SPI0_MOSI1},
    {PF_0, SPI_1, SYS_GPF_MFPL_PF0MFP_SPI1_MOSI0},
    {PF_1, SPI_2, SYS_GPF_MFPL_PF1MFP_SPI2_MOSI0},
    {PF_5, SPI_3, SYS_GPF_MFPL_PF5MFP_SPI3_MOSI0},
    {PG_8, SPI_2, SYS_GPG_MFPH_PG8MFP_SPI2_MOSI0},
    {PH_8, SPI_2, SYS_GPH_MFPH_PH8MFP_SPI2_MOSI0},
    {PH_10, SPI_2, SYS_GPH_MFPH_PH10MFP_SPI2_MOSI1},
    {PI_6, SPI_3, SYS_GPI_MFPL_PI6MFP_SPI3_MOSI0},
    {PI_8, SPI_3, SYS_GPI_MFPH_PI8MFP_SPI3_MOSI1},
    
    {NC,    NC,    0}
};

const PinMap PinMap_SPI_MISO[] = {
    {PA_2, SPI_3, SYS_GPA_MFPL_PA2MFP_SPI3_MISO0},
    {PA_9, SPI_3, SYS_GPA_MFPH_PA9MFP_SPI3_MISO0},
    {PA_11, SPI_3, SYS_GPA_MFPH_PA11MFP_SPI3_MISO1},
    {PB_4, SPI_2, SYS_GPB_MFPL_PB4MFP_SPI2_MISO0},
    {PB_12, SPI_2, SYS_GPB_MFPH_PB12MFP_SPI2_MISO1},
    {PC_3, SPI_0, SYS_GPC_MFPL_PC3MFP_SPI0_MISO1},
    {PC_6, SPI_0, SYS_GPC_MFPL_PC6MFP_SPI0_MISO0},
    {PC_14, SPI_1, SYS_GPC_MFPH_PC14MFP_SPI1_MISO1},
    {PD_0, SPI_1, SYS_GPD_MFPL_PD0MFP_SPI1_MISO0},
    {PD_8, SPI_3, SYS_GPD_MFPH_PD8MFP_SPI3_MISO1},
    {PD_15, SPI_1, SYS_GPD_MFPH_PD15MFP_SPI1_MISO0},
    {PE_2, SPI_0, SYS_GPE_MFPL_PE2MFP_SPI0_MISO0},
    {PE_6, SPI_0, SYS_GPE_MFPL_PE6MFP_SPI0_MISO0},
    {PE_10, SPI_0, SYS_GPE_MFPH_PE10MFP_SPI0_MISO1},
    {PF_4, SPI_3, SYS_GPF_MFPL_PF4MFP_SPI3_MISO0},
    {PG_7, SPI_2, SYS_GPG_MFPL_PG7MFP_SPI2_MISO0},
    {PH_7, SPI_2, SYS_GPH_MFPL_PH7MFP_SPI2_MISO0},
    {PH_9, SPI_2, SYS_GPH_MFPH_PH9MFP_SPI2_MISO1},
    {PI_5, SPI_3, SYS_GPI_MFPL_PI5MFP_SPI3_MISO0},
    {PI_7, SPI_3, SYS_GPI_MFPL_PI7MFP_SPI3_MISO1},
    {PI_12, SPI_2, SYS_GPI_MFPH_PI12MFP_SPI2_MISO1},
    
    {NC,    NC,    0}
};

const PinMap PinMap_SPI_SCLK[] = {
    {PA_4, SPI_3, SYS_GPA_MFPL_PA4MFP_SPI3_CLK},
    {PA_8, SPI_3, SYS_GPA_MFPH_PA8MFP_SPI3_CLK},
    {PB_3, SPI_2, SYS_GPB_MFPL_PB3MFP_SPI2_CLK},
    {PC_8, SPI_0, SYS_GPC_MFPH_PC8MFP_SPI0_CLK},
    {PD_1, SPI_1, SYS_GPD_MFPL_PD1MFP_SPI1_CLK},
    {PD_14, SPI_1, SYS_GPD_MFPH_PD14MFP_SPI1_CLK},
    {PE_5, SPI_0, SYS_GPE_MFPL_PE5MFP_SPI0_CLK},
    {PF_3, SPI_3, SYS_GPF_MFPL_PF3MFP_SPI3_CLK},
    {PG_9, SPI_2, SYS_GPG_MFPH_PG9MFP_SPI2_CLK},
    {PH_6, SPI_2, SYS_GPH_MFPL_PH6MFP_SPI2_CLK},
    {PI_4, SPI_3, SYS_GPI_MFPL_PI4MFP_SPI3_CLK},
    
    {NC,    NC,    0}
};

const PinMap PinMap_SPI_SSEL[] = {
    {PA_5, SPI_3, SYS_GPA_MFPL_PA5MFP_SPI3_SS0},
    {PA_7, SPI_3, SYS_GPA_MFPL_PA7MFP_SPI3_SS0},
    {PB_2, SPI_2, SYS_GPB_MFPL_PB2MFP_SPI2_SS0},
    {PC_2, SPI_0, SYS_GPC_MFPL_PC2MFP_SPI0_SS0},
    {PC_12, SPI_1, SYS_GPC_MFPH_PC12MFP_SPI1_SS0},
    {PD_13, SPI_1, SYS_GPD_MFPH_PD13MFP_SPI1_SS0},
    {PE_4, SPI_0, SYS_GPE_MFPL_PE4MFP_SPI0_SS0},
    {PF_2, SPI_3, SYS_GPF_MFPL_PF2MFP_SPI3_SS0},
    {PH_5, SPI_2, SYS_GPH_MFPL_PH5MFP_SPI2_SS0},
    {PI_3, SPI_3, SYS_GPI_MFPL_PI3MFP_SPI3_SS0},
    {PI_11, SPI_2, SYS_GPI_MFPH_PI11MFP_SPI2_SS0},
    
    {NC,    NC,    0}
};
