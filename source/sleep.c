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

#include "mbed-hal/sleep_api.h"
#include "mbed-hal/serial_api.h"

#if DEVICE_SLEEP

#include "cmsis.h"
#include "device.h"
#include "objects.h"

void mbed_enter_sleep(sleep_t *obj)
{
    // TODO: TO BE CONTINUED
    
    // Wait until UART FIFO empty to get a cleaner console out
    while (! UART_IS_TX_EMPTY(((UART_T *) STDIO_UART)));
    
    // NOTE: To pass mbed-drivers test, timer requires to be fine-grained, so its implementation needs HIRC rather than LIRC/LXT as its clock source.
    //       But as CLK_PowerDown()/CLK_Idle() is called, HIRC will be disabled and timer cannot keep counting and alarm. To overcome the dilemma, 
    //       just make CPU halt and compromise power saving.
    // NOTE: As CLK_PowerDown()/CLK_Idle() is called, HIRC will be disabled in normal mode, but not in ICE mode. This may cause confusion in development.
#if 0
    SYS_UnlockReg();
    CLK_PowerDown();
    SYS_LockReg();
#else
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
#endif
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

void mbed_exit_sleep(sleep_t *obj)
{
    // TODO: TO BE CONTINUED
    
    (void)obj;
}

#endif
