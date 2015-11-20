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
    CLK_PowerDown();
}

void mbed_exit_sleep(sleep_t *obj)
{
    // TODO: TO BE CONTINUED
    
    (void)obj;
}

#endif
