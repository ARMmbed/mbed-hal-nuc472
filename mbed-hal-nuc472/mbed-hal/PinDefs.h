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
#ifndef MBED_PINDEFS_H
#define MBED_PINDEFS_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NU_PORT_SHIFT  12
#define NU_PINNAME_TO_PORT(name)            ((unsigned int)(name) >> NU_PORT_SHIFT)
#define NU_PINNAME_TO_PIN(name)             ((unsigned int)(name) & ~(0xFFFFFFFF << NU_PORT_SHIFT))
#define NU_PORT_N_PIN_TO_PINNAME(port, pin) ((((unsigned int) (port)) << (NU_PORT_SHIFT)) | ((unsigned int) (pin)))
#define NU_PORT_BASE(port)                  ((GPIO_T *)(((uint32_t) GPIOA_BASE) + 0x40 * port))
#define NU_MFP_POS(pin)                     ((pin % 8) * 4)
#define NU_MFP_MSK(pin)                     (0xful << NU_MFP_POS(pin))

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum {
    PullNone = 0,
    PullDown,
    PullUp,
    
    PushPull,
    OpenDrain,
    Quasi,
    
    PullDefault = PullUp,
} PinMode;

#ifdef __cplusplus
}
#endif

#endif //MBED_PINDEFS_H
