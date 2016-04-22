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

#ifndef MBED_TARGET_CONFIG_H
#define MBED_TARGET_CONFIG_H

// FIXME: Number of modules for a peripheral

#define MODULES_SIZE_ANALOGIN  1
#define MODULES_SIZE_ANALOGOUT 1
#define MODULES_SIZE_GPIO      1
#define MODULES_SIZE_SPI       4
#define MODULES_SIZE_I2C       5
#define MODULES_SIZE_PWMOUT    2
#define MODULES_SIZE_SERIAL    6

// Transaction queue size for each peripheral

#define TRANSACTION_QUEUE_SIZE_SPI   16

// Minar platform configuration

#define MINAR_PLATFORM_TIME_BASE 1000
#define MINAR_PLATFORM_MINIMUM_SLEEP 10

// Substitute vIRQ_xxx() for NVIC_xxx() to support uvisor.
#ifdef __cplusplus
extern "C" {
#endif
#include "uvisor-lib/override.h"
#ifdef __cplusplus
}
#endif

#endif
