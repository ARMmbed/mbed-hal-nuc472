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

#include "mbed-hal/us_ticker_api.h"
#include "nu_modutil.h"
#include "mbed-drivers/mbed_assert.h"

#define US_PER_TMR_FIRE     (1000 * 10)
#define TMR_FIRE_FREQ       (1000 * 1000 / US_PER_TMR_FIRE)
#define US_PER_TMR_CLK      1
#define TMR_CLK_FREQ        (1000 * 1000 / US_PER_TMR_CLK)

static void tmr0_vec(void);

static int us_ticker_inited = 0;
static volatile uint32_t counter_fire = 0;
static volatile uint32_t counter_fire_int = (uint32_t) -1;

// NOTE: PCLK is set up in mbed_sdk_init(), but invocation of it is after us_ticker_init() due to C++ global object. Replace with HIRC.
//static const struct nu_modinit_s timer0_modinit = {TIMER_0, TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK, 0, TMR0_RST, TMR0_IRQn, tmr0_vec};
static const struct nu_modinit_s timer0_modinit = {TIMER_0, TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0, TMR0_RST, TMR0_IRQn, tmr0_vec};

#define TMR_CMP_MIN         2
#define TMR_CMP_MAX         0xFFFFFFu

static void tmr0_vec(void)
{
    counter_fire ++;
    if (counter_fire_int != (uint32_t) -1) {
        counter_fire_int --;
        if (counter_fire_int == 0) {
            // NOTE: us_ticker_set_interrupt() may get called in us_ticker_irq_handler();
            counter_fire_int = (uint32_t) -1;
            us_ticker_irq_handler();
        }
    }
    
    TIMER_ClearIntFlag((TIMER_T *) timer0_modinit.modname);
}

void us_ticker_init(void)
{
    if (us_ticker_inited) {
        return;
    }
    
    counter_fire = 0;
    counter_fire_int = (uint32_t) -1;
    us_ticker_inited = 1;
    
    // Reset this module
    SYS_ResetModule(timer0_modinit.rsetidx);
    
    // Select IP clock source
    CLK_SetModuleClock(timer0_modinit.clkidx, timer0_modinit.clksrc, timer0_modinit.clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(timer0_modinit.clkidx);

    uint32_t clk_timer0 = TIMER_GetModuleClock((TIMER_T *) timer0_modinit.modname);
    uint32_t prescale_timer0 = clk_timer0 / TMR_CLK_FREQ - 1;
    MBED_ASSERT((prescale_timer0 != (uint32_t) -1) && prescale_timer0 <= 127);
    uint32_t cmp_timer0 = US_PER_TMR_FIRE / US_PER_TMR_CLK;
    MBED_ASSERT(cmp_timer0 >= TMR_CMP_MIN && cmp_timer0 <= TMR_CMP_MAX);

    ((TIMER_T *) timer0_modinit.modname)->CTL = TIMER_PERIODIC_MODE | prescale_timer0 | TIMER_CTL_CNTDATEN_Msk;
    ((TIMER_T *) timer0_modinit.modname)->CMP = cmp_timer0;

    NVIC_SetVector(timer0_modinit.irq_n, (uint32_t) timer0_modinit.var);
    // Enable timer interrupt
    TIMER_EnableInt((TIMER_T *) timer0_modinit.modname);
    NVIC_EnableIRQ(timer0_modinit.irq_n);
    
    // Start the timer
    TIMER_Start((TIMER_T *) timer0_modinit.modname);
}


uint32_t us_ticker_read()
{
    if (! us_ticker_inited) {
        us_ticker_init();
    }
    
    uint32_t ts = counter_fire * US_PER_TMR_FIRE + TIMER_GetCounter((TIMER_T *) timer0_modinit.modname);
    return ts;
}

void us_ticker_disable_interrupt(void)
{
    counter_fire_int = (uint32_t) -1;
}

void us_ticker_clear_interrupt(void)
{
    //TIMER_ClearIntFlag((TIMER_T *) timer0_modinit.modname);
}

void us_ticker_set_interrupt(timestamp_t timestamp)
{
    int delta = (int)(timestamp - us_ticker_read());
    if (delta <= 0) {
        // This event was in the past:
        us_ticker_irq_handler();
        return;
    }
    
    // Round up to ensure go-off time is after set time
    delta += US_PER_TMR_FIRE - 1;
    counter_fire_int = delta / US_PER_TMR_FIRE;
    MBED_ASSERT(counter_fire_int != 0);
}
