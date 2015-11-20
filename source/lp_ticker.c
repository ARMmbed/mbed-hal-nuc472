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

#include "mbed-hal/lp_ticker_api.h"

#if DEVICE_LOWPOWERTIMER

#include "sleep_api.h"
#include "nu_modutil.h"
//#include "uvisor-lib/uvisor-lib.h"

#define TMR_CLK_FREQ        MINAR_PLATFORM_TIME_BASE

static void tmr1_vec(void);

static int lp_ticker_inited = 0;
static volatile uint32_t counter_tick = 0;
static volatile uint32_t wakeup_tick = (uint32_t) -1;

// NOTE: To wake the system from power down mode, timer clock source must be ether LXT or LIRC.
// NOTE: TIMER_1 for tick and wakeup
static const struct nu_modinit_s timer1_modinit = {TIMER_1, TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_LIRC, 0, TMR1_RST, TMR1_IRQn, tmr1_vec};

#define TMR_CMP_MIN         2
#define TMR_CMP_MAX         0xFFFFFFu

static void tmr1_vec(void)
{
    TIMER_ClearIntFlag((TIMER_T *) timer1_modinit.modname);
    TIMER_ClearWakeupFlag((TIMER_T *) timer1_modinit.modname);
}

void lp_ticker_init(void)
{
    if (lp_ticker_inited) {
        return;
    }
    counter_tick = 0;
    wakeup_tick = TMR_CMP_MAX;
    lp_ticker_inited = 1;
    
    // Reset module
    SYS_ResetModule(timer1_modinit.rsetidx);
    
    // Select IP clock source
    CLK_SetModuleClock(timer1_modinit.clkidx, timer1_modinit.clksrc, timer1_modinit.clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(timer1_modinit.clkidx);

    // Configure clock
    uint32_t clk_timer1 = TIMER_GetModuleClock((TIMER_T *) timer1_modinit.modname);
    uint32_t prescale_timer1 = clk_timer1 / TMR_CLK_FREQ - 1;
    MBED_ASSERT((prescale_timer1 != (uint32_t) -1) && prescale_timer1 <= 127);
    uint32_t cmp_timer1 = wakeup_tick;
    MBED_ASSERT(cmp_timer1 >= TMR_CMP_MIN && cmp_timer1 <= TMR_CMP_MAX);
    // Continuous mode
    ((TIMER_T *) timer1_modinit.modname)->CTL = TIMER_CONTINUOUS_MODE | prescale_timer1 | TIMER_CTL_CNTDATEN_Msk;
    ((TIMER_T *) timer1_modinit.modname)->CMP = cmp_timer1;
    
    // Set vector
    NVIC_SetVector(timer1_modinit.irq_n, (uint32_t) timer1_modinit.var);
    NVIC_EnableIRQ(timer1_modinit.irq_n);
    TIMER_EnableInt((TIMER_T *) timer1_modinit.modname);
    TIMER_EnableWakeup((TIMER_T *) timer1_modinit.modname);
    
    // Schedule wakeup to match semantics of lp_ticker_get_compare_match()
    lp_ticker_set_interrupt(lp_ticker_read(), wakeup_tick);
    
    // Start timer
    TIMER_Start((TIMER_T *) timer1_modinit.modname);
}

uint32_t lp_ticker_read()
{    
    if (! lp_ticker_inited) {
        lp_ticker_init();
    }
    
    counter_tick = TIMER_GetCounter((TIMER_T *) timer1_modinit.modname);
    return counter_tick;
}

uint32_t lp_ticker_get_overflows_counter(void)
{
    // FIXME: Not know what overflow means here
    return 0;
}

uint32_t lp_ticker_get_compare_match(void)
{
    return wakeup_tick;
}

void lp_ticker_set_interrupt(uint32_t now, uint32_t time)
{
    wakeup_tick = time;
    
    //uint32_t wakeup_len_us = (time > now) ? (time - now) : (uint32_t) ((uint64_t) time + 0xFFFFFFFFu - now);
    uint32_t cmp_timer1 = wakeup_tick;
    MBED_ASSERT(cmp_timer1 >= TMR_CMP_MIN && cmp_timer1 <= TMR_CMP_MAX);
    ((TIMER_T *) timer1_modinit.modname)->CMP = cmp_timer1;
}

void lp_ticker_sleep_until(uint32_t now, uint32_t time)
{
    lp_ticker_set_interrupt(now, time);
    sleep_t sleep_obj;
    mbed_enter_sleep(&sleep_obj);
    mbed_exit_sleep(&sleep_obj);
}

#endif
