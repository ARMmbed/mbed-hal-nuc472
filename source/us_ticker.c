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
#include "mbed-drivers/mbed_assert.h"
#include "nu_modutil.h"
#include "nu_miscutil.h"

// NOTE: mbed-drivers-test-timeout test requires 100 us timer granularity.
#define US_PER_TMR_INT      (1000 * 10)
#define TMR_FIRE_FREQ       (1000 * 1000 / US_PER_TMR_INT)
#define US_PER_TMR_CLK      1
#define TMR_CLK_FREQ        (1000 * 1000 / US_PER_TMR_CLK)

static void tmr0_vec(void);
static void tmr1_vec(void);
static void us_ticker_arm_cd(void);

static int us_ticker_inited = 0;
static volatile uint32_t counter_major = 0;
static volatile int cd_major_minor = 0;
static volatile int cd_minor = 0;

// NOTE: PCLK is set up in mbed_hal_init(), but invocation of it is after us_ticker_init() due to C++ global object.
//       CLK_CLKSEL1_TMR0SEL_PCLK --> CLK_CLKSEL1_TMR0SEL_HIRC
// NOTE: TIMER_0 for normal counter, TIMER_1 for countdown.
static const struct nu_modinit_s timer0_modinit = {TIMER_0, TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0, TMR0_RST, TMR0_IRQn, tmr0_vec};
static const struct nu_modinit_s timer1_modinit = {TIMER_1, TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0, TMR1_RST, TMR1_IRQn, tmr1_vec};

#define TMR_CMP_MIN         2
#define TMR_CMP_MAX         0xFFFFFFu

void us_ticker_init(void)
{
    if (us_ticker_inited) {
        return;
    }
    
    counter_major = 0;
    cd_major_minor = 0;
    cd_minor = 0;
    us_ticker_inited = 1;
    
    // Reset IP
    SYS_ResetModule(timer0_modinit.rsetidx);
    SYS_ResetModule(timer1_modinit.rsetidx);
    
    // Select IP clock source
    CLK_SetModuleClock(timer0_modinit.clkidx, timer0_modinit.clksrc, timer0_modinit.clkdiv);
    CLK_SetModuleClock(timer1_modinit.clkidx, timer1_modinit.clksrc, timer1_modinit.clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(timer0_modinit.clkidx);
    CLK_EnableModuleClock(timer1_modinit.clkidx);

    // Timer for normal counter
    uint32_t clk_timer0 = TIMER_GetModuleClock((TIMER_T *) NU_MODBASE(timer0_modinit.modname));
    uint32_t prescale_timer0 = clk_timer0 / TMR_CLK_FREQ - 1;
    MBED_ASSERT((prescale_timer0 != (uint32_t) -1) && prescale_timer0 <= 127);
    uint32_t cmp_timer0 = US_PER_TMR_INT / US_PER_TMR_CLK;
    MBED_ASSERT(cmp_timer0 >= TMR_CMP_MIN && cmp_timer0 <= TMR_CMP_MAX);
    ((TIMER_T *) NU_MODBASE(timer0_modinit.modname))->CTL = TIMER_PERIODIC_MODE | prescale_timer0 | TIMER_CTL_CNTDATEN_Msk;
    ((TIMER_T *) NU_MODBASE(timer0_modinit.modname))->CMP = cmp_timer0;
    
    NVIC_SetVector(timer0_modinit.irq_n, (uint32_t) timer0_modinit.var);
    NVIC_SetVector(timer1_modinit.irq_n, (uint32_t) timer1_modinit.var);
    
    NVIC_EnableIRQ(timer0_modinit.irq_n);
    NVIC_EnableIRQ(timer1_modinit.irq_n);
    
    TIMER_EnableInt((TIMER_T *) NU_MODBASE(timer0_modinit.modname));
    TIMER_Start((TIMER_T *) NU_MODBASE(timer0_modinit.modname));
}

uint32_t us_ticker_read()
{
    if (! us_ticker_inited) {
        us_ticker_init();
    }
    
    TIMER_T * timer0_base = (TIMER_T *) NU_MODBASE(timer0_modinit.modname);
    
    // NOTE: As TIMER_CNT = TIMER_CMP and counter_major has increased by one, TIMER_CNT may not reset to 0 immediately. Add check code to avoid instant error.
    while (timer0_base->CMP == TIMER_GetCounter(timer0_base));
    
    uint32_t ts = counter_major * US_PER_TMR_INT + TIMER_GetCounter(timer0_base);
    return ts;
}

void us_ticker_disable_interrupt(void)
{
    TIMER_DisableInt((TIMER_T *) NU_MODBASE(timer1_modinit.modname));
}

void us_ticker_clear_interrupt(void)
{
    TIMER_ClearIntFlag((TIMER_T *) NU_MODBASE(timer1_modinit.modname));
}

void us_ticker_set_interrupt(timestamp_t timestamp)
{
    TIMER_Stop((TIMER_T *) NU_MODBASE(timer1_modinit.modname));
    
    int delta = (int) (timestamp - us_ticker_read());
    // NOTE: If this event was in the past, arm an interrupt to be triggered immediately.
    cd_major_minor = NU_MAX(TMR_CMP_MIN, delta);
    
    us_ticker_arm_cd();
}

static void tmr0_vec(void)
{
    TIMER_ClearIntFlag((TIMER_T *) NU_MODBASE(timer0_modinit.modname));
    counter_major ++;
}

static void tmr1_vec(void)
{
    TIMER_ClearIntFlag((TIMER_T *) NU_MODBASE(timer1_modinit.modname));
    cd_major_minor -= cd_minor;
    if (cd_major_minor <= 0) {
        // NOTE: us_ticker_set_interrupt() may get called in us_ticker_irq_handler();
        us_ticker_irq_handler();
    }
    else {
        us_ticker_arm_cd();
    }
}

static void us_ticker_arm_cd(void)
{
    TIMER_T * timer1_base = (TIMER_T *) NU_MODBASE(timer1_modinit.modname);
    
    // Reset 8-bit PSC counter, 24-bit up counter value and CNTEN bit
    timer1_base->CTL |= TIMER_CTL_RSTCNT_Msk;
    // One-shot mode, Clock = 1 MHz 
    uint32_t clk_timer1 = TIMER_GetModuleClock((TIMER_T *) NU_MODBASE(timer1_modinit.modname));
    uint32_t prescale_timer1 = clk_timer1 / TMR_CLK_FREQ - 1;
    MBED_ASSERT((prescale_timer1 != (uint32_t) -1) && prescale_timer1 <= 127);
    timer1_base->CTL |= TIMER_ONESHOT_MODE | prescale_timer1 | TIMER_CTL_CNTDATEN_Msk;
    
    cd_minor = NU_CLAMP(cd_major_minor, TMR_CMP_MIN, TMR_CMP_MAX);
    timer1_base->CMP = cd_minor;
    
    TIMER_EnableInt(timer1_base);
    TIMER_Start(timer1_base);
}