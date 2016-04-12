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
 
#include "mbed-hal/i2c_api.h"

#if DEVICE_I2C

#include "cmsis.h"
#include "mbed-hal/pinmap.h"
#include "PeripheralPins.h"
#include "nu_modutil.h"
#include "nu_miscutil.h"
#include "nu_bitutil.h"
//#include "uvisor-lib/uvisor-lib.h"

static uint32_t i2c_modinit_mask = 0;

static const struct nu_modinit_s i2c_modinit_tab[] = {
    {I2C_0, I2C0_MODULE, 0, 0, I2C0_RST, I2C0_IRQn, NULL},
    {I2C_1, I2C1_MODULE, 0, 0, I2C1_RST, I2C1_IRQn, NULL},
    {I2C_2, I2C2_MODULE, 0, 0, I2C2_RST, I2C2_IRQn, NULL},
    {I2C_3, I2C3_MODULE, 0, 0, I2C3_RST, I2C3_IRQn, NULL},
    {I2C_4, I2C4_MODULE, 0, 0, I2C4_RST, I2C4_IRQn, NULL},
    
    {NC, 0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

static int i2c_do_write(i2c_t *obj, char data);
static int i2c_do_read(i2c_t *obj, char *data, int last);
#define NU_I2C_TIMEOUT_STAT_INT     5000
#define NU_I2C_TIMEOUT_STOP         5000
static int i2c_poll_status_timeout(i2c_t *obj, int (*is_status)(i2c_t *obj), uint32_t timeout);
static int i2c_is_stat_int(i2c_t *obj);
static int i2c_is_stop_det(i2c_t *obj);
static int i2c_revise_address(int address, int read);

#if DEVICE_I2C_ASYNCH
static void i2c_buffer_set(i2c_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length);
static void i2c_enable_vector_interrupt(i2c_t *obj, uint32_t handler, int enable);
#endif

void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
    uint32_t i2c_sda = pinmap_peripheral(sda, PinMap_I2C_SDA);
    uint32_t i2c_scl = pinmap_peripheral(scl, PinMap_I2C_SCL);
    obj->i2c.i2c = (I2CName) pinmap_merge(i2c_sda, i2c_scl);
    MBED_ASSERT((int)obj->i2c.i2c != NC);
    
    const struct nu_modinit_s *modinit = get_modinit(obj->i2c.i2c, i2c_modinit_tab);
    MBED_ASSERT(modinit != NULL);
    MBED_ASSERT(modinit->modname == obj->i2c.i2c);
    
    // Reset this module
    SYS_ResetModule(modinit->rsetidx);
    
    // Enable IP clock
    CLK_EnableModuleClock(modinit->clkidx);

    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);
    
#if DEVICE_I2C_ASYNCH
    obj->i2c.dma_usage = DMA_USAGE_NEVER;
    obj->i2c.event = 0;
    obj->i2c.stop = 0;
    obj->i2c.address = 0;
#endif

    I2C_Open((I2C_T *) NU_MODBASE(obj->i2c.i2c), 100000);
    
    // Mark this module to be inited.
    int i = modinit - i2c_modinit_tab;
    i2c_modinit_mask |= 1 << i;
}

int i2c_start(i2c_t *obj)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    
    uint32_t status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0x08:  // Start
        case 0x10:  // Master Repeat Start
            return 0;
    }
    
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_STA_Msk | I2C_CTL_SI_Msk);
    if (i2c_poll_status_timeout(obj, i2c_is_stat_int, NU_I2C_TIMEOUT_STAT_INT)) {
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0x08:  // Start
        case 0x10:  // Master Repeat Start
            return 0;
    }
    
    return I2C_ERROR_BUS_BUSY;
}

int i2c_stop(i2c_t *obj)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    
    uint32_t status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0xF8:  // Bus Releases. Exists in both master/slave modes, and won't raise.
        return 0;
    }
    
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_STO_Msk | I2C_CTL_SI_Msk);
    if (i2c_poll_status_timeout(obj, i2c_is_stop_det, NU_I2C_TIMEOUT_STOP)) {
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0xF8:  // Bus Releases. Exists in both master/slave modes, and won't raise.
        return 0;
    }
    
    return I2C_ERROR_BUS_BUSY;
}

void i2c_frequency(i2c_t *obj, int hz)
{
    I2C_SetBusClockFreq((I2C_T *) NU_MODBASE(obj->i2c.i2c), hz);
}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{
    int i;

    if (i2c_start(obj)) {
        i2c_stop(obj);
        return I2C_ERROR_BUS_BUSY;
    }

    if (i2c_do_write(obj, i2c_revise_address(address, 1))) {
        i2c_stop(obj);
        return I2C_ERROR_NO_SLAVE;
    }

    // Read in bytes
    for (i = 0; i < length; i ++) {
        int last = (i == (length - 1)) ? 1 : 0;
        if (i2c_do_read(obj, data + i, last)) {
            i2c_stop(obj);
            return i;
        }
    }

    // If not repeated start, send stop.
    if (stop) {
        i2c_stop(obj);
    }

    return length;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    int i;

    if (i2c_start(obj)) {
        i2c_stop(obj);
        return I2C_ERROR_BUS_BUSY;
    }

    if (i2c_do_write(obj, i2c_revise_address(address, 0))) {
        i2c_stop(obj);
        return I2C_ERROR_NO_SLAVE;
    }

    for (i = 0; i < length; i ++) {
        if (i2c_do_write(obj, data[i])) {
            i2c_stop(obj);
            return i;
        }
    }

    if (stop) {
        i2c_stop(obj);
    }

    return length;
}

void i2c_reset(i2c_t *obj)
{
    // FIXME: better reset mechanism?
#if 0
    i2c_stop(obj);
#else
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_STO_Msk | I2C_CTL_SI_Msk);
    i2c_poll_status_timeout(obj, i2c_is_stop_det, NU_I2C_TIMEOUT_STOP);
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);
#endif
}

int i2c_byte_read(i2c_t *obj, int last)
{
    char data = 0;
    i2c_do_read(obj, &data, last);
    return data;
}

int i2c_byte_write(i2c_t *obj, int data)
{
    return i2c_do_write(obj, (data & 0xFF));
}

int i2c_allow_powerdown(void)
{
    uint32_t modinit_mask = i2c_modinit_mask;
    while (modinit_mask) {
        int i2c_idx = nu_ctz(modinit_mask);
        const struct nu_modinit_s *modinit = i2c_modinit_tab + i2c_idx;
        if (modinit->modname != NC) {
            I2C_T *i2c_base = NU_MODBASE(modinit->modname);
            // Disallow entering power-down mode if I2C transfer is enabled.
            if (i2c_base->CTL & I2C_CTL_INTEN_Msk) {
                return 0;
            }
        }
        modinit_mask &= ~(1 << i2c_idx);
    }
    
    return 1;
}

static int i2c_do_write(i2c_t *obj, char data)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    uint32_t status;
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0x08:  // Start
        case 0x10:  // Master Repeat Start
        case 0x18:  // Master Transmit Address ACK
            break;
        case 0x20:  // Master Transmit Address NACK
            return I2C_ERROR_NO_SLAVE;
        case 0x28:  // Master Transmit Data ACK
            break;
        //case 0x30:  // Master Transmit Data NACK
        //case 0x38:  // Master Arbitration Lost
        //case 0x40:  // Master Receive Address ACK
        //case 0x48:  // Master Receive Address NACK
        //case 0x50:  // Master Receive Data ACK
        //case 0x58:  // Master Receive Data NACK
        //case 0x00:  // Bus error
        default:
            return I2C_ERROR_BUS_BUSY;
    }
    
    I2C_SET_DATA(i2c_base, data);
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);

    if (i2c_poll_status_timeout(obj, i2c_is_stat_int, NU_I2C_TIMEOUT_STAT_INT)) {
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        //case 0x08:  // Start
        //case 0x10:  // Master Repeat Start
        case 0x18:  // Master Transmit Address ACK
            return 0;
        case 0x20:  // Master Transmit Address NACK
            return I2C_ERROR_NO_SLAVE;
        case 0x28:  // Master Transmit Data ACK
            return 0;
        //case 0x30:  // Master Transmit Data NACK  
        //case 0x38:  // Master Arbitration Lost
        case 0x40:  // Master Receive Address ACK
            return 0;
        case 0x48:  // Master Receive Address NACK
            return I2C_ERROR_NO_SLAVE;
        //case 0x50:  // Master Receive Data ACK
        //case 0x58:  // Master Receive Data NACK
        //case 0x00:  // Bus error
        default:
            return I2C_ERROR_BUS_BUSY;
    }
}

static int i2c_do_read(i2c_t *obj, char *data, int last)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    uint32_t status;
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        //case 0x08:  // Start
        //case 0x10:  // Master Repeat Start
        //case 0x18:  // Master Transmit Address ACK
        //case 0x20:  // Master Transmit Address NACK
        //case 0x28:  // Master Transmit Data ACK
        //case 0x30:  // Master Transmit Data NACK
        //case 0x38:  // Master Arbitration Lost
        case 0x40:  // Master Receive Address ACK
            break;
        case 0x48:  // Master Receive Address NACK
            return I2C_ERROR_NO_SLAVE;
        case 0x50:  // Master Receive Data ACK
            break;
        case 0x58:  // Master Receive Data NACK
            return I2C_ERROR_BUS_BUSY;
        //case 0x00:  // Bus error
        default:
            return I2C_ERROR_BUS_BUSY;
    }
   
    I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk | (last ? 0: I2C_CTL_AA_Msk));

    if (i2c_poll_status_timeout(obj, i2c_is_stat_int, NU_I2C_TIMEOUT_STAT_INT)) {
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        //case 0x08:  // Start
        //case 0x10:  // Master Repeat Start
        //case 0x18:  // Master Transmit Address ACK
        //case 0x20:  // Master Transmit Address NACK
        //case 0x28:  // Master Transmit Data ACK
        //case 0x30:  // Master Transmit Data NACK
        //case 0x38:  // Master Arbitration Lost
        //case 0x40:  // Master Receive Address ACK
        //case 0x48:  // Master Receive Address NACK
        case 0x50:  // Master Receive Data ACK
        case 0x58:  // Master Receive Data NACK
            break;
        //case 0x00:  // Bus error
        default:
            return I2C_ERROR_BUS_BUSY;
    }
    
    *data = I2C_GET_DATA(i2c_base);
    return 0;
}

static int i2c_poll_status_timeout(i2c_t *obj, int (*is_status)(i2c_t *obj), uint32_t timeout)
{
    while (timeout -- && ! is_status(obj));

    return timeout == (uint32_t) -1;
}

static int i2c_is_stat_int(i2c_t *obj)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    return !! (i2c_base->CTL & I2C_CTL_SI_Msk);
}

static int i2c_is_stop_det(i2c_t *obj)
{
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    return ! (i2c_base->CTL & I2C_CTL_STO_Msk);
}

static int i2c_revise_address(int address, int read)
{
    // FIXME: Format of passed address is unclear.
    return read ? (address | 1) : (address & 0xFE);
}

#if DEVICE_I2C_ASYNCH

void i2c_transfer_asynch(i2c_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length, uint32_t address, uint32_t stop, uint32_t handler, uint32_t event, DMAUsage hint)
{
    // NOTE: NUC472 I2C only supports 7-bit slave address.
    MBED_ASSERT((address & 0xFFFFFF80) == 0);
    
    // NOTE: First transmit and then receive.
    
    (void) hint;
    obj->i2c.dma_usage = DMA_USAGE_NEVER;
    obj->i2c.stop = stop;
    obj->i2c.address = address;
    obj->i2c.event = event;
    i2c_buffer_set(obj, tx, tx_length, rx, rx_length);

    //I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    
    i2c_enable_vector_interrupt(obj, handler, 1);
    i2c_start(obj);
}

uint32_t i2c_irq_handler_asynch(i2c_t *obj)
{
    int event = 0;

    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    uint32_t status = I2C_GET_STATUS(i2c_base);
    switch (status) {
        case 0x08:  // Start
        case 0x10: {// Master Repeat Start
            if (obj->tx_buff.buffer && obj->tx_buff.pos < obj->tx_buff.length) {
                I2C_SET_DATA(i2c_base, (i2c_revise_address(obj->i2c.address, 0)));
                I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);
            }
            else if (obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length) {
                I2C_SET_DATA(i2c_base, (i2c_revise_address(obj->i2c.address, 1)));
                I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);
            }
            else {
                event = I2C_EVENT_TRANSFER_COMPLETE;
                if (obj->i2c.stop) {
                    i2c_stop(obj);
                }
            }
            break;
        }
        
        case 0x18:  // Master Transmit Address ACK
        case 0x28:  // Master Transmit Data ACK
            if (obj->tx_buff.buffer && obj->tx_buff.pos < obj->tx_buff.length) {
                uint8_t *tx = (uint8_t *)obj->tx_buff.buffer;
                I2C_SET_DATA(i2c_base, tx[obj->tx_buff.pos ++]);
                I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);
            }
            else if (obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length) {
                i2c_start(obj);
            }
            else {
                event = I2C_EVENT_TRANSFER_COMPLETE;
                if (obj->i2c.stop) {
                    i2c_stop(obj);
                }
            }
            break;
            
        case 0x20:  // Master Transmit Address NACK
            event = I2C_EVENT_ERROR_NO_SLAVE;
            if (obj->i2c.stop) {
                i2c_stop(obj);
            }
            break;
            
        case 0x30:  // Master Transmit Data NACK
            event = I2C_EVENT_TRANSFER_EARLY_NACK;
            if (obj->i2c.stop) {
                i2c_stop(obj);
            }
            break;
                
        case 0x38:  // Master Arbitration Lost
            I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk);  // Enter not addressed SLV mode
            event = I2C_EVENT_ERROR;
            break;
            
        case 0x50:  // Master Receive Data ACK
            if (obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length) {
                uint8_t *rx = (uint8_t *) obj->rx_buff.buffer;
                rx[obj->rx_buff.pos ++] = I2C_GET_DATA(((I2C_T *) NU_MODBASE(obj->i2c.i2c)));
            }
        case 0x40:  // Master Receive Address ACK
            I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_SI_Msk | ((obj->rx_buff.pos != obj->rx_buff.length - 1) ? I2C_CTL_AA_Msk : 0));
            break;
            
        case 0x48:  // Master Receive Address NACK    
            event = I2C_EVENT_ERROR_NO_SLAVE;
            if (obj->i2c.stop) {
                i2c_stop(obj);
            }
            break;
            
        case 0x58:  // Master Receive Data NACK
            if (obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length) {
                uint8_t *rx = (uint8_t *) obj->rx_buff.buffer;
                rx[obj->rx_buff.pos ++] = I2C_GET_DATA(((I2C_T *) NU_MODBASE(obj->i2c.i2c)));
            }
            I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_STA_Msk | I2C_CTL_SI_Msk);
            break;
            
        case 0x00:  // Bus error
            event = I2C_EVENT_ERROR;
            i2c_reset(obj);
            break;
            
        default:
            event = I2C_EVENT_ERROR;
            if (obj->i2c.stop) {
                i2c_stop(obj);
            }
    }
    
    if (event) {
        i2c_enable_vector_interrupt(obj, 0, 0);
    }

    return (event & obj->i2c.event);
}

uint8_t i2c_active(i2c_t *obj)
{   
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    return !! (i2c_base->CTL & I2C_CTL_INTEN_Msk);
}

void i2c_abort_asynch(i2c_t *obj)
{
    i2c_enable_vector_interrupt(obj, 0, 0);
    i2c_stop(obj);
}

static void i2c_buffer_set(i2c_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length)
{
    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
}

static void i2c_enable_vector_interrupt(i2c_t *obj, uint32_t handler, int enable)
{
    const struct nu_modinit_s *modinit = get_modinit(obj->i2c.i2c, i2c_modinit_tab);
    MBED_ASSERT(modinit != NULL);
    MBED_ASSERT(modinit->modname == obj->i2c.i2c);
    
    I2C_T *i2c_base = (I2C_T *) NU_MODBASE(obj->i2c.i2c);
    
    if (enable) {
        NVIC_SetVector(modinit->irq_n, handler);
        NVIC_EnableIRQ(modinit->irq_n);
        // NOTE: Implementation of I2C_EnableInt() will cause side effect. Replace it.
        //I2C_EnableInt(i2c_base);
        I2C_SET_CONTROL_REG(i2c_base, I2C_CTL_INTEN_Msk);
    }
    else {
        //NVIC_SetVector(modinit->irq_n, handler);
        NVIC_DisableIRQ(modinit->irq_n);
        // NOTE: Implementation of I2C_DisableInt() will cause side effect. Replace it.
        //I2C_DisableInt(i2c_base);
        i2c_base->CTL = (i2c_base->CTL & ~0x3c) & ~I2C_CTL_INTEN_Msk;
    }
}

#endif

#endif
