// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   Based on UHS30 USB_HOST_SHIELD.h file suitable for Xarmlib
// @date    20 May 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
//
// See README.md file for additional credits and acknowledgments.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// ----------------------------------------------------------------------------

#ifndef SPI_MAX3421E_H
#define SPI_MAX3421E_H

#ifdef LOAD_MAX3421E
#include "UHS_max3421e.h"
#include "api/api_digital_out.hpp"
#include "hal/hal_pin_int.hpp"
#include "hal/hal_spi.hpp"


#if !defined(DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD)
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD 0
#endif
#if DEBUG_PRINTF_EXTRA_HUGE
#if DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD
#define MAX_HOST_DEBUG(...) printf_P(__VA_ARGS__)
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif


// NOTE: On the max3421e the irq enable and irq bits are in the same position

// IRQs used if CPU is interrupted
#define ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )

#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)


class MAX3421E_HOST : public UHS_USB_HOST_BASE
{
public:

    // IRQ handlers definition
    using IrqHandlerType = int32_t();
    using IrqHandler     = xarmlib::Delegate<IrqHandlerType>;

private:

    using DigitalOut = xarmlib::DigitalOut;
    using SpiMaster  = xarmlib::hal::SpiMaster;
    using Pin        = xarmlib::hal::Pin;
    using PinInt     = xarmlib::hal::PinInt;
    using Gpio       = xarmlib::hal::Gpio;
    using UsTicker   = xarmlib::hal::UsTicker;

    SpiMaster *pSpi;             // SPI master class instance pointer
    DigitalOut ss_pin;           // SPI slave select
    PinInt     irq_pin;          // MAX3421E INT IRQ pin
    IrqHandler irq_user_handler; // User defined ISR bottom handler

    // TO-DO: move these into the parent class.
    volatile uint8_t vbusState;
    volatile uint16_t sof_countdown;

    // TO-DO: pack into a struct/union and use one byte
    volatile bool busevent;
    volatile bool sofevent;
    volatile bool counted;
    volatile bool condet;
    volatile bool doingreset;

public:

    UHS_NI MAX3421E_HOST(SpiMaster *spi_master, const Pin::Name spi_ss, const Pin::Name max_int) :
        pSpi(spi_master),
        ss_pin(spi_ss, { Gpio::OutputMode::push_pull_high }),
        irq_pin(max_int, PinInt::InputModeConfig{}, PinInt::IntMode::interrupt_falling_edge)
    {
        sof_countdown = 0;
        doingreset = false;
        busevent = false;
        sofevent = false;
        condet = false;
        hub_present = 0;
    }

    virtual bool UHS_NI sof_delay(uint16_t x)
    {
        sof_countdown = x;

        while((sof_countdown != 0) && !condet)
        {
            SYSTEM_OR_SPECIAL_YIELD();
        }

        return (!condet);
    }

    virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t* dataptr);

    virtual void UHS_NI vbusPower(VBUS_t state)
    {
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL) | (uint8_t)(state));
    }

    //void UHS_NI Task(void);

    virtual uint8_t SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit);
    virtual uint8_t OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data);
    virtual uint8_t InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data);
    virtual uint8_t ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr);
    virtual uint8_t ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr);
    virtual uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

    void UHS_NI ReleaseChildren(void)
    {
        for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
            if(devConfig[i])
                devConfig[i]->Release();

        hub_present = 0;
    }

    virtual void IsHub(bool p)
    {
        hub_present = p ? bmHUBPRE : 0;
    }

    virtual void VBUS_changed(void);

    virtual void UHS_NI doHostReset(void)
    {
        noInterrupts();

        doingreset = true;
        busevent = true;

        regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet
        regWr(rHCTL, bmBUSRST);      // issue bus reset

        interrupts();

        while(busevent)
        {
            SYSTEM_OR_SPECIAL_YIELD();
        }

        noInterrupts();

        sofevent = true;

        interrupts();

        // Wait for SOF
        while(sofevent);

        noInterrupts();

        doingreset = false;

        interrupts();
    }

    // NOTE: Don't forget to enable Port IRQ and set IRQ priority!
    int16_t UHS_NI Init(int16_t mseconds, const IrqHandler& irq_handler);

    int16_t UHS_NI Init(const IrqHandler& irq_handler)
    {
        return Init(INT16_MIN, irq_handler);
    }

    int32_t ISRTask(void);
    void ISRbottom(void);
    void busprobe(void);
    uint16_t reset(void);

    // MAX3421e specific
    void regWr(uint8_t reg, uint8_t data);
    void gpioWr(uint8_t data);
    uint8_t regRd(uint8_t reg);
    uint8_t gpioRd(void);
    uint8_t* bytesWr(uint8_t reg, uint8_t nbytes, uint8_t* data_p);
    uint8_t* bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p);
};


#if !defined(USB_HOST_SHIELD_LOADED)
#include "spi_max3421e_INLINE.h"
#endif
#else
#error "define LOAD_MAX3421E in your sketch, never include spi_max3421e.h in a driver."
#endif
#endif /* SPI_MAX3421E_H */
