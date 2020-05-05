// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   Based on UHS30 USB_HOST_SHIELD.h file suitable for Xarmlib
// @date    5 May 2020
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

// uncomment to get 'printf' console debugging. NOT FOR UNO!
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD

#ifdef LOAD_MAX3421E
#include "UHS_max3421e.h"
#include "api/api_digital_in.hpp"
#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"


#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD
#define MAX_HOST_DEBUG(...) printf_P(__VA_ARGS__)
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif

#if !defined(USB_HOST_SHIELD_USE_ISR)
#if defined(USE_MULTIPLE_APP_API)
#define USB_HOST_SHIELD_USE_ISR 0
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif


#define IRQ_IS_EDGE 0


// NOTE: On the max3421e the irq enable and irq bits are in the same position.

// IRQs used if CPU polls
#define ENIBITSPOLLED (bmCONDETIE | bmBUSEVENTIE  | bmFRAMEIE)
// IRQs used if CPU is interrupted
#define ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )

#if !USB_HOST_SHIELD_USE_ISR
#define IRQ_CHECK_MASK (ENIBITSPOLLED & ICLRALLBITS)
#define IRQ_IS_EDGE 0
#else
#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)
#endif


//#if !defined(IRQ_SENSE)
//#define IRQ_SENSE LOW
//#endif
//#if !defined(bmPULSEWIDTH)
//#define bmPULSEWIDTH 0
//#endif
//#if !defined(bmIRQ_SENSE)
//#define bmIRQ_SENSE bmINTLEVEL
//#endif

class MAX3421E_HOST : public UHS_USB_HOST_BASE
#if defined(SWI_IRQ_NUM)
, public dyn_SWI
#endif
{
    using DigitalOut = xarmlib::DigitalOut;
    using DigitalIn  = xarmlib::DigitalIn;
    using SpiMaster  = xarmlib::hal::SpiMaster;
    using Pin        = xarmlib::hal::Pin;
    using Gpio       = xarmlib::hal::Gpio;

    SpiMaster *pSpi;    // SPI master class instance pointer
    DigitalOut ss_pin;  // SPI slave select
    DigitalIn  irq_pin; // MAX3421E INT IRQ pin

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
        irq_pin(max_int, { Gpio::InputMode::pull_up })
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
#if !USB_HOST_SHIELD_USE_ISR
            Task();
#endif
        }

        return (!condet);
    }

    virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t* dataptr);

    virtual void UHS_NI vbusPower(VBUS_t state)
    {
        regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE) | (uint8_t)(state));
    }

    void UHS_NI Task(void);

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
#if USB_HOST_SHIELD_USE_ISR
        noInterrupts();
#endif
        doingreset = true;
        busevent = true;

        regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet
        regWr(rHCTL, bmBUSRST);      // issue bus reset

#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        interrupts();
#endif

        while(busevent)
        {
            DDSB();
            SYSTEM_OR_SPECIAL_YIELD();
        }
#endif
#if USB_HOST_SHIELD_USE_ISR
        noInterrupts();
#endif
        sofevent = true;

#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        interrupts();
#endif

        // Wait for SOF
        while(sofevent);

#if USB_HOST_SHIELD_USE_ISR
        noInterrupts();
#endif

        doingreset = false;

#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        interrupts();
    }

    int16_t UHS_NI Init(int16_t mseconds);

    int16_t UHS_NI Init(void)
    {
        return Init(INT16_MIN);
    }

    void ISRTask(void);
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

    // ARM/NVIC specific, used to emulate reentrant ISR.
#if defined(SWI_IRQ_NUM)

    void dyn_SWISR(void)
    {
        ISRbottom();
    }
#endif
};

#if !defined(SPIclass)
#define SPIclass SPI
#endif
#if !defined(USB_HOST_SHIELD_LOADED)
#include "spi_max3421e_INLINE.h"
#endif
#else
#error "define LOAD_MAX3421E in your sketch, never include spi_max3421e.h in a driver."
#endif
#endif /* SPI_MAX3421E_H */
