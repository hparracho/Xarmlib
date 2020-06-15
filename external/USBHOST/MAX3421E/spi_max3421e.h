// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   Based on UHS30 USB_HOST_SHIELD.h file suitable for Xarmlib
// @date    9 June 2020
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


// NOTE: On the max3421e the irq enable and irq bits are in the same position.

// IRQs used if CPU is interrupted
#define ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )
#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)


class MAX3421E_HOST : public UHS_USB_HOST_BASE
{
        using DigitalOut = xarmlib::DigitalOut;
        using SpiMaster  = xarmlib::hal::SpiMaster;
        using Pin        = xarmlib::hal::Pin;
        using PinInt     = xarmlib::hal::PinInt;
        using Gpio       = xarmlib::hal::Gpio;
        using UsTicker   = xarmlib::hal::UsTicker;

        SpiMaster  *m_spi;      // SPI master class instance pointer
        DigitalOut  m_ss;       // SPI slave select
        PinInt      m_max_int;  // MAX3421E INT IRQ pin

        volatile bool m_interrupt_pending{ false };

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
        m_spi(spi_master),
        m_ss(spi_ss, { Gpio::OutputMode::push_pull_high }),
        m_max_int(max_int, PinInt::InputModeConfig{}, PinInt::IntMode::interrupt_logic_zero)
        {
            sof_countdown = 0;
            busevent = false;
            doingreset = false;
            sofevent = false;
            condet = false;
            hub_present = 0;
        }

        virtual bool UHS_NI sof_delay(uint16_t x)
        {
            sof_countdown = x;
            while((sof_countdown != 0) && !condet)
            {
                    //SYSTEM_OR_SPECIAL_YIELD();
            }

            return (!condet);
        }

        virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t* dataptr);

        virtual void UHS_NI vbusPower(VBUS_t state)
        {
            regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL) | (uint8_t)(state));
        };

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
            if(p) {
                    hub_present = bmHUBPRE;
            } else {
                    hub_present = 0;
            }
        }

        virtual void VBUS_changed(void);

        virtual void UHS_NI doHostReset(void)
        {
            doingreset = true;
            busevent = true;
            regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.
            regWr(rHCTL, bmBUSRST); //issue bus reset

            while(busevent)
            {
                //SYSTEM_OR_SPECIAL_YIELD();
            }

            sofevent = true;

            // Wait for SOF
            while(sofevent)
            {}

            doingreset = false;
        }

        int16_t UHS_NI Init(int16_t mseconds);

        int16_t UHS_NI Init(void)
        {
            return Init(INT16_MIN);
        }

        bool is_interrupt_pending()    { return m_interrupt_pending; }
        void clear_interrupt_pending() { m_interrupt_pending = false; }

        int32_t ISRTask();
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

        virtual void UHS_NI suspend_host(void)
        {
                // Used on MCU that lack control of IRQ priority (AVR).
                // Suspends ISRs, for critical code. IRQ will be serviced after it is resumed.
                // NOTE: you must track the state yourself!
        }

        virtual void UHS_NI resume_host(void);
};


#if !defined(USB_HOST_SHIELD_LOADED)
#include "spi_max3421e_INLINE.h"
#endif
#else
#error "define LOAD_MAX3421E in your sketch, never include spi_max3421e.h in a driver."
#endif
#endif /* USB_HOST_SHIELD_H */
