// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   Based on UHS30 USB_HOST_SHIELD.h file suitable for Xarmlib
// @date    5 June 2020
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


class MAX3421E_HOST : public UHS_USB_HOST_BASE
{
    using DigitalOut = xarmlib::DigitalOut;
    using SpiMaster  = xarmlib::hal::SpiMaster;
    using Pin        = xarmlib::hal::Pin;
    using PinInt     = xarmlib::hal::PinInt;
    using Gpio       = xarmlib::hal::Gpio;
    using UsTicker   = xarmlib::hal::UsTicker;

public:

    UHS_NI MAX3421E_HOST(SpiMaster *spi_master, const Pin::Name spi_ss, const Pin::Name max_int) :
        m_spi(spi_master),
        m_ss(spi_ss, { Gpio::OutputMode::push_pull_high }),
        m_max_int(max_int, PinInt::InputModeConfig{}, PinInt::IntMode::interrupt_falling_edge)
    {}

    // Initialize USB hardware
    // NOTE: Don't forget to enable Port IRQ and set IRQ priority!
    void initialize();

    bool is_task_pending()    { return m_task_pending; }
    void clear_task_pending() { m_task_pending = false; }

    void task();

private:

    int32_t isr();

    void reset();
    void vbus_changed();
    void issue_bus_reset();

    void release_children();

    void doHostReset();

//    void UHS_NI vbusPower(VBUS_t state)
//    {
//        regWr(rPINCTL, bmFDUPSPI | (uint8_t)(state));
//    }

    uint8_t SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit);
    uint8_t OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data);
    uint8_t InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data);
    UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t* dataptr);
    uint8_t ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr);
    uint8_t ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr);
    uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

    void IsHub(bool p)
    {
        hub_present = p ? bmHUBPRE : 0;
    }

    // MAX3421e specific
    void write_reg(const uint8_t reg, const uint8_t data);
    uint8_t read_reg(const uint8_t reg);
    uint8_t* write_bytes(const uint8_t reg, uint8_t nbytes, uint8_t* data_p);
    uint8_t* read_bytes(const uint8_t reg, uint8_t nbytes, uint8_t* data_p);
    void write_gpio(uint8_t data);
    uint8_t read_gpio();

#ifdef DEBUG_USB_HOST
    void DBG_state();
#else
#define DBG_state VOID0
#endif

    SpiMaster  *m_spi;      // SPI master class instance pointer
    DigitalOut  m_ss;       // SPI slave select
    PinInt      m_max_int;  // MAX3421E INT IRQ pin

    volatile bool m_task_pending{ false };
    volatile bool m_bus_event{ false };
    volatile bool m_sof_event{ false };
};


#if !defined(USB_HOST_SHIELD_LOADED)
#include "spi_max3421e_INLINE.h"
#endif
#else
#error "define LOAD_MAX3421E in your sketch, never include spi_max3421e.h in a driver."
#endif
#endif /* SPI_MAX3421E_H */
