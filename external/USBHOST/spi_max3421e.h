// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   USB_Host_Shield_2.0 usbhost.h file suitable for Xarmlib
// @date    8 June 2020
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

#if !defined(_usb_h_) || defined(__SPI_MAX3421E_H__)
#error "Never include spi_max3421e.h directly; include Usb.h instead"
#else
#define __SPI_MAX3421E_H__

#include "api/api_digital_in.hpp"
#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_us_ticker.hpp"


typedef enum
{
    vbus_on = 0,
    vbus_off = GPX_VBDET
} VBUS_t;

class MAX3421E
{
    using DigitalOut = xarmlib::DigitalOut;
    using DigitalIn  = xarmlib::DigitalIn;
    using SpiMaster  = xarmlib::hal::SpiMaster;
    using Pin        = xarmlib::hal::Pin;
    using Gpio       = xarmlib::hal::Gpio;
    using UsTicker   = xarmlib::hal::UsTicker;

    inline static uint8_t vbusState = 0;

    SpiMaster *pSpi;    // SPI master class instance pointer
    DigitalOut spiSs;   // SPI slave select
    DigitalIn  maxInt;  // MAX3421E INT input pin

public:

    // Constructor
    MAX3421E(SpiMaster *spi_master, const Pin::Name spi_ss, const Pin::Name max_int) :
        pSpi(spi_master),
        spiSs(spi_ss, { Gpio::OutputMode::push_pull_high }),
        maxInt(max_int, { Gpio::InputMode::pull_up })
    {}

    // Write single byte into MAX3421 register
    void regWr(uint8_t reg, uint8_t data)
    {
        pSpi->mutex_take();
        spiSs = 0;

        pSpi->transfer(reg | 0x02);
        pSpi->transfer(data);

        spiSs = 1;
        pSpi->mutex_give();
    }

    // Multiple-byte write
    // returns a pointer to memory position after last written
    uint8_t* bytesWr(uint8_t reg, uint8_t nbytes, uint8_t* data_p)
    {
        pSpi->mutex_take();
        spiSs = 0;

        pSpi->transfer(reg | 0x02);

        while(nbytes)
        {
            pSpi->transfer(*data_p);
            nbytes--;
            data_p++; // advance data pointer
        }

        spiSs = 1;
        pSpi->mutex_give();

        return data_p;
    }

    // GPIO Write
    // GPIO byte is split between 2 registers, so two writes are needed to write one byte
    // GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2
    void gpioWr(uint8_t data)
    {
        regWr(rIOPINS1, data);
        data >>= 4;
        regWr(rIOPINS2, data);
    }

    // Single host register read
    uint8_t regRd(uint8_t reg)
    {
        pSpi->mutex_take();
        spiSs = 0;

        pSpi->transfer(reg);
        uint8_t rv = pSpi->transfer(0); // Send empty byte

        spiSs = 1;
        pSpi->mutex_give();

        return rv;
    }

    // Multiple-byte register read
    // returns a pointer to a memory position after last read
    uint8_t* bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p)
    {
        pSpi->mutex_take();
        spiSs = 0;

        pSpi->transfer(reg);

        while(nbytes)
        {
            *data_p++ = pSpi->transfer(0);
            nbytes--;
        }

        spiSs = 1;
        pSpi->mutex_give();

        return data_p;
    }

    // Reads the current GPI input values
    // (see gpioWr for explanation)
    // returns uint8_t Bitwise value of all 8 GPI inputs
    // GPIN pins are in high nibbles of IOPINS1, IOPINS2
    uint8_t gpioRd()
    {
        uint8_t gpin = 0;

        gpin  = regRd(rIOPINS2);        // pins 4-7
        gpin &= 0xf0;                   // clean lower nibble
        gpin |= (regRd(rIOPINS1) >> 4); // shift low bits and OR with upper from previous operation

        return gpin;
    }

    // Reads the current GPI output values
    // returns uint8_t Bitwise value of all 8 GPI outputs
    // GPOUT pins are in low nibbles of IOPINS1, IOPINS2
    uint8_t gpioRdOutput()
    {
        uint8_t gpout = 0;

        gpout  = regRd(rIOPINS1);       // pins 0-3
        gpout &= 0x0f;                  // clean upper nibble
        gpout |= (regRd(rIOPINS2) << 4);// shift high bits and OR with lower from previous operation

        return gpout;
    }

    // Reset MAX3421E
    // returns number of cycles it took for PLL to stabilize after reset
    // or zero if PLL haven't stabilized in 65535 cycles
    uint16_t reset()
    {
        uint16_t i = 0;

        regWr(rUSBCTL, bmCHIPRES);
        regWr(rUSBCTL, 0x00);

        while(++i)
        {
            if((regRd(rUSBIRQ) & bmOSCOKIRQ))
            {
                break;
            }
        }

        return i;
    }

    // Initialize MAX3421E
    // returns 0 if success, -1 if not
    int8_t Init()
    {
        // Set full-duplex SPI and INT output pin to level-active
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

        if(reset() == 0) // OSCOKIRQ hasn't asserted in time
        {
            return -1;
        }

        // Set pull-downs and Host mode
        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
        // Peripheral Connect/Disconnect Interrupt Enable & Frame Generator Interrupt Enable
        regWr(rHIEN, bmCONDETIE | bmFRAMEIE);

        // Check if device is connected...

        // Sample the state of the USB bus
        regWr(rHCTL, bmSAMPLEBUS);

        // Wait for sample operation to finish
        while(!(regRd(rHCTL) & bmSAMPLEBUS));

        // Check if anything is connected
        busprobe();

        // Clear Peripheral Connect/Disconnect Interrupt Request
        regWr(rHIRQ, bmCONDETIRQ);
        // Enable the INT pin
        regWr(rCPUCTL, bmIE);

        return 0;
    }

    void vbusPower(VBUS_t state)
    {
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | state));
    }

    uint8_t getVbusState()
    {
        return vbusState;
    };

    // Probe bus to determine device presence and speed and switch host to this speed
    void busprobe()
    {
        // Get J,K status
        uint8_t bus_sample = regRd(rHRSL);
        // Zero the rest of the byte
        bus_sample &= (bmJSTATUS | bmKSTATUS);

        switch(bus_sample)  // start full-speed or low-speed host
        {
            case bmJSTATUS:
                if((regRd(rMODE) & bmLOWSPEED) == 0)
                {
                    regWr(rMODE, MODE_FS_HOST); // start full-speed host
                    vbusState = FSHOST;
                }
                else
                {
                    regWr(rMODE, MODE_LS_HOST); // start low-speed host
                    vbusState = LSHOST;
                }
                break;
            case bmKSTATUS:
                if((regRd(rMODE) & bmLOWSPEED) == 0)
                {
                    regWr(rMODE, MODE_LS_HOST); // start low-speed host
                    vbusState = LSHOST;
                }
                else
                {
                    regWr(rMODE, MODE_FS_HOST); // start full-speed host
                    vbusState = FSHOST;
                }
                break;
            case bmSE1: // illegal state
                vbusState = SE1;
                break;
            case bmSE0: // disconnected state
                regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);
                vbusState = SE0;
                break;
        }
    }

    uint8_t IntHandler()
    {
        uint8_t HIRQ_sendback = 0x00;

        // Determine interrupt source
        uint8_t HIRQ = regRd(rHIRQ);

        /*
        if(HIRQ & bmFRAMEIRQ) // -> 1ms SOF interrupt handler
        {
            HIRQ_sendback |= bmFRAMEIRQ;
        }
        */
        if(HIRQ & bmCONDETIRQ)
        {
            busprobe();

            HIRQ_sendback |= bmCONDETIRQ;
        }

        // End HIRQ interrupts handling, clear serviced IRQs
        regWr(rHIRQ, HIRQ_sendback);

        return HIRQ_sendback;
    }

    // State change task and interrupt handler
    uint8_t Task()
    {
        uint8_t rcode = 0;

        if(maxInt == 0)
        {
            rcode = IntHandler();
        }

        return rcode;
    }
};

#endif // __SPI_MAX3421E_H__
