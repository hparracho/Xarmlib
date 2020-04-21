// ----------------------------------------------------------------------------
// @file    spi_max3421e.h
// @brief   SPI MAX3421E driver class.
// @notes   Strongly based on max3421e.h and usbhost.h files from
//          https://github.com/felis/USB_Host_Shield_2.0
//          (commit as of 13 September 2019)
// @date    21 April 2020
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




#define SE0     0
#define SE1     1
#define FSHOST  2
#define LSHOST  3

/* MAX3421E command byte format: rrrrr0wa where 'r' is register number  */
//
// MAX3421E Registers in HOST mode.
//
#define rRCVFIFO    0x08    //1<<3
#define rSNDFIFO    0x10    //2<<3
#define rSUDFIFO    0x20    //4<<3
#define rRCVBC      0x30    //6<<3
#define rSNDBC      0x38    //7<<3

#define rUSBIRQ     0x68    //13<<3
/* USBIRQ Bits  */
#define bmVBUSIRQ   0x40    //b6
#define bmNOVBUSIRQ 0x20    //b5
#define bmOSCOKIRQ  0x01    //b0

#define rUSBIEN     0x70    //14<<3
/* USBIEN Bits  */
#define bmVBUSIE    0x40    //b6
#define bmNOVBUSIE  0x20    //b5
#define bmOSCOKIE   0x01    //b0

#define rUSBCTL     0x78    //15<<3
/* USBCTL Bits  */
#define bmCHIPRES   0x20    //b5
#define bmPWRDOWN   0x10    //b4

#define rCPUCTL     0x80    //16<<3
/* CPUCTL Bits  */
#define bmPUSLEWID1 0x80    //b7
#define bmPULSEWID0 0x40    //b6
#define bmIE        0x01    //b0

#define rPINCTL     0x88    //17<<3
/* PINCTL Bits  */
#define bmFDUPSPI   0x10    //b4
#define bmINTLEVEL  0x08    //b3
#define bmPOSINT    0x04    //b2
#define bmGPXB      0x02    //b1
#define bmGPXA      0x01    //b0
// GPX pin selections
#define GPX_OPERATE 0x00
#define GPX_VBDET   0x01
#define GPX_BUSACT  0x02
#define GPX_SOF     0x03

#define rREVISION   0x90    //18<<3

#define rIOPINS1    0xa0    //20<<3

/* IOPINS1 Bits */
#define bmGPOUT0    0x01
#define bmGPOUT1    0x02
#define bmGPOUT2    0x04
#define bmGPOUT3    0x08
#define bmGPIN0     0x10
#define bmGPIN1     0x20
#define bmGPIN2     0x40
#define bmGPIN3     0x80

#define rIOPINS2    0xa8    //21<<3
/* IOPINS2 Bits */
#define bmGPOUT4    0x01
#define bmGPOUT5    0x02
#define bmGPOUT6    0x04
#define bmGPOUT7    0x08
#define bmGPIN4     0x10
#define bmGPIN5     0x20
#define bmGPIN6     0x40
#define bmGPIN7     0x80

#define rGPINIRQ    0xb0    //22<<3
/* GPINIRQ Bits */
#define bmGPINIRQ0 0x01
#define bmGPINIRQ1 0x02
#define bmGPINIRQ2 0x04
#define bmGPINIRQ3 0x08
#define bmGPINIRQ4 0x10
#define bmGPINIRQ5 0x20
#define bmGPINIRQ6 0x40
#define bmGPINIRQ7 0x80

#define rGPINIEN    0xb8    //23<<3
/* GPINIEN Bits */
#define bmGPINIEN0 0x01
#define bmGPINIEN1 0x02
#define bmGPINIEN2 0x04
#define bmGPINIEN3 0x08
#define bmGPINIEN4 0x10
#define bmGPINIEN5 0x20
#define bmGPINIEN6 0x40
#define bmGPINIEN7 0x80

#define rGPINPOL    0xc0    //24<<3
/* GPINPOL Bits */
#define bmGPINPOL0 0x01
#define bmGPINPOL1 0x02
#define bmGPINPOL2 0x04
#define bmGPINPOL3 0x08
#define bmGPINPOL4 0x10
#define bmGPINPOL5 0x20
#define bmGPINPOL6 0x40
#define bmGPINPOL7 0x80

#define rHIRQ       0xc8    //25<<3
/* HIRQ Bits */
#define bmBUSEVENTIRQ   0x01   // indicates BUS Reset Done or BUS Resume
#define bmRWUIRQ        0x02
#define bmRCVDAVIRQ     0x04
#define bmSNDBAVIRQ     0x08
#define bmSUSDNIRQ      0x10
#define bmCONDETIRQ     0x20
#define bmFRAMEIRQ      0x40
#define bmHXFRDNIRQ     0x80

#define rHIEN           0xd0    //26<<3

/* HIEN Bits */
#define bmBUSEVENTIE    0x01
#define bmRWUIE         0x02
#define bmRCVDAVIE      0x04
#define bmSNDBAVIE      0x08
#define bmSUSDNIE       0x10
#define bmCONDETIE      0x20
#define bmFRAMEIE       0x40
#define bmHXFRDNIE      0x80

#define rMODE           0xd8    //27<<3

/* MODE Bits */
#define bmHOST          0x01
#define bmLOWSPEED      0x02
#define bmHUBPRE        0x04
#define bmSOFKAENAB     0x08
#define bmSEPIRQ        0x10
#define bmDELAYISO      0x20
#define bmDMPULLDN      0x40
#define bmDPPULLDN      0x80

#define rPERADDR    0xe0    //28<<3

#define rHCTL       0xe8    //29<<3
/* HCTL Bits */
#define bmBUSRST        0x01
#define bmFRMRST        0x02
#define bmSAMPLEBUS     0x04
#define bmSIGRSM        0x08
#define bmRCVTOG0       0x10
#define bmRCVTOG1       0x20
#define bmSNDTOG0       0x40
#define bmSNDTOG1       0x80

#define rHXFR       0xf0    //30<<3
/* Host transfer token values for writing the HXFR register (R30)   */
/* OR this bit field with the endpoint number in bits 3:0               */
#define tokSETUP  0x10  // HS=0, ISO=0, OUTNIN=0, SETUP=1
#define tokIN     0x00  // HS=0, ISO=0, OUTNIN=0, SETUP=0
#define tokOUT    0x20  // HS=0, ISO=0, OUTNIN=1, SETUP=0
#define tokINHS   0x80  // HS=1, ISO=0, OUTNIN=0, SETUP=0
#define tokOUTHS  0xA0  // HS=1, ISO=0, OUTNIN=1, SETUP=0
#define tokISOIN  0x40  // HS=0, ISO=1, OUTNIN=0, SETUP=0
#define tokISOOUT 0x60  // HS=0, ISO=1, OUTNIN=1, SETUP=0

#define rHRSL       0xf8    //31<<3

/* HRSL Bits */
#define bmRCVTOGRD  0x10
#define bmSNDTOGRD  0x20
#define bmKSTATUS   0x40
#define bmJSTATUS   0x80
#define bmSE0       0x00    //SE0 - disconnect state
#define bmSE1       0xc0    //SE1 - illegal state

/* Host error result codes, the 4 LSB's in the HRSL register */
#define hrSUCCESS   0x00
#define hrBUSY      0x01
#define hrBADREQ    0x02
#define hrUNDEF     0x03
#define hrNAK       0x04
#define hrSTALL     0x05
#define hrTOGERR    0x06
#define hrWRONGPID  0x07
#define hrBADBC     0x08
#define hrPIDERR    0x09
#define hrPKTERR    0x0A
#define hrCRCERR    0x0B
#define hrKERR      0x0C
#define hrJERR      0x0D
#define hrTIMEOUT   0x0E
#define hrBABBLE    0x0F

#define MODE_FS_HOST    (bmDPPULLDN|bmDMPULLDN|bmHOST|bmSOFKAENAB)
#define MODE_LS_HOST    (bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB)




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
        maxInt(max_int, { Gpio::InputMode::hiz }) //@TODO: mode should be confirmed
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

        gpin = regRd(rIOPINS2); //pins 4-7
        gpin &= 0xf0; //clean lower nibble
        gpin |= (regRd(rIOPINS1) >> 4); //shift low bits and OR with upper from previous operation

        return gpin;
    }

    // Reads the current GPI output values
    // returns uint8_t Bitwise value of all 8 GPI outputs
    // GPOUT pins are in low nibbles of IOPINS1, IOPINS2
    uint8_t gpioRdOutput()
    {
        uint8_t gpout = 0;

        gpout = regRd(rIOPINS1); //pins 0-3
        gpout &= 0x0f; //clean upper nibble
        gpout |= (regRd(rIOPINS2) << 4); //shift high bits and OR with lower from previous operation

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
    // set Host mode, pullups, and stuff
    // returns 0 if success, -1 if not
    int8_t Init()
    {
        /* MAX3421E - full-duplex SPI, level interrupt */
        // GPX pin on. Moved here, otherwise we flicker the vbus.
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

        if(reset() == 0) // OSCOKIRQ hasn't asserted in time
        {
            return -1;
        }

        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

        regWr(rHIEN, bmCONDETIE | bmFRAMEIE); // connection detection

        /* check if device is connected */
        regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
        while(!(regRd(rHCTL) & bmSAMPLEBUS)); // wait for sample operation to finish

        busprobe(); // check if anything is connected

        regWr(rHIRQ, bmCONDETIRQ); // clear connection detect interrupt
        regWr(rCPUCTL, 0x01); // enable interrupt pin

        return 0;
    }

    // Initialize MAX3421E
    // set Host mode, pullups, and stuff
    // returns 0 if success, -1 if not
    int8_t Init(int mseconds)
    {
        /* MAX3421E - full-duplex SPI, level interrupt, vbus off */
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | GPX_VBDET));

        if(reset() == 0) // OSCOKIRQ hasn't asserted in time
        {
            return -1;
        }

        // Delay a minimum of 1 second to ensure any capacitors are drained.
        // 1 second is required to make sure we do not smoke a Microdrive!
        if(mseconds < 1000) mseconds = 1000;
        delay(mseconds);

        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

        regWr(rHIEN, bmCONDETIE | bmFRAMEIE); // connection detection

        /* check if device is connected */
        regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
        while(!(regRd(rHCTL) & bmSAMPLEBUS)); // wait for sample operation to finish

        busprobe(); // check if anything is connected

        regWr(rHIRQ, bmCONDETIRQ); // clear connection detect interrupt
        regWr(rCPUCTL, 0x01); // enable interrupt pin

        // GPX pin on. This is done here so that busprobe will fail if we have a switch connected.
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

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
        uint8_t bus_sample = regRd(rHRSL); // Get J,K status
        bus_sample &= (bmJSTATUS | bmKSTATUS); // zero the rest of the byte

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
        uint8_t HIRQ;
        uint8_t HIRQ_sendback = 0x00;

        HIRQ = regRd(rHIRQ); // determine interrupt source

        if(HIRQ & bmCONDETIRQ)
        {
            busprobe();

            HIRQ_sendback |= bmCONDETIRQ;
        }

        // End HIRQ interrupts handling, clear serviced IRQs
        regWr(rHIRQ, HIRQ_sendback);

        return HIRQ_sendback;
    }

    // MAX3421E state change task and interrupt handler
    uint8_t Task()
    {
        uint8_t rcode = 0;
        uint8_t pinvalue;
        //USB_HOST_SERIAL.print("Vbus state: ");
        //USB_HOST_SERIAL.println( vbusState, HEX );
        pinvalue = INTR::IsSet(); //Read();
        //pinvalue = digitalRead( MAX_INT );
        if(pinvalue == 0) {
                rcode = IntHandler();
        }
        //    pinvalue = digitalRead( MAX_GPX );
        //    if( pinvalue == LOW ) {
        //        GpxHandler();
        //    }
        //    usbSM();                                //USB state machine
        return ( rcode);
    }
};

#endif // __SPI_MAX3421E_H__
