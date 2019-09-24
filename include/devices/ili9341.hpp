// ----------------------------------------------------------------------------
// @file    ili9341.hpp
// @brief   TFT LCD ILI9341 driver class.
// @date    4 September 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_DEVICES_ILI9341_HPP
#define __XARMLIB_DEVICES_ILI9341_HPP

#include "parallel_lcd.hpp"

namespace xarmlib
{




class Ili9341 : public ParallelLcd
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Ili9341(const Pin::Name   dc,
                const Pin::Name   wr,
                const Pin::Name   rd,
                const Pin::Name   reset,
                const Orientation orientation) : ParallelLcd(dc, wr, rd, reset, TFT_FORMAT_WIDTH, TFT_FORMAT_HEIGHT, orientation)
        {}

        void set_orientation(const Orientation orientation)
        {
            m_orientation = orientation;

            uint8_t parameter = 0;

            switch(m_orientation)
            {
                case Orientation::portrait:
                    parameter = MADCTL_MX | MADCTL_BGR;
                    m_width  = TFT_FORMAT_WIDTH;
                    m_height = TFT_FORMAT_HEIGHT;
                    break;
                case Orientation::landscape:
                    parameter = MADCTL_MV | MADCTL_BGR;
                    m_width  = TFT_FORMAT_HEIGHT;
                    m_height = TFT_FORMAT_WIDTH;
                    break;
                case Orientation::portrait_rotated:
                    parameter = MADCTL_MY | MADCTL_BGR;
                    m_width  = TFT_FORMAT_WIDTH;
                    m_height = TFT_FORMAT_HEIGHT;
                    break;
                case Orientation::landscape_rotated:
                    parameter = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
                    m_width  = TFT_FORMAT_HEIGHT;
                    m_height = TFT_FORMAT_WIDTH;
                    break;
            }

            write_command(CMD_MADCTL);
            write_8bits(parameter);
        }

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // ILI9341 command definitions
        enum CMD : uint8_t
        {
            CMD_NOP       = 0x00,   // No operation
            CMD_SWRESET   = 0x01,   // Software reset
            CMD_RDDID     = 0x04,   // Read display identification information
            CMD_RDDST     = 0x09,   // Read Display Status

            CMD_RDDPM     = 0x0A,   // Read Display Power Mode
            CMD_RDDMADCTL = 0x0B,   // Read Display MADCTL
            CMD_RDDCOLMOD = 0x0C,   // Read Display Pixel Format
            CMD_RDDIM     = 0x0D,   // Read Display Image Mode
            CMD_RDDSDR    = 0x0F,   // Read Display Self-Diagnostic Result

            CMD_SPLIN     = 0x10,   // Enter Sleep Mode
            CMD_SLPOUT    = 0x11,   // Sleep Out
            CMD_PTLON     = 0x12,   // Partial Mode ON
            CMD_NORON     = 0x13,   // Normal Display Mode ON

            CMD_DINVOFF   = 0x20,   // Display Inversion OFF
            CMD_DINVON    = 0x21,   // Display Inversion ON
            CMD_GAMSET    = 0x26,   // Gamma Set
            CMD_DISPOFF   = 0x28,   // Display OFF
            CMD_DISPON    = 0x29,   // Display ON

            CMD_CASET     = 0x2A,   // Column Address Set
            CMD_PASET     = 0x2B,   // Page Address Set
            CMD_RAMWR     = 0x2C,   // Memory Write
            CMD_RGBSET    = 0x2D,   // Color Set
            CMD_RAMRD     = 0x2E,   // Memory Read

            CMD_PLTAR     = 0x30,   // Partial Area
            CMD_VSCRDEF   = 0x33,   // Vertical Scrolling Definition
            CMD_MADCTL    = 0x36,   // Memory Access Control
            CMD_VSCRSADD  = 0x37,   // Vertical Scrolling Start Address
            CMD_IDMOFF    = 0x38,   // Idle Mode OFF
            CMD_IDMON     = 0x39,   // Idle Mode ON
            CMD_PIXSET    = 0x3A,   // COLMOD: Pixel Format Set

            CMD_FRMCTR1   = 0xB1,   // Frame Rate Control (In Normal Mode/Full Colors)
            CMD_FRMCTR2   = 0xB2,   // Frame Rate Control (In Idle Mode/8 colors)
            CMD_FRMCTR3   = 0xB3,   // Frame Rate control (In Partial Mode/Full Colors)
            CMD_INVTR     = 0xB4,   // Display Inversion Control
            CMD_DISCTRL   = 0xB6,   // Display Function Control

            CMD_PWCTRL1   = 0xC0,   // Power Control 1
            CMD_PWCTRL2   = 0xC1,   // Power Control 2
            CMD_VMCTRL1   = 0xC5,   // VCOM Control 1
            CMD_VMCTRL2   = 0xC7,   // VCOM Control 2

            CMD_PWCTRLA   = 0xCB,   // Power control A (Extend register command)
            CMD_PWCTRLB   = 0xCF,   // Power control B (Extend register command)

            CMD_RDID1     = 0xDA,   // Read ID 1
            CMD_RDID2     = 0xDB,   // Read ID 2
            CMD_RDID3     = 0xDC,   // Read ID 3
            CMD_RDID4     = 0xD3,   // Read ID 4

            CMD_PGAMCTRL  = 0xE0,   // Positive Gamma Control
            CMD_NGAMCTRL  = 0xE1,   // Negative Gamma Correction

            CMD_DTCTRLA   = 0xE8,   // Driver timing control A (Extend register command)
            CMD_DTCTRLB   = 0xEA,   // Driver timing control B (Extend register command)
            CMD_PWOSCTRL  = 0xED,   // Power on sequence control (Extend register command)

            CMD_PRCTRL    = 0xF7    // Pump ratio control (Extend register command)
        };

        // Initialization command structure
        template <uint8_t ArgMaxSize>
        struct InitCmd
        {
            CMD     cmd;
            uint8_t arg_array_size;
            uint8_t arg_array[ArgMaxSize];
            uint8_t wait_ms = 0;
        };

        // Initialization array type
        template <uint8_t Size, uint8_t ArgMaxSize>
        using InitCmdArray = std::array<InitCmd<ArgMaxSize>, Size>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void reset()
        {
            if(is_hardware_reset() == true)
            {
                do_hardware_reset();
            }
            else
            {
                // No hardware reset pin => engage software reset
                write_command(CMD_SWRESET);

                // Sleep Out command cannot be sent for 120 ms after applying software reset
                UsTicker::wait(std::chrono::milliseconds(120));
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // TFT format definitions
        enum TFT_FORMAT : uint16_t
        {
            TFT_FORMAT_WIDTH  = 240,    // ILI9341 max TFT width
            TFT_FORMAT_HEIGHT = 320     // ILI9341 max TFT height
        };

        // Memory Access Control register
        enum MADCTL : uint8_t
        {
            MADCTL_MY  = 0x80,  // Bottom to top
            MADCTL_MX  = 0x40,  // Right to left
            MADCTL_MV  = 0x20,  // Reverse mode
            MADCTL_ML  = 0x10,  // LCD refresh bottom to top
            MADCTL_RGB = 0x00,  // Red-Green-Blue pixel order
            MADCTL_BGR = 0x08,  // Blue-Green-Red pixel order
            MADCTL_MH  = 0x04   // LCD refresh right to left
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void set_address_window(const uint16_t x1, const uint16_t y1, const uint16_t width, const uint16_t height)
        {
            assert(x1 < m_width);
            assert(y1 < m_height);

            const uint16_t x2 = (x1 + width - 1);
            const uint16_t y2 = (y1 + height - 1);

            assert(x2 < m_width);
            assert(y2 < m_height);

            write_command(CMD_CASET);   // Column address set
            write_16bits(x1);
            write_16bits(x2);
            write_command(CMD_PASET);   // Row address set
            write_16bits(y1);
            write_16bits(y2);
            write_command(CMD_RAMWR);   // Write to RAM
        }
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_ILI9341_HPP
