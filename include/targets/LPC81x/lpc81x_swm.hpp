// ----------------------------------------------------------------------------
// @file    lpc81x_swm.hpp
// @brief   NXP LPC81x Switch Matrix (SWM) class.
// @date    9 July 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_LPC81X_SWM_HPP
#define __XARMLIB_TARGETS_LPC81X_SWM_HPP

#include "targets/LPC81x/lpc81x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class Swm
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Switch Matrix movable pins
        enum class PinMovable
        {
            U0_TXD_O        = 0x00, // PINASSIGN0 - UART0 TXD Output
            U0_RXD_I        = 0x01, // PINASSIGN0 - UART0 RXD Input
            U0_RTS_O        = 0x02, // PINASSIGN0 - UART0 RTS Output
            U0_CTS_I        = 0x03, // PINASSIGN0 - UART0 CTS Input
            U0_SCLK_IO      = 0x10, // PINASSIGN1 - UART0 SCLK I/O
            U1_TXD_O        = 0x11, // PINASSIGN1 - UART1 TXD Output
            U1_RXD_I        = 0x12, // PINASSIGN1 - UART1 RXD Input
            U1_RTS_O        = 0x13, // PINASSIGN1 - UART1 RTS Output
            U1_CTS_I        = 0x20, // PINASSIGN2 - UART1 CTS Input
            U1_SCLK_IO      = 0x21, // PINASSIGN2 - UART1 SCLK I/O
            U2_TXD_O        = 0x22, // PINASSIGN2 - UART2 TXD Output
            U2_RXD_I        = 0x23, // PINASSIGN2 - UART2 RXD Input
            U2_RTS_O        = 0x30, // PINASSIGN3 - UART2 RTS Output
            U2_CTS_I        = 0x31, // PINASSIGN3 - UART2 CTS Input
            U2_SCLK_IO      = 0x32, // PINASSIGN3 - UART2 SCLK I/O
            SPI0_SCK_IO     = 0x33, // PINASSIGN3 - SPI0 SCK I/O
            SPI0_MOSI_IO    = 0x40, // PINASSIGN4 - SPI0 MOSI I/O
            SPI0_MISO_IO    = 0x41, // PINASSIGN4 - SPI0 MISO I/O
            SPI0_SSEL_IO    = 0x42, // PINASSIGN4 - SPI0 SSEL I/O
            SPI1_SCK_IO     = 0x43, // PINASSIGN4 - SPI1 SCK I/O
            SPI1_MOSI_IO    = 0x50, // PINASSIGN5 - SPI1 MOSI I/O
            SPI1_MISO_IO    = 0x51, // PINASSIGN5 - SPI1 MISO I/O
            SPI1_SSEL_IO    = 0x52, // PINASSIGN5 - SPI1 SSEL I/O
            SCT_PIN0_I      = 0x53, // PINASSIGN5 - SCT PIN0 Input
            SCT_PIN1_I      = 0x60, // PINASSIGN6 - SCT PIN1 Input
            SCT_PIN2_I      = 0x61, // PINASSIGN6 - SCT PIN2 Input
            SCT_PIN3_I      = 0x62, // PINASSIGN6 - SCT PIN3 Input
            SCT_OUT0_O      = 0x63, // PINASSIGN6 - SCT OUT0 Output
            SCT_OUT1_O      = 0x70, // PINASSIGN7 - SCT OUT1 Output
            SCT_OUT2_O      = 0x71, // PINASSIGN7 - SCT OUT2 Output
            SCT_OUT3_O      = 0x72, // PINASSIGN7 - SCT OUT3 Output
            I2C_SDA_IO      = 0x73, // PINASSIGN7 - I2C SDA I/O
            I2C_SCL_IO      = 0x80, // PINASSIGN8 - I2C SCL I/O
            ACMP_O_O        = 0x81, // PINASSIGN8 - Analog comparator Output
            CLKOUT_O        = 0x82, // PINASSIGN8 - CLKOUT Output
            GPIO_INT_BMAT_O = 0x83  // PINASSIGN8 - GPIO INT BMAT Output
        };

        // Switch Matrix fixed pins
        enum class PinFixed
        {
            ACMP_I1 = 0,            // ACMP I1
            ACMP_I2 = 1,            // ACMP I2
            SWCLK   = 2,            // SWCLK
            SWDIO   = 3,            // SWDIO
            XTALIN  = 4,            // XTALIN
            XTALOUT = 5,            // XTALOUT
            RST     = 6,            // Reset
            CLKIN   = 7,            // Clock Input
            VDDCMP  = 8             // VDD
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Assign a movable pin function to a physical pin in Switch Matrix
        static void assign(const PinMovable movable, const Pin::Name pin)
        {
            if(pin == Pin::Name::NC)
            {
                return;
            }

            const  int32_t pin_index = static_cast<int32_t>(movable) >> 4;
            const uint32_t pin_shift = (static_cast<uint32_t>(movable) & 0x0F) << 3;
            const uint32_t reg_value = LPC_SWM->PINASSIGN[pin_index] & (~(0xFF << pin_shift));

            LPC_SWM->PINASSIGN[pin_index] = reg_value | (static_cast<uint32_t>(pin) << pin_shift);
        }

        // Unassign a movable pin function from a physical pin in Switch Matrix
        static void unassign(const PinMovable movable)
        {
            (void)movable;

            // Helder Parracho @ 22 March 2018
            // @TODO: Implement this function for completeness sake.
        }

        // Enable a fixed function pin in the Switch Matrix
        static void enable(const PinFixed fixed)
        {
            LPC_SWM->PINENABLE0 &= ~(1 << static_cast<uint32_t>(fixed));
        }

        // Disable a fixed function pin in the Switch Matrix
        static void disable(const PinFixed fixed)
        {
            LPC_SWM->PINENABLE0 |= (1 << static_cast<uint32_t>(fixed));
        }
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_SWM_HPP
