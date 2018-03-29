// ----------------------------------------------------------------------------
// @file    lpc84x_swm.hpp
// @brief   NXP LPC84x Switch Matrix (SWM) class.
// @date    28 March 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_SWM_HPP
#define __XARMLIB_TARGETS_LPC84X_SWM_HPP

#include "targets/LPC84x/lpc84x_pins.hpp"

namespace xarmlib
{
namespace lpc84x
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
            U0_TXD_O        = 0x00,     // PINASSIGN0 - UART0 TXD Output
            U0_RXD_I        = 0x01,     // PINASSIGN0 - UART0 RXD Input
            U0_RTS_O        = 0x02,     // PINASSIGN0 - UART0 RTS Output
            U0_CTS_I        = 0x03,     // PINASSIGN0 - UART0 CTS Input
            U0_SCLK_IO      = 0x10,     // PINASSIGN1 - UART0 SCLK I/O
            U1_TXD_O        = 0x11,     // PINASSIGN1 - UART1 TXD Output
            U1_RXD_I        = 0x12,     // PINASSIGN1 - UART1 RXD Input
            U1_RTS_O        = 0x13,     // PINASSIGN1 - UART1 RTS Output
            U1_CTS_I        = 0x20,     // PINASSIGN2 - UART1 CTS Input
            U1_SCLK_IO      = 0x21,     // PINASSIGN2 - UART1 SCLK I/O
            U2_TXD_O        = 0x22,     // PINASSIGN2 - UART2 TXD Output
            U2_RXD_I        = 0x23,     // PINASSIGN2 - UART2 RXD Input
            U2_RTS_O        = 0x30,     // PINASSIGN3 - UART2 RTS Output
            U2_CTS_I        = 0x31,     // PINASSIGN3 - UART2 CTS Input
            U2_SCLK_IO      = 0x32,     // PINASSIGN3 - UART2 SCLK I/O
            SPI0_SCK_IO     = 0x33,     // PINASSIGN3 - SPI0 SCK I/O
            SPI0_MOSI_IO    = 0x40,     // PINASSIGN4 - SPI0 MOSI I/O
            SPI0_MISO_IO    = 0x41,     // PINASSIGN4 - SPI0 MISO I/O
            SPI0_SSEL0_IO   = 0x42,     // PINASSIGN4 - SPI0 SSEL0 I/O
            SPI0_SSEL1_IO   = 0x43,     // PINASSIGN4 - SPI0 SSEL1 I/O
            SPI0_SSEL2_IO   = 0x50,     // PINASSIGN5 - SPI0 SSEL2 I/O
            SPI0_SSEL3_IO   = 0x51,     // PINASSIGN5 - SPI0 SSEL3 I/O
            SPI1_SCK_IO     = 0x52,     // PINASSIGN5 - SPI1 SCK I/O
            SPI1_MOSI_IO    = 0x53,     // PINASSIGN5 - SPI1 MOSI I/O
            SPI1_MISO_IO    = 0x60,     // PINASSIGN6 - SPI1 MISO I/O
            SPI1_SSEL0_IO   = 0x61,     // PINASSIGN6 - SPI1 SSEL0 I/O
            SPI1_SSEL1_IO   = 0x62,     // PINASSIGN6 - SPI1 SSEL1 I/O
            SCT_PIN0_I      = 0x63,     // PINASSIGN6 - SCT PIN0 Input
            SCT_PIN1_I      = 0x70,     // PINASSIGN7 - SCT PIN1 Input
            SCT_PIN2_I      = 0x71,     // PINASSIGN7 - SCT PIN2 Input
            SCT_PIN3_I      = 0x72,     // PINASSIGN7 - SCT PIN3 Input
            SCT_OUT0_O      = 0x73,     // PINASSIGN7 - SCT OUT0 Output
            SCT_OUT1_O      = 0x80,     // PINASSIGN8 - SCT OUT1 Output
            SCT_OUT2_O      = 0x81,     // PINASSIGN8 - SCT OUT2 Output
            SCT_OUT3_O      = 0x82,     // PINASSIGN8 - SCT OUT3 Output
            SCT_OUT4_O      = 0x83,     // PINASSIGN8 - SCT OUT4 Output
            SCT_OUT5_O      = 0x90,     // PINASSIGN9 - SCT OUT5 Output
            SCT_OUT6_O      = 0x91,     // PINASSIGN9 - SCT OUT6 Output
            I2C1_SDA_IO     = 0x92,     // PINASSIGN9 - I2C1 SDA I/O
            I2C1_SCL_IO     = 0x93,     // PINASSIGN9 - I2C1 SCL I/O
            I2C2_SDA_IO     = 0xA0,     // PINASSIGN10 - I2C2 SDA I/O
            I2C2_SCL_IO     = 0xA1,     // PINASSIGN10 - I2C2 SCL I/O
            I2C3_SDA_IO     = 0xA2,     // PINASSIGN10 - I2C3 SDA I/O
            I2C3_SCL_IO     = 0xA3,     // PINASSIGN10 - I2C3 SCL I/O
            ACMP_O_O        = 0xB0,     // PINASSIGN11 - Analog comparator Output
            CLKOUT_O        = 0xB1,     // PINASSIGN11 - CLKOUT Output
            GPIO_INT_BMAT_O = 0xB3,     // PINASSIGN11 - GPIO INT BMAT Output
            U3_TXD_O        = 0xB4,     // PINASSIGN11 - UART3 TXD Output
            U3_RXD_I        = 0xC0,     // PINASSIGN12 - UART3 RXD Input
            U3_SCK_IO       = 0xC1,     // PINASSIGN12 - UART3 SCLK I/O
            U4_TXD_O        = 0xC2,     // PINASSIGN12 - UART4 TXD Output
            U4_RXD_I        = 0xC3,     // PINASSIGN12 - UART4 RXD Input
            U4_SCK_IO       = 0xD0,     // PINASSIGN13 - UART4 SCLK I/O
            T0_MAT0_O       = 0xD1,     // PINASSIGN13 - Timer0 Match0 Output
            T0_MAT1_O       = 0xD2,     // PINASSIGN13 - Timer0 Match1 Output
            T0_MAT2_O       = 0xD3,     // PINASSIGN13 - Timer0 Match2 Output
            T0_MAT3_O       = 0xE0,     // PINASSIGN14 - Timer0 Match3 Output
            T0_CAP0_I       = 0xE1,     // PINASSIGN14 - Timer0 Capture0 Input
            T0_CAP1_I       = 0xE2,     // PINASSIGN14 - Timer0 Capture1 Input
            T0_CAP2_I       = 0xE3,     // PINASSIGN14 - Timer0 Capture2 Input
        };

        // Switch Matrix fixed pins
        enum class PinFixed
        {
            ACMP_I1 = 0,                // ACMP I1
            ACMP_I2,                    // ACMP I2
            ACMP_I3,                    // ACMP I3
            ACMP_I4,                    // ACMP I4
            ACMP_I5,                    // ACMP I5
            SWCLK,                      // SWCLK
            SWDIO,                      // SWDIO
            XTALIN,                     // XTALIN
            XTALOUT,                    // XTALOUT
            RESETN,                     // Reset
            CLKIN,                      // Clock Input
            VDDCMP,                     // VDDCMP
            I2C0_SDA,                   // I2C0_SDA
            I2C0_SCL,                   // I2C0_SCL
            ADC_0,                      // ADC_0
            ADC_1,                      // ADC_1
            ADC_2,                      // ADC_2
            ADC_3,                      // ADC_3
            ADC_4,                      // ADC_4
            ADC_5,                      // ADC_5
            ADC_6,                      // ADC_6
            ADC_7,                      // ADC_7
            ADC_8,                      // ADC_8
            ADC_9,                      // ADC_9
            ADC_10,                     // ADC_10
            ADC_11,                     // ADC_11
            DACOUT0,                    // DACOUT0
            DACOUT1,                    // DACOUT1
            CAPT_X0,                    // CAPT_X0
            CAPT_X1,                    // CAPT_X1
            CAPT_X2,                    // CAPT_X2
            CAPT_X3,                    // CAPT_X3

            CAPT_X4 = 32,               // CAPT_X4
            CAPT_X5,                    // CAPT_X5
            CAPT_X6,                    // CAPT_X6
            CAPT_X7,                    // CAPT_X7
            CAPT_X8,                    // CAPT_X8
            CAPT_YL,                    // CAPT YL
            CAPT_YH                     // CAPT YH
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
            if(static_cast<uint32_t>(fixed) < 32)
            {
                LPC_SWM->PINENABLE0 &= ~(1 << static_cast<uint32_t>(fixed));
            }
            else
            {
                LPC_SWM->PINENABLE1 &= ~(1 << (static_cast<uint32_t>(fixed) - 32));
            }
        }

        // Disable a fixed function pin in the Switch Matrix
        static void disable(const PinFixed fixed)
        {
            if(static_cast<uint32_t>(fixed) < 32)
            {
                LPC_SWM->PINENABLE0 |= (1 << static_cast<uint32_t>(fixed));
            }
            else
            {
                LPC_SWM->PINENABLE1 |= (1 << (static_cast<uint32_t>(fixed) - 32));
            }
        }
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SWM_HPP
