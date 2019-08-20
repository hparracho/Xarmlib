// ----------------------------------------------------------------------------
// @file    lpc84x_swm.hpp
// @brief   NXP LPC84x Switch Matrix (SWM) class.
// @date    9 April 2019
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

#include "targets/LPC84x/lpc84x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class SwmDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Switch Matrix movable pins
        enum class PinMovable
        {
            u0_txd_o        = 0x00,     // PINASSIGN0 - UART0 TXD Output
            u0_rxd_i        = 0x01,     // PINASSIGN0 - UART0 RXD Input
            u0_rts_o        = 0x02,     // PINASSIGN0 - UART0 RTS Output
            u0_cts_i        = 0x03,     // PINASSIGN0 - UART0 CTS Input
            u0_sclk_io      = 0x10,     // PINASSIGN1 - UART0 SCLK I/O
            u1_txd_o        = 0x11,     // PINASSIGN1 - UART1 TXD Output
            u1_rxd_i        = 0x12,     // PINASSIGN1 - UART1 RXD Input
            u1_rts_o        = 0x13,     // PINASSIGN1 - UART1 RTS Output
            u1_cts_i        = 0x20,     // PINASSIGN2 - UART1 CTS Input
            u1_sclk_io      = 0x21,     // PINASSIGN2 - UART1 SCLK I/O
            u2_txd_o        = 0x22,     // PINASSIGN2 - UART2 TXD Output
            u2_rxd_i        = 0x23,     // PINASSIGN2 - UART2 RXD Input
            u2_rts_o        = 0x30,     // PINASSIGN3 - UART2 RTS Output
            u2_cts_i        = 0x31,     // PINASSIGN3 - UART2 CTS Input
            u2_sclk_io      = 0x32,     // PINASSIGN3 - UART2 SCLK I/O
            spi0_sck_io     = 0x33,     // PINASSIGN3 - SPI0 SCK I/O
            spi0_mosi_io    = 0x40,     // PINASSIGN4 - SPI0 MOSI I/O
            spi0_miso_io    = 0x41,     // PINASSIGN4 - SPI0 MISO I/O
            spi0_ssel0_io   = 0x42,     // PINASSIGN4 - SPI0 SSEL0 I/O
            spi0_ssel1_io   = 0x43,     // PINASSIGN4 - SPI0 SSEL1 I/O
            spi0_ssel2_io   = 0x50,     // PINASSIGN5 - SPI0 SSEL2 I/O
            spi0_ssel3_io   = 0x51,     // PINASSIGN5 - SPI0 SSEL3 I/O
            spi1_sck_io     = 0x52,     // PINASSIGN5 - SPI1 SCK I/O
            spi1_mosi_io    = 0x53,     // PINASSIGN5 - SPI1 MOSI I/O
            spi1_miso_io    = 0x60,     // PINASSIGN6 - SPI1 MISO I/O
            spi1_ssel0_io   = 0x61,     // PINASSIGN6 - SPI1 SSEL0 I/O
            spi1_ssel1_io   = 0x62,     // PINASSIGN6 - SPI1 SSEL1 I/O
            sct_pin0_i      = 0x63,     // PINASSIGN6 - SCT PIN0 Input
            sct_pin1_i      = 0x70,     // PINASSIGN7 - SCT PIN1 Input
            sct_pin2_i      = 0x71,     // PINASSIGN7 - SCT PIN2 Input
            sct_pin3_i      = 0x72,     // PINASSIGN7 - SCT PIN3 Input
            sct_out0_o      = 0x73,     // PINASSIGN7 - SCT OUT0 Output
            sct_out1_o      = 0x80,     // PINASSIGN8 - SCT OUT1 Output
            sct_out2_o      = 0x81,     // PINASSIGN8 - SCT OUT2 Output
            sct_out3_o      = 0x82,     // PINASSIGN8 - SCT OUT3 Output
            sct_out4_o      = 0x83,     // PINASSIGN8 - SCT OUT4 Output
            sct_out5_o      = 0x90,     // PINASSIGN9 - SCT OUT5 Output
            sct_out6_o      = 0x91,     // PINASSIGN9 - SCT OUT6 Output
            i2c1_sda_io     = 0x92,     // PINASSIGN9 - I2C1 SDA I/O
            i2c1_scl_io     = 0x93,     // PINASSIGN9 - I2C1 SCL I/O
            i2c2_sda_io     = 0xA0,     // PINASSIGN10 - I2C2 SDA I/O
            i2c2_scl_io     = 0xA1,     // PINASSIGN10 - I2C2 SCL I/O
            i2c3_sda_io     = 0xA2,     // PINASSIGN10 - I2C3 SDA I/O
            i2c3_scl_io     = 0xA3,     // PINASSIGN10 - I2C3 SCL I/O
            acmp_o_o        = 0xB0,     // PINASSIGN11 - Analog comparator Output
            clkout_o        = 0xB1,     // PINASSIGN11 - CLKOUT Output
            gpio_int_bmat_o = 0xB3,     // PINASSIGN11 - GPIO INT BMAT Output
            u3_txd_o        = 0xB4,     // PINASSIGN11 - UART3 TXD Output
            u3_rxd_i        = 0xC0,     // PINASSIGN12 - UART3 RXD Input
            u3_sck_io       = 0xC1,     // PINASSIGN12 - UART3 SCLK I/O
            u4_txd_o        = 0xC2,     // PINASSIGN12 - UART4 TXD Output
            u4_rxd_i        = 0xC3,     // PINASSIGN12 - UART4 RXD Input
            u4_sck_io       = 0xD0,     // PINASSIGN13 - UART4 SCLK I/O
            t0_mat0_o       = 0xD1,     // PINASSIGN13 - Timer0 Match0 Output
            t0_mat1_o       = 0xD2,     // PINASSIGN13 - Timer0 Match1 Output
            t0_mat2_o       = 0xD3,     // PINASSIGN13 - Timer0 Match2 Output
            t0_mat3_o       = 0xE0,     // PINASSIGN14 - Timer0 Match3 Output
            t0_cap0_i       = 0xE1,     // PINASSIGN14 - Timer0 Capture0 Input
            t0_cap1_i       = 0xE2,     // PINASSIGN14 - Timer0 Capture1 Input
            t0_cap2_i       = 0xE3,     // PINASSIGN14 - Timer0 Capture2 Input
        };

        // Switch Matrix fixed pins
        enum class PinFixed
        {
            acmp_i1 = 0,                // ACMP I1
            acmp_i2,                    // ACMP I2
            acmp_i3,                    // ACMP I3
            acmp_i4,                    // ACMP I4
            acmp_i5,                    // ACMP I5
            swclk,                      // SWCLK
            swdio,                      // SWDIO
            xtalin,                     // XTALIN
            xtalout,                    // XTALOUT
            resetn,                     // Reset
            clkin,                      // Clock Input
            vddcmp,                     // VDDCMP
            i2c0_sda,                   // I2C0_SDA
            i2c0_scl,                   // I2C0_SCL
            adc_0,                      // ADC_0
            adc_1,                      // ADC_1
            adc_2,                      // ADC_2
            adc_3,                      // ADC_3
            adc_4,                      // ADC_4
            adc_5,                      // ADC_5
            adc_6,                      // ADC_6
            adc_7,                      // ADC_7
            adc_8,                      // ADC_8
            adc_9,                      // ADC_9
            adc_10,                     // ADC_10
            adc_11,                     // ADC_11
            dacout0,                    // DACOUT0
            dacout1,                    // DACOUT1
            capt_x0,                    // CAPT_X0
            capt_x1,                    // CAPT_X1
            capt_x2,                    // CAPT_X2
            capt_x3,                    // CAPT_X3

            capt_x4 = 32,               // CAPT_X4
            capt_x5,                    // CAPT_X5
            capt_x6,                    // CAPT_X6
            capt_x7,                    // CAPT_X7
            capt_x8,                    // CAPT_X8
            capt_yl,                    // CAPT YL
            capt_yh                     // CAPT YH
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Assign a movable pin function to a physical pin in Switch Matrix
        static void assign(const PinMovable movable, const PinDriver::Name pin)
        {
            if(pin == PinDriver::Name::nc)
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
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SWM_HPP
