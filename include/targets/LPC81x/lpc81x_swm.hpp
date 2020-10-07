// ----------------------------------------------------------------------------
// @file    lpc81x_swm.hpp
// @brief   NXP LPC81x Switch Matrix (SWM) class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_TARGETS_LPC81X_SWM_HPP
#define XARMLIB_TARGETS_LPC81X_SWM_HPP

#include "targets/LPC81x/lpc81x_pin.hpp"




namespace xarmlib::targets::lpc81x
{

class Swm
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Switch Matrix movable pins
    enum class PinMovable
    {
        u0_txd_o        = 0x00, // PINASSIGN0 - UART0 TXD Output
        u0_rxd_i        = 0x01, // PINASSIGN0 - UART0 RXD Input
        u0_rts_o        = 0x02, // PINASSIGN0 - UART0 RTS Output
        u0_cts_i        = 0x03, // PINASSIGN0 - UART0 CTS Input
        u0_sclk_io      = 0x10, // PINASSIGN1 - UART0 SCLK I/O
        u1_txd_o        = 0x11, // PINASSIGN1 - UART1 TXD Output
        u1_rxd_i        = 0x12, // PINASSIGN1 - UART1 RXD Input
        u1_rts_o        = 0x13, // PINASSIGN1 - UART1 RTS Output
        u1_cts_i        = 0x20, // PINASSIGN2 - UART1 CTS Input
        u1_sclk_io      = 0x21, // PINASSIGN2 - UART1 SCLK I/O
        u2_txd_o        = 0x22, // PINASSIGN2 - UART2 TXD Output
        U2_RXD_I        = 0x23, // PINASSIGN2 - UART2 RXD Input
        u2_rts_o        = 0x30, // PINASSIGN3 - UART2 RTS Output
        u2_cts_i        = 0x31, // PINASSIGN3 - UART2 CTS Input
        u2_sclk_io      = 0x32, // PINASSIGN3 - UART2 SCLK I/O
        spi0_sck_io     = 0x33, // PINASSIGN3 - SPI0 SCK I/O
        spi0_mosi_io    = 0x40, // PINASSIGN4 - SPI0 MOSI I/O
        spi0_miso_io    = 0x41, // PINASSIGN4 - SPI0 MISO I/O
        spi0_ssel_io    = 0x42, // PINASSIGN4 - SPI0 SSEL I/O
        spi1_sck_io     = 0x43, // PINASSIGN4 - SPI1 SCK I/O
        spi1_mosi_io    = 0x50, // PINASSIGN5 - SPI1 MOSI I/O
        spi1_miso_io    = 0x51, // PINASSIGN5 - SPI1 MISO I/O
        spi1_ssel_io    = 0x52, // PINASSIGN5 - SPI1 SSEL I/O
        sct_pin0_i      = 0x53, // PINASSIGN5 - SCT PIN0 Input
        sct_pin1_i      = 0x60, // PINASSIGN6 - SCT PIN1 Input
        sct_pin2_i      = 0x61, // PINASSIGN6 - SCT PIN2 Input
        sct_pin3_i      = 0x62, // PINASSIGN6 - SCT PIN3 Input
        sct_out0_o      = 0x63, // PINASSIGN6 - SCT OUT0 Output
        sct_out1_o      = 0x70, // PINASSIGN7 - SCT OUT1 Output
        sct_out2_o      = 0x71, // PINASSIGN7 - SCT OUT2 Output
        sct_out3_o      = 0x72, // PINASSIGN7 - SCT OUT3 Output
        i2c_sda_io      = 0x73, // PINASSIGN7 - I2C SDA I/O
        i2c_scl_io      = 0x80, // PINASSIGN8 - I2C SCL I/O
        acmp_o_o        = 0x81, // PINASSIGN8 - Analog comparator Output
        clkout_o        = 0x82, // PINASSIGN8 - CLKOUT Output
        gpio_int_bmat_o = 0x83  // PINASSIGN8 - GPIO INT BMAT Output
    };

    // Switch Matrix fixed pins
    enum class PinFixed
    {
        acmp_i1 = 0,            // ACMP I1
        acmp_i2 = 1,            // ACMP I2
        swclk   = 2,            // SWCLK
        swdio   = 3,            // SWDIO
        xtalin  = 4,            // XTALIN
        xtalout = 5,            // XTALOUT
        rst     = 6,            // Reset
        clkin   = 7,            // Clock Input
        vddcmp  = 8             // VDD
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Assign a movable pin function to a physical pin in Switch Matrix
    static void assign(const PinMovable movable, const Pin::Name pin_name)
    {
        if(pin_name != Pin::Name::nc)
        {
            const  int32_t pin_index = static_cast<int32_t>(movable) >> 4;
            const uint32_t pin_shift = (static_cast<uint32_t>(movable) & 0x0F) << 3;
            const uint32_t reg_value = LPC_SWM->PINASSIGN[pin_index] & (~(0xFF << pin_shift));

            LPC_SWM->PINASSIGN[pin_index] = reg_value | (static_cast<uint32_t>(pin_name) << pin_shift);
        }
    }

    // Unassign a movable pin function from a physical pin in Switch Matrix
    static void unassign(const PinMovable movable)
    {
        const  int32_t pin_index = static_cast<int32_t>(movable) >> 4;
        const uint32_t pin_shift = (static_cast<uint32_t>(movable) & 0x0F) << 3;

        LPC_SWM->PINASSIGN[pin_index] |= (0xFF << pin_shift);
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

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_SWM_HPP
