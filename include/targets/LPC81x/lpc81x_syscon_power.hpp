// ----------------------------------------------------------------------------
// @file    lpc81x_syscon_power.hpp
// @brief   NXP LPC81x SYSCON power control / brown-out classes.
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

#ifndef XARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP
#define XARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"




namespace xarmlib::targets::lpc81x
{

class Power
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Power configuration
    enum class Peripheral
    {
        // PDRUNCFG, Power configuration register
        ircout = 0,
        irc    = 1,
        flash  = 2,
        bod    = 3,
        sysosc = 5,
        wdtosc = 6,
        syspll = 7,
        acmp   = 15
    };

    // Peripheral reset
    enum class ResetPeripheral
    {
        spi0 = 0,
        spi1,
        usartfrg,
        usart0,
        usart1,
        usart2,
        i2c,
        mrt,
        sct,
        wkt,
        gpio,
        flash,
        acmp
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Powers up a block
    static void power_up(const Peripheral peripheral)
    {
        // Apply power to the block by setting low
        LPC_SYSCON->PDRUNCFG &= ~(1 << static_cast<uint32_t>(peripheral));
    }

    // Powers down a block
    static void power_down(const Peripheral peripheral)
    {
        // Disable the block by setting high
        LPC_SYSCON->PDRUNCFG |= (1 << static_cast<uint32_t>(peripheral));
    }

    // Resets a peripheral
    static void reset(const ResetPeripheral peripheral)
    {
        // Assert reset for a peripheral (the peripheral will stay in reset until reset is de-asserted)
        LPC_SYSCON->PRESETCTRL &= ~(1 << static_cast<uint32_t>(peripheral));
        // De-assert reset for a peripheral
        LPC_SYSCON->PRESETCTRL |=  (1 << static_cast<uint32_t>(peripheral));
    }
};




class BrownOut
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Brown-out detector reset or interrupt level
    enum class Level
    {
     // RESERVED        // Brown-out reset at 1.46V ~ 1.63V / interrupt at 1.65V ~ 1.80V (RESERVED?)
        level_1 = 1,    // Brown-out reset at 2.06V ~ 2.15V / interrupt at 2.22V ~ 2.35V
        level_2,        // Brown-out reset at 2.35V ~ 2.43V / interrupt at 2.52V ~ 2.66V
        level_3,        // Brown-out reset at 2.63V ~ 2.71V / interrupt at 2.80V ~ 2.90V
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Sets brown-out detection reset level and enables it
    static void enable_reset(const Level level)
    {
        // BOD reset enable (BODRSTENA) = (1 << 4)
        LPC_SYSCON->BODCTRL |= static_cast<uint32_t>(level) | (1 << 4);
    }

    // Disables brown-out detection reset
    static void disable_reset()
    {
        // BOD reset enable (BODRSTENA) = (1 << 4)
        LPC_SYSCON->BODCTRL &= ~(1 << 4);
    }
};

} // namespace xarmlib::targets::lpc81x




#endif // ARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP
