// ----------------------------------------------------------------------------
// @file    lpc81x_us_ticker.hpp
// @brief   NXP LPC81x Ticker class (microsecond resolution).
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

#ifndef XARMLIB_TARGETS_LPC81X_US_TICKER_HPP
#define XARMLIB_TARGETS_LPC81X_US_TICKER_HPP

#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_us_ticker_base.hpp"




namespace xarmlib::targets::lpc81x
{

class UsTicker : public hal::UsTickerBase<UsTicker>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static uint32_t read()
    {
        return LPC_SCT->COUNT;
    }

    // This function initializes the microsecond ticker. It is called
    // at the final stage the of hardware initialization function
    // 'mcu_startup_initialize_hardware()' right before 'main()' is invoked.
    static void initialize()
    {
        // Enable and reset the SCT clock
        SysClock::enable(SysClock::Peripheral::sct);
        Power::reset(Power::ResetPeripheral::sct);

        // Unified counter (32 bits)
        LPC_SCT->CONFIG |= 1;

        // Halt and clear the counter
        LPC_SCT->CTRL |= (1 << 2) | (1 << 3);

        // System Clock -> us_ticker 1MHz
        LPC_SCT->CTRL |= ((SystemCoreClock / 1000000 - 1) << 5);

        // Unhalt the counter - clearing bit 2 of the CTRL register
        LPC_SCT->CTRL &= ~(1 << 2);
    }
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_US_TICKER_HPP
