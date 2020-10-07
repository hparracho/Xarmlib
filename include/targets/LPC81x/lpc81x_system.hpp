// ----------------------------------------------------------------------------
// @file    lpc81x_system.hpp
// @brief   NXP LPC81x system level configuration class.
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

#ifndef XARMLIB_TARGETS_LPC81X_SYSTEM_HPP
#define XARMLIB_TARGETS_LPC81X_SYSTEM_HPP

#include "targets/LPC81x/lpc81x_specs.hpp"
#include "hal/hal_system_base.hpp"




namespace xarmlib::targets::lpc81x
{

struct SystemTraits
{
    // 12 MHz internal RC oscillator
    static constexpr int32_t k_irc_freq   = 12000000;
    // Crystal frequency (12 MHz fixed)
    static constexpr int32_t k_xtal_freq  = 12000000;
    // External clock pin input frequency (currently not implemented)
    static constexpr int32_t k_clkin_freq = 0;

    // Possible clock frequencies selection
    enum class Clock : int32_t
    {
        osc_12mhz  = 12000000,      // Using direct 12 MHz internal RC oscillator
        osc_24mhz  = 24000000,      // Using 12 MHz internal RC oscillator and PLL
        osc_30mhz  = 30000000,      // Using 12 MHz internal RC oscillator and PLL

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
        // The use of an external crystal is not possible in DIP8 packages
        // NOTE: Crystal frequency set fixed to 12MHz
        xtal_12mhz = 12000000,      // Using direct external crystal
        xtal_24mhz = 24000000,      // Using external crystal and PPL
        xtal_30mhz = 30000000       // Using external crystal and PPL

        // External clock pin input frequency currently not implemented
#endif
    };

    static constexpr int32_t get_core_clock_frequency(const Clock clock)
    {
        return static_cast<int32_t>(clock);
    }

    static constexpr int32_t get_main_clock_frequency(const Clock clock)
    {
        const auto core_clock_freq = get_core_clock_frequency(clock);

        return (core_clock_freq == 30000000) ? 60000000 : core_clock_freq;
    }
};

using System = hal::SystemBase<SystemTraits>;

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_SYSTEM_HPP
