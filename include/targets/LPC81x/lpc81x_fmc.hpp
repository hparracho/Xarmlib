// ----------------------------------------------------------------------------
// @file    lpc81x_fmc.hpp
// @brief   NXP LPC81x Flash Memory Controller (FMC) class.
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

#ifndef XARMLIB_TARGETS_LPC81X_FMC_HPP
#define XARMLIB_TARGETS_LPC81X_FMC_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"




namespace xarmlib::targets::lpc81x
{

class Fmc
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Flash access time
    enum class AccessTime
    {
        time_1_sysclk = 0,      // Flash accesses use 1 system clock (valid for up to 20 MHz CPU clock)
        time_2_sysclk,          // Flash accesses use 2 system clocks (valid for up to 30 MHz CPU clock)
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Set flash memory access time in clocks
    static void set_access_time(const AccessTime clocks)
    {
        LPC_FMC->FLASHCFG = (LPC_FMC->FLASHCFG & (~0x03))
                          | static_cast<uint32_t>(clocks);
    }
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_FMC_HPP
