// ----------------------------------------------------------------------------
// @file    lpc81x_cmsis.cpp
// @brief   CMSIS Core Peripheral Access Layer source file for NXP LPC81x MCUs.
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

#include "xarmlib_config.hpp"
#include "core/target_specs.hpp"

#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_syscon_clock.hpp"




extern "C"
{

// CMSIS system core clock variable is directly loaded from Xarmlib's configuration constant
uint32_t SystemCoreClock = xarmlib::targets::lpc81x::System::
                           get_core_clock_frequency(xarmlib::XARMLIB_CONFIG_SYSTEM_CLOCK);




// Update system core clock frequency
// NOTE: This function should be called every time the system has a clock frequency change.
void SystemCoreClockUpdate()
{
    // Store the clock frequency in the SystemCoreClock global RAM location
    SystemCoreClock = xarmlib::targets::lpc81x::SysClock::get_system_clock_frequency();
}

} // extern "C"




#endif // defined(__LPC81X__)
