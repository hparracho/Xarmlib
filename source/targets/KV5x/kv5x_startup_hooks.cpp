// ----------------------------------------------------------------------------
// @file    kv5x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for Kinetis KV5x MCUs.
// @date    10 January 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#include "core/target_specs.hpp"

#ifdef __KV5X__

#include "xarmlib_config.hpp"
#include "targets/KV5x/kv5x_fsl_clock_config.h"
#include "fsl_pmc.h"
#include "fsl_wdog.h"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{

extern "C"
{




void mcu_startup_initialize_hardware_early()
{
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
    // Set CP10, CP11 Full Access
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));
#endif // ((__FPU_PRESENT == 1) && (__FPU_USED == 1))

    // Unlock and disable Watchdog
    WDOG_Unlock(WDOG);
    WDOG_Disable(WDOG);

    // Enable instruction and data caches
#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
      SCB_EnableICache();
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
      SCB_EnableDCache();
#endif
}




void mcu_startup_initialize_hardware()
{
//    const pmc_low_volt_detect_config_t lvd_config =
//    {
//        false,                      // Interrupt disabled
//        true,                       // System reset enabled
//        kPMC_LowVoltDetectHighTrip  // LVDH: 2.48 ~ 2.64 (typ. 2.56) V
//    };
//
//    PMC_ConfigureLowVoltDetect(PMC, &lvd_config);
//
//    switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
//    {
//        case SystemDriver::Clock::irc_4mhz_vlpr:     clock_config_irc_4mhz_vlpr();     break;
//        case SystemDriver::Clock::xtal_80mhz_run:    clock_config_xtal_80mhz_run();    break;
//        case SystemDriver::Clock::xtal_160mhz_hsrun: clock_config_xtal_160mhz_hsrun(); break;
//        case SystemDriver::Clock::irc_96mhz_run:
//        default:                                     clock_config_irc_96mhz_run();     break;
//    }
}




} // extern "C"

} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __KV5X__
