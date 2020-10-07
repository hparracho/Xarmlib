// ----------------------------------------------------------------------------
// @file    lpc81x_watchdog.hpp
// @brief   NXP LPC81x Watchdog class.
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

#ifndef XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP
#define XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP

#include "core/chrono.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_watchdog_base.hpp"




namespace xarmlib::targets::lpc81x
{

class Watchdog : public hal::WatchdogBase<Watchdog>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    template<int64_t duration_value, typename Duration = std::chrono::microseconds>
    static void start()
    {
        static_assert(chrono::is_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

        assert(s_initialized == false);
        s_initialized = true;

#ifndef NDEBUG

        Power::power_up(Power::Peripheral::wdtosc);
        SysClock::enable(SysClock::Peripheral::wwdt);

        // Disable watchdog
        LPC_WWDT->MOD     = 0;
        LPC_WWDT->WARNINT = 0x00;
        LPC_WWDT->WINDOW  = 0xFFFFFF;

        constexpr auto wdt_config = SysClock::get_watchdog_osc_config<duration_value, Duration>();

        SysClock::set_watchdog_osc_frequency(wdt_config.frequency, wdt_config.divider);

        LPC_WWDT->TC  = wdt_config.counter;
        LPC_WWDT->MOD = mod_wden | mod_wdreset | mod_wdprotect | mod_lock;

        reset();
#endif
    }

    // Feed the watchdog timer
    static void reset()
    {
        assert(s_initialized == true);
#ifndef NDEBUG
        // Don't want anything else to happen between feeds (like another feed)

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        LPC_WWDT->FEED = 0xAA;
        LPC_WWDT->FEED = 0x55;

        __set_PRIMASK(primask);
#endif
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // ------------------------------------------------------------------------

    // Windowed Watchdog MOD Register bits
    enum MOD
    {
        mod_wden      = (1 << 0),       // WWDT enable bit
        mod_wdreset   = (1 << 1),       // WWDT reset enable bit
        mod_wdtof     = (1 << 2),       // WWDT time-out flag bit
        mod_wdint     = (1 << 3),       // WWDT warning interrupt flag bit
        mod_wdprotect = (1 << 4),       // WWDT update mode bit
        mod_lock      = (1 << 5)        // WWDT osc lock mode bit
    };

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    inline static bool s_initialized {false};
};

} // namespace xarmlib::targets::lpc81x




#endif  // XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP
