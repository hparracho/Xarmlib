// ----------------------------------------------------------------------------
// @file    lpc81x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC81x MCU.
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

#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "targets/LPC81x/lpc81x_us_ticker.hpp"




namespace xarmlib::targets::lpc81x
{

extern "C"
{

// ----------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------------

static void mcu_startup_set_irc_clock()
{
    if constexpr(XARMLIB_CONFIG_SYSTEM_CLOCK == System::Clock::osc_12mhz)
    {
        // Set the main clock divide by 1
        SysClock::set_system_clock_divider(1);

        // Set main clock source directly to the IRC oscillator
        SysClock::set_main_clock_source(SysClock::MainClockSource::irc);
    }
    else
    {
        // Set IRC oscillator source for system PLL clock select
        SysClock::set_system_pll_source(SysClock::SystemPllSource::irc);

        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
        {
            case System::Clock::osc_30mhz: SysClock::set_system_pll_divider(4, 1); // 30 MHz => M=5; P=2; DIV=2
                                           SysClock::set_system_clock_divider(2);  // Divide the main_clock by 2
                                           break;
            case System::Clock::osc_24mhz:
            default:                       SysClock::set_system_pll_divider(1, 2); // 24 MHz => M=2; P=3; DIV=1
                                           SysClock::set_system_clock_divider(1);  // Divide the main_clock by 1
                                           break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        Power::power_up(Power::Peripheral::syspll);

        // Wait for the system PLL to lock
        SysClock::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        SysClock::set_main_clock_source(SysClock::MainClockSource::sys_pll_out_clk);
    }
}




#if (TARGET_PACKAGE_PIN_COUNT > 8)

// The use of an external crystal is not possible in DIP8 packages
static void mcu_startup_set_xtal_clock()
{
    // Disable pull-up and pull-down for XTALIN and XTALOUT pin
    Pin::set_mode(Pin::Name::p0_8, Pin::FunctionMode::hiz);
    Pin::set_mode(Pin::Name::p0_9, Pin::FunctionMode::hiz);

    // Use Switch Matrix to enable XTALIN/XTALOUT functions
    Swm::enable(Swm::PinFixed::xtalin);
    Swm::enable(Swm::PinFixed::xtalout);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    SysClock::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    Power::power_up(Power::Peripheral::sysosc);

    // Wait 500 us for system oscillator to stabilize (typical time from datasheet).
    // The for loop takes 7 clocks per iteration and executes at a maximum of 30 MHz
    // (33.333 ns), so worst case: i = (500 us) / (7 * 33.333 ns) = 2142.9 => 2143
    for(uint32_t i = 0; i < 2143; i++) { __NOP(); }

    // Set system oscillator source for system PLL clock select
    SysClock::set_system_pll_source(SysClock::SystemPllSource::sys_osc_clk);

    if constexpr (XARMLIB_CONFIG_SYSTEM_CLOCK == System::Clock::xtal_12mhz)
    {
        // Set the main clock divide by 1
        SysClock::set_system_clock_divider(1);

        // Set system PLL in source for main clock select
        SysClock::set_main_clock_source(SysClock::MainClockSource::sys_pll_in_clk);
    }
    else
    {
        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
        {
            case System::Clock::xtal_30mhz: SysClock::set_system_pll_divider(4, 1); // 30 MHz => M=5; P=2; DIV=2
                                            SysClock::set_system_clock_divider(2);  // Divide the main_clock by 2
                                            break;
            case System::Clock::xtal_24mhz:
            default:                        SysClock::set_system_pll_divider(1, 2); // 24 MHz => M=2; P=3; DIV=1
                                            SysClock::set_system_clock_divider(1);  // Divide the main_clock by 1
                                            break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        Power::power_up(Power::Peripheral::syspll);

        // Wait for the system PLL to lock
        SysClock::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        SysClock::set_main_clock_source(SysClock::MainClockSource::sys_pll_out_clk);
    }

#if defined (NDEBUG) && defined (XARMLIB_CONFIG_CRP_SETTING) && (XARMLIB_CONFIG_CRP_SETTING != CRP_NO_CRP)
    // Disable the unused IRC oscillator when compiling a final application
    Power::power_down(Power::Peripheral::irc);
    Power::power_down(Power::Peripheral::ircout);
#endif
}

#else // !(TARGET_PACKAGE_PIN_COUNT > 8)

static void mcu_startup_set_open_drain_pins_as_output_low()
{
    const uint32_t open_drain_pins_mask = (1UL << 10) | (1UL << 11);

    // Set bits 10 and 11 in the GPIO DIR0 register to 1 to enable the output driver
    // and write 1 to bits 10 and 11 in the GPIO CLR0 register to drive the outputs LOW internally.
    LPC_GPIO->DIR0 |= open_drain_pins_mask;
    LPC_GPIO->CLR0 |= open_drain_pins_mask;
}

#endif // !(TARGET_PACKAGE_PIN_COUNT > 8)




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

void mcu_startup_initialize_hardware_early()
{}




void mcu_startup_initialize_hardware()
{
    // ------------------------------------------------------------------------
    // Emanuel Pinto @ 21 June 2018
    // NOTE: The errata sheet advises that the required minimum wait time
    //       of the power supply on the VDD pin specification is 2 ms.

    // Wait 2 ms before ramping up.
    // The for loop takes 7 clocks per iteration and executes at a maximum of 30 MHz
    // (33.333 ns), so worst case: i = (2 ms) / (7 * 33.333 ns) = 8571.5 => 8572
    for(uint32_t i = 0; i < 8572; i++) { __NOP(); }
    // ------------------------------------------------------------------------

    // Enable brown-out detection with reset level 3 (2.63V ~ 2.71V)
    BrownOut::enable_reset(BrownOut::Level::level_3);

    // Enable Switch Matrix clock
    SysClock::enable(SysClock::Peripheral::swm);
    // Enable IOCON clock
    SysClock::enable(SysClock::Peripheral::iocon);

#ifdef NDEBUG
    // DISABLE SWD WHEN COMPILING IN RELEASE!!!
    // NOTE: The boot loader assigns the SWD functions to pins PIO0_2 and PIO0_3.
    //       If the user code disables the SWD functions through the switch matrix
    //       to use the pins for other functions, the SWD port is disabled.
    Swm::disable(Swm::PinFixed::swclk);
    Swm::disable(Swm::PinFixed::swdio);
#endif

#if (TARGET_PACKAGE_PIN_COUNT > 8)
    if constexpr(XARMLIB_CONFIG_SYSTEM_CLOCK <= System::Clock::osc_30mhz)
    {
        mcu_startup_set_irc_clock();
    }
    else
    {
        mcu_startup_set_xtal_clock();
    }
#else // !(TARGET_PACKAGE_PIN_COUNT > 8)
    // The use of an external crystal is not possible in DIP8 packages
    mcu_startup_set_irc_clock();

    // NOTE from manual: If the open-drain pins PIO0_10 and PIO0_11 are not
    // available on the package, prevent the pins from internally floating.
    mcu_startup_set_open_drain_pins_as_output_low();
#endif // !(TARGET_PACKAGE_PIN_COUNT > 8)

    // ------------------------------------------------------------------------
    // Helder Parracho @ 28 September 2020
    // --- DEPRECATED---
    //
    // Call the CSMSIS system clock routine to store the clock
    // frequency in the SystemCoreClock global RAM location.
    // SystemCoreClockUpdate();
    //
    // ------------------------------------------------------------------------
    // SystemCoreClock variable is directly loaded from Xarmlib's configuration
    // constant. If later the clock frequency is changed by the application
    // it is still required to call 'SystemCoreClockUpdate()' to update the variable.
    // ------------------------------------------------------------------------

    // Helder Parracho @ 1 October 2020 - Since it is hard to find a firmware
    // where ticker isn't needed, decided to initialize it at startup.
    UsTicker::initialize();
}

} // extern "C"

} // namespace xarmlib::targets::lpc81x




#endif // defined(__LPC81X__)
