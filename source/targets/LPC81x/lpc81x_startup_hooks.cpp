// ----------------------------------------------------------------------------
// @file    lpc81x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC81x MCU.
// @date    4 March 2019
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

#include "core/target_specs.hpp"

#ifdef __LPC81X__

#include "xarmlib_config.hpp"
#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




extern "C"
{




// ----------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------------

static inline void mcu_startup_set_irc_clock()
{
    if(XARMLIB_CONFIG_SYSTEM_CLOCK == SystemDriver::Clock::osc_12mhz)
    {
        // Set the main clock divide by 1
        ClockDriver::set_system_clock_divider(1);

        // Set main clock source directly to the IRC oscillator
        ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::irc);
    }
    else
    {
        // Set IRC oscillator source for system PLL clock select
        ClockDriver::set_system_pll_source(ClockDriver::SystemPllSource::irc);

        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
        {
            case SystemDriver::Clock::osc_30mhz: ClockDriver::set_system_pll_divider(4, 1); // 30 MHz => M=5; P=2; DIV=2
                                                 ClockDriver::set_system_clock_divider(2);  // Divide the main_clock by 2
                                                 break;
            case SystemDriver::Clock::osc_24mhz:
            default:                             ClockDriver::set_system_pll_divider(1, 2); // 24 MHz => M=2; P=3; DIV=1
                                                 ClockDriver::set_system_clock_divider(1);  // Divide the main_clock by 1
                                                 break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        PowerDriver::power_up(PowerDriver::Peripheral::syspll);

        // Wait for the system PLL to lock
        ClockDriver::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::sys_pll_out_clk);
    }
}




#if (TARGET_PACKAGE_PIN_COUNT > 8)

// The use of an external crystal is not possible in DIP8 packages
static inline void mcu_startup_set_xtal_clock()
{
    // Disable pull-up and pull-down for XTALIN and XTALOUT pin
    PinDriver::set_mode(PinDriver::Name::p0_8, PinDriver::FunctionMode::hiz);
    PinDriver::set_mode(PinDriver::Name::p0_9, PinDriver::FunctionMode::hiz);

    // Use Switch Matrix to enable XTALIN/XTALOUT functions
    SwmDriver::enable(SwmDriver::PinFixed::xtalin);
    SwmDriver::enable(SwmDriver::PinFixed::xtalout);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    ClockDriver::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    PowerDriver::power_up(PowerDriver::Peripheral::sysosc);

    // Wait 500 us for system oscillator to stabilize (typical time from datasheet). The for
    // loop takes 7 clocks per iteration and executes at a maximum of 30 MHz (33.333 ns),
    // so worst case: i = (500 us) / (7 * 33.333 ns) = 2142.9 => 2143
    for(uint32_t i = 0; i < 2143; i++) __NOP();

    // Set system oscillator source for system PLL clock select
    ClockDriver::set_system_pll_source(ClockDriver::SystemPllSource::sys_osc_clk);

    if(XARMLIB_CONFIG_SYSTEM_CLOCK == SystemDriver::Clock::xtal_12mhz)
    {
        // Set the main clock divide by 1
        ClockDriver::set_system_clock_divider(1);

        // Set system PLL in source for main clock select
        ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::sys_pll_in_clk);
    }
    else
    {
        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
        {
            case SystemDriver::Clock::xtal_30mhz: ClockDriver::set_system_pll_divider(4, 1); // 30 MHz => M=5; P=2; DIV=2
                                                  ClockDriver::set_system_clock_divider(2);  // Divide the main_clock by 2
                                                  break;
            case SystemDriver::Clock::xtal_24mhz:
            default:                              ClockDriver::set_system_pll_divider(1, 2); // 24 MHz => M=2; P=3; DIV=1
                                                  ClockDriver::set_system_clock_divider(1);  // Divide the main_clock by 1
                                                  break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        PowerDriver::power_up(PowerDriver::Peripheral::syspll);

        // Wait for the system PLL to lock
        ClockDriver::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::sys_pll_out_clk);
    }

#if defined (NDEBUG) && defined (XARMLIB_CONFIG_CRP_SETTING) && (XARMLIB_CONFIG_CRP_SETTING != CRP_NO_CRP)
    // Disable the unused IRC oscillator when compiling a final application
    PowerDriver::power_down(PowerDriver::Peripheral::irc);
    PowerDriver::power_down(PowerDriver::Peripheral::ircout);
#endif
}

#else // !(TARGET_PACKAGE_PIN_COUNT > 8)

static inline void mcu_startup_set_open_drain_pins_as_output_low()
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
    for(uint32_t i = 0; i < 8572; i++) __NOP();
    // ------------------------------------------------------------------------

    // Enable brown-out detection with reset level 3 (2.63V ~ 2.71V)
    BrownOutDriver::enable_reset(BrownOutDriver::Level::level_3);

    // Enable Switch Matrix clock
    ClockDriver::enable(ClockDriver::Peripheral::swm);
    // Enable IOCON clock
    ClockDriver::enable(ClockDriver::Peripheral::iocon);

#ifdef NDEBUG
    // DISABLE SWD WHEN COMPILING IN RELEASE!!!
    // NOTE: The boot loader assigns the SWD functions to pins PIO0_2 and PIO0_3.
    //       If the user code disables the SWD functions through the switch matrix
    //       to use the pins for other functions, the SWD port is disabled.
    SwmDriver::disable(SwmDriver::PinFixed::swclk);
    SwmDriver::disable(SwmDriver::PinFixed::swdio);
#endif

#if (TARGET_PACKAGE_PIN_COUNT > 8)
    if(XARMLIB_CONFIG_SYSTEM_CLOCK <= SystemDriver::Clock::osc_30mhz)
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

    // Call the CSMSIS system clock routine to store the clock
    // frequency in the SystemCoreClock global RAM location.
    SystemCoreClockUpdate();
}




} // extern "C"

} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __LPC81X__
