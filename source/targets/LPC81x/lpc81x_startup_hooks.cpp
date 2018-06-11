// ----------------------------------------------------------------------------
// @file    lpc81x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC81x MCU.
// @date    7 June 2018
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

#include "system/target"

#ifdef __LPC81X__

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "xarmlib_config.hpp"

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
    if(XARMLIB_SYSTEM_CLOCK == System::Clock::OSC_12MHZ)
    {
        // Set the main clock divide by 1
        Clock::set_system_clock_divider(1);

        // Set main clock source directly to the IRC oscillator
        Clock::set_main_clock_source(Clock::MainClockSource::IRC);
    }
    else
    {
        // Set IRC oscillator source for system PLL clock select
        Clock::set_system_pll_source(Clock::SystemPllSource::IRC);

        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_SYSTEM_CLOCK)
        {
            case System::Clock::OSC_30MHZ: Clock::set_system_pll_divider(4, 1); // 30MHz => M=5; P=2; DIV=2
                                           Clock::set_system_clock_divider(2);  // Divide the main_clock by 2
                                           break;
            case System::Clock::OSC_24MHZ:
            default:                       Clock::set_system_pll_divider(1, 2); // 24MHz => M=2; P=3; DIV=1
                                           Clock::set_system_clock_divider(1);  // Divide the main_clock by 1
                                           break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        Power::power_up(Power::Peripheral::SYSPLL);

        // Wait for the system PLL to lock
        Clock::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        Clock::set_main_clock_source(Clock::MainClockSource::SYS_PLL_OUT_CLK);
    }
}




static inline void mcu_startup_set_xtal_clock()
{
#if (__LPC81X_GPIOS__ >= 14)
    // Disable pull-up and pull-down for XTALIN and XTALOUT pin
    Pin::set_mode(Pin::Name::P0_8, Pin::FunctionMode::HIZ);
    Pin::set_mode(Pin::Name::P0_9, Pin::FunctionMode::HIZ);

    // Use Switch Matrix to enable XTALIN/XTALOUT functions
    Swm::enable(Swm::PinFixed::XTALIN);
    Swm::enable(Swm::PinFixed::XTALOUT);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    Clock::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    Power::power_up(Power::Peripheral::SYSOSC);

    // Wait 500 uSec for system oscillator to stabilize (typical time from datasheet). The for
    // loop takes 7 clocks per iteration and executes at a maximum of 30 MHz (33.333 nSec),
    // so worst case: i = (500 uSec) / (7 * 33.333 nSec) = 2142.9 => 2143
    for(uint32_t i = 0; i < 2143; i++) __NOP();

    // Set system oscillator source for system PLL clock select
    Clock::set_system_pll_source(Clock::SystemPllSource::SYS_OSC_CLK);

    if(XARMLIB_SYSTEM_CLOCK == System::Clock::XTAL_12MHZ)
    {
        // Set the main clock divide by 1
        Clock::set_system_clock_divider(1);

        // Set system PLL in source for main clock select
        Clock::set_main_clock_source(Clock::MainClockSource::SYS_PLL_IN_CLK);
    }
    else
    {
        // Configure the PLL subsystem according to the system configuration
        switch(XARMLIB_SYSTEM_CLOCK)
        {
            case System::Clock::XTAL_30MHZ: Clock::set_system_pll_divider(4, 1); // 30MHz => M=5; P=2; DIV=2
                                            Clock::set_system_clock_divider(2);  // Divide the main_clock by 2
                                            break;
            case System::Clock::XTAL_24MHZ:
            default:                        Clock::set_system_pll_divider(1, 2); // 24MHz => M=2; P=3; DIV=1
                                            Clock::set_system_clock_divider(1);  // Divide the main_clock by 1
                                            break;
        }

        // Power-up system PLL *ONLY* after setting the dividers
        Power::power_up(Power::Peripheral::SYSPLL);

        // Wait for the system PLL to lock
        Clock::wait_system_pll_lock();

        // Set system PLL out source for main clock select
        Clock::set_main_clock_source(Clock::MainClockSource::SYS_PLL_OUT_CLK);
    }
#endif

    // Disable the unused IRC oscillator
    Power::power_down(Power::Peripheral::IRC);
    Power::power_down(Power::Peripheral::IRCOUT);
}




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

void mcu_startup_initialize_hardware_early(void)
{}




void mcu_startup_initialize_hardware(void)
{
    // Enable brown-out detection with reset level 3 (2.63V ~ 2.71V)
    BrownOut::enable_reset(BrownOut::Level::LEVEL_3);

    // Enable Switch Matrix clock
    Clock::enable(Clock::Peripheral::SWM);
    // Enable IOCON clock
    Clock::enable(Clock::Peripheral::IOCON);

#if (__LPC81X_GPIOS__ < 14)
    // NOTE from manual:
    // If the open-drain pins PIO0_10 and PIO0_11 are not available on the package,
    // prevent the pins from internally floating
    Gpio::set_open_drain_pins_as_low_output();
#endif

    if(XARMLIB_SYSTEM_CLOCK <= System::Clock::OSC_30MHZ)
    {
        mcu_startup_set_irc_clock();
    }
    else
    {
        mcu_startup_set_xtal_clock();
    }

    // Call the CSMSIS system clock routine to store the clock
    // frequency in the SystemCoreClock global RAM location.
    SystemCoreClockUpdate();
}




} // extern "C"

} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __LPC81X__
