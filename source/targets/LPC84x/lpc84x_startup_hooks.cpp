// ----------------------------------------------------------------------------
// @file    lpc84x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC84x MCU.
// @date    2 May 2018
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

#include "system/target.h"

#ifdef __LPC84X__

#include "targets/LPC84x/lpc84x_cmsis.h"
#include "targets/LPC84x/lpc84x_faim.hpp"
#include "targets/LPC84x/lpc84x_romdivide.h"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

#include "xarmlib_config.h"

namespace xarmlib
{
namespace lpc84x
{




extern "C"
{




// ----------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------------

static inline void mcu_startup_set_fro_clock()
{
    // Configure the FRO subsystem according to the system configuration
    switch(XARMLIB_SYSTEM_CLOCK)
    {                                                                                         // FRO freq  | FRO direct
        case System::Clock::OSC_LOW_POWER_1125KHZ:
        case System::Clock::OSC_9MHZ:              Clock::set_fro_frequency(Clock::FroFrequency::FREQ_18MHZ, false); break;

        case System::Clock::OSC_LOW_POWER_1500KHZ:
        case System::Clock::OSC_12MHZ:             Clock::set_fro_frequency(Clock::FroFrequency::FREQ_24MHZ, false); break;

        case System::Clock::OSC_LOW_POWER_1875KHZ:
        case System::Clock::OSC_15MHZ:             Clock::set_fro_frequency(Clock::FroFrequency::FREQ_30MHZ, false); break;

        case System::Clock::OSC_18MHZ:             Clock::set_fro_frequency(Clock::FroFrequency::FREQ_18MHZ, true ); break;
        case System::Clock::OSC_30MHZ:             Clock::set_fro_frequency(Clock::FroFrequency::FREQ_30MHZ, true ); break;
        case System::Clock::OSC_24MHZ:
        default:                                   Clock::set_fro_frequency(Clock::FroFrequency::FREQ_24MHZ, true ); break;
    }

    // Set FRO source for main_clk_pre_pll
    Clock::set_main_clock_source(Clock::MainClockSource::FRO);

    // Set main_clk_pre_pll (FRO) source for main_clk
    Clock::set_main_clock_pll_source(Clock::MainClockPllSource::MAIN_CLK_PRE_PLL);

    // Set the main_clock divide by 1
    Clock::set_system_clock_divider(1);
}




static inline void mcu_startup_set_xtal_clock()
{
    // Disable pull-up and pull-down for XTALIN and XTALOUT pin
    Pin::mode(Pin::Name::P0_8, Pin::Mode::PULL_NONE);
    Pin::mode(Pin::Name::P0_9, Pin::Mode::PULL_NONE);

    // Use Switch Matrix to enable XTALIN/XTALOUT functions
    Swm::enable(Swm::PinFixed::XTALIN);
    Swm::enable(Swm::PinFixed::XTALOUT);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    Clock::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    Power::power_up(Power::Peripheral::SYSOSC);

    // Wait 500 uSec for sysosc to stabilize (typical time from datasheet). The for
    // loop takes 7 clocks per iteration and executes at a maximum of 30 MHz (33 nSec),
    // so worst case: i = (500 uSec) / (7 * 33 nSec) = 2164.5 => 2165
    for(uint32_t i = 0; i < 2165; i++) __NOP();

    // Choose sys_osc_clk source for external clock select (EXTCLKSEL)
    Clock::set_external_clock_source(Clock::ExternalClockSource::SYS_OSC_CLK);

    // Set external_clk source for PLL clock select (SYSPLLCLKSEL)
    Clock::set_system_pll_source(Clock::SystemPllSource::EXTERNAL_CLK);

    // Configure the PLL subsystem according to the system configuration
    switch(XARMLIB_SYSTEM_CLOCK)
    {
        case System::Clock::XTAL_9MHZ:  Clock::set_system_pll_divider(2, 2); //  9MHz => M=3; P=4; DIV=4
                                        Clock::set_system_clock_divider(4);  // Divide the main_clock by 4 (SYSAHBCLKDIV)
                                        break;
        case System::Clock::XTAL_12MHZ: Clock::set_system_pll_divider(1, 2); // 12MHz => M=2; P=4; DIV=2
                                        Clock::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                        break;
        case System::Clock::XTAL_15MHZ: Clock::set_system_pll_divider(4, 1); // 15MHz => M=5; P=2; DIV=4
                                        Clock::set_system_clock_divider(4);  // Divide the main_clock by 4 (SYSAHBCLKDIV)
                                        break;
        case System::Clock::XTAL_18MHZ: Clock::set_system_pll_divider(2, 2); // 18MHz => M=3; P=4; DIV=2
                                        Clock::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                        break;
        case System::Clock::XTAL_30MHZ: Clock::set_system_pll_divider(4, 1); // 30MHz => M=5; P=2; DIV=2
                                        Clock::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                        break;
        case System::Clock::XTAL_24MHZ:
        default:                        Clock::set_system_pll_divider(1, 2); // 24MHz => M=2; P=4; DIV=1
                                        Clock::set_system_clock_divider(1);  // Divide the main_clock by 1 (SYSAHBCLKDIV)
                                        break;
    }

    // Power-up system PLL *ONLY* after setting the dividers
    Power::power_up(Power::Peripheral::SYSPLL);

    // Wait for the system PLL to lock
    Clock::wait_system_pll_lock();

    // Set sys_pll_clk source for main clock PLL select(MAINCLKPLLSEL)
    Clock::set_main_clock_pll_source(Clock::MainClockPllSource::SYS_PLL_CLK);

    // Disable the unused internal oscillator
    Power::power_down(Power::Peripheral::FRO);
    Power::power_down(Power::Peripheral::FROOUT);
}




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

void mcu_startup_initialize_hardware_early(void)
{}




void mcu_startup_initialize_hardware(void)
{
    // ------------------------------------------------------------------------
    // Helder Parracho @ 20 March 2018
    // @REVIEW: Brown-Out Detector with bug? Disabled while pending for a solution...
    // Helder Parracho @ 28 April 2018
    // @REVIEW: Disabled the reset function before powering up the peripheral. Also,
    //          Inserted a 10us waiting loop between power up and enabling reset function.
    //          The workaround seems to work. Lets leave it enabled for now.

    // Disable brown-out reset function before powering up peripheral
    BrownOut::disable_reset();
    Power::power_up(Power::Peripheral::BOD);

    // Wait 10 uSec. The for loop takes 7 clocks per iteration
    // and executes at a maximum of 30 MHz (33 nSec), so worst case:
    // i = (10 uSec) / (7 * 33 nSec) = 43.3 => 44
    for(uint32_t i = 0; i < 44; i++) __NOP();

    // Enable brown-out detection with reset level 3 (2.63V ~ 2.76V)
    BrownOut::enable_reset(BrownOut::Level::LEVEL_3);
    // ------------------------------------------------------------------------

    // Patch the AEABI integer divide functions to use MCU's romdivide library
    ROMDIVIDE_PatchAeabiIntegerDivide();

    // Get the FAIM low power boot flag from the clock frequency selection in the system configuration
    const auto boot_config = (XARMLIB_SYSTEM_CLOCK <= System::Clock::OSC_LOW_POWER_1875KHZ) ? Faim::Boot::LOW_POWER : Faim::Boot::NORMAL;

    // Ensure the FAIM configuration is well defined accordingly to the supplied parameters
    Faim::ensures(XARMLIB_CONFIG_FAIM_SWD,
                  boot_config,
                  XARMLIB_CONFIG_FAIM_ISP_UART0_TX_PIN,
                  XARMLIB_CONFIG_FAIM_ISP_UART0_RX_PIN,
                  XARMLIB_CONFIG_FAIM_GPIO_PINS);

    // Disable clock input sources that aren't needed
    Clock::set_clockout_source(Clock::ClockoutSource::NONE);
    Clock::set_sct_clock_source(Clock::SctClockSource::NONE);
    Clock::set_adc_clock_source(Clock::AdcClockSource::NONE);
    Clock::set_frg_clock_source(Clock::FrgClockSelect::FRG0, Clock::FrgClockSource::NONE);
    Clock::set_frg_clock_source(Clock::FrgClockSelect::FRG1, Clock::FrgClockSource::NONE);

    // Enable Switch Matrix clock
    Clock::enable(Clock::Peripheral::SWM);
    // Enable IOCON clock
    Clock::enable(Clock::Peripheral::IOCON);

    if(XARMLIB_SYSTEM_CLOCK <= System::Clock::OSC_30MHZ)
    {
        mcu_startup_set_fro_clock();
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

} // namespace lpc84x
} // namespace xarmlib

#endif // __LPC84X__
