// ----------------------------------------------------------------------------
// @file    lpc84x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC84x MCU.
// @date    30 November 2018
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

#ifdef __LPC84X__

#include "xarmlib_config.hpp"
#include "targets/LPC84x/lpc84x_faim.hpp"
#include "targets/LPC84x/lpc84x_romdivide.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
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
    switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
    {                                                                                                           // FRO freq  | FRO direct
        case SystemDriver::Clock::OSC_LOW_POWER_1125KHZ:
        case SystemDriver::Clock::OSC_9MHZ:              ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_18MHZ, false); break;

        case SystemDriver::Clock::OSC_LOW_POWER_1500KHZ:
        case SystemDriver::Clock::OSC_12MHZ:             ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_24MHZ, false); break;

        case SystemDriver::Clock::OSC_LOW_POWER_1875KHZ:
        case SystemDriver::Clock::OSC_15MHZ:             ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_30MHZ, false); break;

        case SystemDriver::Clock::OSC_18MHZ:             ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_18MHZ, true ); break;
        case SystemDriver::Clock::OSC_30MHZ:             ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_30MHZ, true ); break;
        case SystemDriver::Clock::OSC_24MHZ:
        default:                                         ClockDriver::set_fro_frequency(ClockDriver::FroFrequency::FREQ_24MHZ, true ); break;
    }

    // Set FRO source for main_clk_pre_pll
    ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::FRO);

    // Set main_clk_pre_pll (FRO) source for main_clk
    ClockDriver::set_main_clock_pll_source(ClockDriver::MainClockPllSource::MAIN_CLK_PRE_PLL);

    // Set the main_clock divide by 1
    ClockDriver::set_system_clock_divider(1);
}




static inline void mcu_startup_set_xtal_clock()
{
    // Disable pull-up and pull-down for XTALIN and XTALOUT pin
    PinDriver::set_mode(PinDriver::Name::P0_8, PinDriver::FunctionMode::HIZ);
    PinDriver::set_mode(PinDriver::Name::P0_9, PinDriver::FunctionMode::HIZ);

    // Use Switch Matrix to enable XTALIN/XTALOUT functions
    SwmDriver::enable(SwmDriver::PinFixed::XTALIN);
    SwmDriver::enable(SwmDriver::PinFixed::XTALOUT);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    ClockDriver::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    PowerDriver::power_up(PowerDriver::Peripheral::SYSOSC);

    // Wait 500 us for system oscillator to stabilize (typical time from datasheet). The for
    // loop takes 7 clocks per iteration and executes at a maximum of 30 MHz (33.333 ns),
    // so worst case: i = (500 us) / (7 * 33.333 ns) = 2142.9 => 2143
    for(uint32_t i = 0; i < 2143; i++) __NOP();

    // Choose sys_osc_clk source for external clock select (EXTCLKSEL)
    ClockDriver::set_external_clock_source(ClockDriver::ExternalClockSource::SYS_OSC_CLK);

    // Set external_clk source for PLL clock select (SYSPLLCLKSEL)
    ClockDriver::set_system_pll_source(ClockDriver::SystemPllSource::EXTERNAL_CLK);

    // Configure the PLL subsystem according to the system configuration
    switch(XARMLIB_CONFIG_SYSTEM_CLOCK)
    {
        case SystemDriver::Clock::XTAL_9MHZ:  ClockDriver::set_system_pll_divider(2, 2); //  9MHz => M=3; P=4; DIV=4
                                              ClockDriver::set_system_clock_divider(4);  // Divide the main_clock by 4 (SYSAHBCLKDIV)
                                              break;
        case SystemDriver::Clock::XTAL_12MHZ: ClockDriver::set_system_pll_divider(1, 2); // 12MHz => M=2; P=4; DIV=2
                                              ClockDriver::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                              break;
        case SystemDriver::Clock::XTAL_15MHZ: ClockDriver::set_system_pll_divider(4, 1); // 15MHz => M=5; P=2; DIV=4
                                              ClockDriver::set_system_clock_divider(4);  // Divide the main_clock by 4 (SYSAHBCLKDIV)
                                              break;
        case SystemDriver::Clock::XTAL_18MHZ: ClockDriver::set_system_pll_divider(2, 2); // 18MHz => M=3; P=4; DIV=2
                                              ClockDriver::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                              break;
        case SystemDriver::Clock::XTAL_30MHZ: ClockDriver::set_system_pll_divider(4, 1); // 30MHz => M=5; P=2; DIV=2
                                              ClockDriver::set_system_clock_divider(2);  // Divide the main_clock by 2 (SYSAHBCLKDIV)
                                              break;
        case SystemDriver::Clock::XTAL_24MHZ:
        default:                              ClockDriver::set_system_pll_divider(1, 2); // 24MHz => M=2; P=4; DIV=1
                                              ClockDriver::set_system_clock_divider(1);  // Divide the main_clock by 1 (SYSAHBCLKDIV)
                                              break;
    }

    // Power-up system PLL *ONLY* after setting the dividers
    PowerDriver::power_up(PowerDriver::Peripheral::SYSPLL);

    // Wait for the system PLL to lock
    ClockDriver::wait_system_pll_lock();

    // Set sys_pll_clk source for main clock PLL select(MAINCLKPLLSEL)
    ClockDriver::set_main_clock_pll_source(ClockDriver::MainClockPllSource::SYS_PLL_CLK);

    // Disable the unused internal oscillator
    PowerDriver::power_down(PowerDriver::Peripheral::FRO);
    PowerDriver::power_down(PowerDriver::Peripheral::FROOUT);
}




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

void mcu_startup_initialize_hardware_early()
{}




void mcu_startup_initialize_hardware()
{
    // ------------------------------------------------------------------------
    // https://community.nxp.com/thread/472132
    // Helder Parracho @ 20 March 2018
    // @REVIEW: Brown-Out Detector with bug? Disabled while pending for a solution...
    // Helder Parracho @ 28 April 2018
    // @REVIEW: Disabled the reset function before powering up the peripheral. Also,
    //          Inserted a 10us waiting loop between power up and enabling reset function.
    //          The workaround seems to work. Lets leave it enabled for now.

    // Disable brown-out reset function before powering up peripheral
    BrownOutDriver::disable_reset();
    PowerDriver::power_up(PowerDriver::Peripheral::BOD);

    // Wait 10 us. The for loop takes 7 clocks per iteration
    // and executes at a maximum of 30 MHz (33.333 ns), so
    // worst case: i = (10 us) / (7 * 33.333 ns) = 42.9 => 43
    for(uint32_t i = 0; i < 43; i++) __NOP();

    // Enable brown-out detection with reset level 3 (2.63V ~ 2.76V)
    BrownOutDriver::enable_reset(BrownOutDriver::Level::LEVEL_3);
    // ------------------------------------------------------------------------

    // Patch the AEABI integer divide functions to use MCU's romdivide library
    ROMDIVIDE_PatchAeabiIntegerDivide();

    // Get the FAIM low power boot flag from the clock frequency selection in the system configuration
    const auto boot_config = (XARMLIB_CONFIG_SYSTEM_CLOCK <= SystemDriver::Clock::OSC_LOW_POWER_1875KHZ) ? FaimDriver::Boot::LOW_POWER : FaimDriver::Boot::NORMAL;

    // Ensure the FAIM configuration is well defined accordingly to the supplied parameters
    FaimDriver::ensures(XARMLIB_CONFIG_FAIM_SWD,
                        boot_config,
                        XARMLIB_CONFIG_FAIM_ISP_UART0_TX_PIN,
                        XARMLIB_CONFIG_FAIM_ISP_UART0_RX_PIN,
                        XARMLIB_CONFIG_FAIM_GPIO_PINS);

    // Disable clock input sources that aren't needed
    ClockDriver::set_clockout_source(ClockDriver::ClockoutSource::NONE);
    ClockDriver::set_sct_clock_source(ClockDriver::SctClockSource::NONE);
    ClockDriver::set_adc_clock_source(ClockDriver::AdcClockSource::NONE);
    ClockDriver::set_frg_clock_source(ClockDriver::FrgClockSelect::FRG0, ClockDriver::FrgClockSource::NONE);
    ClockDriver::set_frg_clock_source(ClockDriver::FrgClockSelect::FRG1, ClockDriver::FrgClockSource::NONE);

    // Enable Switch Matrix clock
    ClockDriver::enable(ClockDriver::Peripheral::SWM);
    // Enable IOCON clock
    ClockDriver::enable(ClockDriver::Peripheral::IOCON);

    if(XARMLIB_CONFIG_SYSTEM_CLOCK <= SystemDriver::Clock::OSC_30MHZ)
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
} // namespace targets
} // namespace xarmlib

#endif // __LPC84X__
