// ----------------------------------------------------------------------------
// @file    lpc84x_startup_hooks.cpp
// @brief   Startup initialization hooks definition for NXP LPC84x MCU.
// @date    13 March 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
//
// Emanuel Pinto(emanuelangelopinto@gmail.com) is an official contributor of
// this library and some of the following code is based on his original work.
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




#ifdef __LPC84X__

#include "xarmlib_config.h"

#ifdef __USE_ROMDIVIDE
#include "targets/LPC84x/lpc84x_romdivide.h"
#endif

#include <targets/LPC84x/lpc84x_syscon_power.hpp>
#include <targets/LPC84x/lpc84x_syscon_clock.hpp>




namespace xarmlib
{
namespace lpc84x
{




// ----------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------------

static inline void mcu_startup_set_fro_clock()
{
    // Configure the FRO subsystem according to the project configurations
    //Clock_SetFro(g_lpc84x_config_fro_frequency, g_lpc84x_config_fro_direct);

    // Set FRO source for main_clk_pre_pll
    Clock::set_main_clock_source(Clock::MainClockSource::FRO);

    // Set main_clk_pre_pll (FRO) source for main_clk
    Clock::set_main_clock_pll_source(Clock::MainClockPllSource::MAIN_CLK_PRE_PLL);

    // Set the main_clock divide by 1
    Clock::set_system_clock_divider(1);
}




static inline void mcu_startup_set_xtal_clock()
{
    // Disable pull-up and pull-down for XTALIN and XTALOUT pins
    //Pin_Mode(PinName::P0_8, PinMode::PULL_NONE);
    //Pin_Mode(PinName::P0_9, PinMode::PULL_NONE);

    // Use Switch Matrix Tool to enable XTALIN/XTALOUT function
    //Swm_EnableFixedPin(SwmPinFixed::XTALIN);
    //Swm_EnableFixedPin(SwmPinFixed::XTALOUT);

    // Use crystal oscillator with 1-20 MHz frequency range
    const bool bypass_osc = false;
    const bool high_freq  = false;
    Clock::set_system_oscillator(bypass_osc, high_freq);

    // Power-up crystal oscillator
    Power::power_up(Power::Peripheral::SYSOSC);

    // Wait 500 uSec for sysosc to stabilize (typical time from datasheet)
    // The for loop takes 7 clocks per iteration and executes at a maximum of 30 MHz (33 nSec) based on FRO settings
    // So worst case i = (500 uSec) / (7 * 33 nSec) = 2142
    for(uint32_t i = 0; i < 2142; i++) __NOP();

    // Choose sys_osc_clk source for external_clk
    Clock::set_external_clock_source(Clock::ExternalClockSource::SYS_OSC_CLK);

    // Set external clock source for main_clk_pre_pll
    Clock::set_main_clock_source(Clock::MainClockSource::EXTERNAL_CLK);

    // Set main_clk_pre_pll (external clock) source for main_clk
    Clock::set_main_clock_pll_source(Clock::MainClockPllSource::MAIN_CLK_PRE_PLL);

    // Set the main_clock divide by 1
    Clock::set_system_clock_divider(1);
}




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

extern "C" void mcu_startup_initialize_hardware_early(void)
{}




extern "C" void mcu_startup_initialize_hardware(void)
{
#ifdef __USE_ROMDIVIDE
    // Patch the AEABI integer divide functions to use MCU's romdivide library.
    ROMDIVIDE_PatchAeabiIntegerDivide();
#endif

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

    #if defined(USE_XTAL)
        mcu_startup_set_xtal_clock();
    #else
        mcu_startup_set_fro_clock();
    #endif

    // Call the CSMSIS system initialization routine.
    //SystemInit();

    // Call the CSMSIS system clock routine to store the clock
    // frequency in the SystemCoreClock global RAM location.
    //SystemCoreClockUpdate();

    // Enable brown-out detection with reset level 3 (2.63V ~ 2.76V)
    Power::power_up(Power::Peripheral::BOD);
    BrownOut::enable_reset(BrownOut::Level::LEVEL_3);
}




} // namespace lpc84x
} // namespace xarmlib

#endif // __LPC84X__
