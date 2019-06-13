// ----------------------------------------------------------------------------
// @file    lpc84x_flash_boot.hpp
// @brief   NXP LPC84x flash boot class.
// @date    5 June 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_LPC84X_FLASH_BOOT_HPP
#define __XARMLIB_TARGETS_LPC84X_FLASH_BOOT_HPP

#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class FlashBootDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // NOTE: the application should destruct all objects instantiated by itself before
        static void boot_application(const int32_t flash_address)
        {
            system_cleanup();

            // NOTE: static variables are needed since changed the stack pointer out from under the compiler
            //       we need to ensure the values we are using are not stored on the previous stack

            // Get the application stack pointer (first entry in the application vector table)
            static uint32_t app_stack = *((uint32_t *)flash_address);

            typedef void (*app)(void);

            // Get the application entry point (second entry in the application vector table)
            static app app_entry = (app) *((uint32_t *)(flash_address + 4));

            // Reconfigure vector table offset register to match the application location
            SCB->VTOR = (flash_address & SCB_VTOR_TBLOFF_Msk);

            // Set stack pointers to the application stack pointer
            __set_MSP(app_stack);
            __set_PSP(app_stack);

            // Jump to the application
            app_entry();
            // Never returns...
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void system_cleanup()
        {
            // clear all IRQs
            NVIC->ICER[0] = 0xFFFFFFFF;

            // Clear all pending IRQs
            NVIC->ICPR[0] = 0xFFFFFFFF;

            // Disable IOCON clock
            ClockDriver::disable(ClockDriver::Peripheral::iocon);
            // Disable Switch Matrix clock
            ClockDriver::disable(ClockDriver::Peripheral::swm);

            // Set FRO source for main_clk_pre_pll
            ClockDriver::set_main_clock_source(ClockDriver::MainClockSource::fro);

            // Set main_clk_pre_pll (FRO) source for main_clk
            ClockDriver::set_main_clock_pll_source(ClockDriver::MainClockPllSource::main_clk_pre_pll);

            // Set FRO source for PLL clock select (SYSPLLCLKSEL)
            ClockDriver::set_system_pll_source(ClockDriver::SystemPllSource::fro);

            // Power-down system PLL
            PowerDriver::power_down(PowerDriver::Peripheral::syspll);

            // Memory barriers for good measure
            __ISB();
            __DSB();
        }
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_FLASH_BOOT_HPP
