// ----------------------------------------------------------------------------
// @file    lpc81x_flash_boot.hpp
// @brief   NXP LPC81x flash boot class.
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

#ifndef XARMLIB_TARGETS_LPC81X_FLASH_BOOT_HPP
#define XARMLIB_TARGETS_LPC81X_FLASH_BOOT_HPP

#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"




namespace xarmlib::targets::lpc81x
{

class FlashBoot
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // NOTE: the application should destruct all objects instantiated by itself before
    [[noreturn]] static void boot_application(const int32_t flash_address)
    {
        system_cleanup();

        // Get the application stack pointer (first entry in the application vector table)
        uint32_t app_stack = *((uint32_t *)flash_address);

        typedef void (*app)(void);

        // Get the application entry point (second entry in the application vector table)
        app app_entry = (app) *((uint32_t *)(flash_address + 4));

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

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static void system_cleanup()
    {
        // clear all IRQs
        NVIC->ICER[0] = 0xFFFFFFFFUL;

        // Clear all pending IRQs
        NVIC->ICPR[0] = 0xFFFFFFFFUL;

        // Disable IOCON clock
        SysClock::disable(SysClock::Peripheral::iocon);
        // Disable Switch Matrix clock
        SysClock::disable(SysClock::Peripheral::swm);

        // Set main clock source directly to the IRC oscillator
        SysClock::set_main_clock_source(SysClock::MainClockSource::irc);

        // Power-down system PLL
        Power::power_down(Power::Peripheral::syspll);

        // Restore global interrupt (needed to debug)
        //__enable_irq(); @TODO: test if this is really needed

        // Memory barriers for good measure
        __ISB();
        __DSB();
    }
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_FLASH_BOOT_HPP
