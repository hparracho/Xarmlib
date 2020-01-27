// ----------------------------------------------------------------------------
// @file    kv5x_flash_boot.hpp
// @brief   Kinetis KV5x flash boot class.
// @date    16 January 2020
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

#ifndef __XARMLIB_TARGETS_KV5X_FLASH_BOOT_HPP
#define __XARMLIB_TARGETS_KV5X_FLASH_BOOT_HPP

#include "targets/KV5x/kv5x_cmsis.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




class FlashBootDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // NOTES: - the application should destruct all objects instantiated by itself before;
        //        - only allowed in Normal Run mode (System::clock::*_run).
        static void boot_application(const int32_t flash_address)
        {
            // Check to make sure in RUN mode
            assert((SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK) == 1);

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

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void system_cleanup()
        {
            // clear all IRQs
            NVIC->ICER[0] = 0xFFFFFFFF;
            NVIC->ICER[1] = 0xFFFFFFFF;
            NVIC->ICER[2] = 0xFFFFFFFF;
            NVIC->ICER[3] = 0xFFFFFFFF;
            NVIC->ICER[4] = 0xFFFFFFFF;
            NVIC->ICER[5] = 0xFFFFFFFF;
            NVIC->ICER[6] = 0xFFFFFFFF;
            NVIC->ICER[7] = 0xFFFFFFFF;

            // Clear all pending IRQs
            NVIC->ICPR[0] = 0xFFFFFFFF;
            NVIC->ICPR[1] = 0xFFFFFFFF;
            NVIC->ICPR[2] = 0xFFFFFFFF;
            NVIC->ICPR[3] = 0xFFFFFFFF;
            NVIC->ICPR[4] = 0xFFFFFFFF;
            NVIC->ICPR[5] = 0xFFFFFFFF;
            NVIC->ICPR[6] = 0xFFFFFFFF;
            NVIC->ICPR[7] = 0xFFFFFFFF;

            // De-initialize hardware such as disabling ports and XBARs clock gate
            SIM->SCGC5 &= static_cast<uint32_t>(~(SIM_SCGC5_PORTA_MASK
                                                | SIM_SCGC5_PORTB_MASK
                                                | SIM_SCGC5_PORTC_MASK
                                                | SIM_SCGC5_PORTD_MASK
                                                | SIM_SCGC5_PORTE_MASK
                                                | SIM_SCGC5_XBARA_MASK
                                                | SIM_SCGC5_XBARB_MASK));

            // Restore global interrupt (needed to debug)
            __enable_irq();

            // Memory barriers for good measure
            __ISB();
            __DSB();
        }
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV5X_FLASH_BOOT_HPP
