// ----------------------------------------------------------------------------
// @file    lpc81x_interrupts.cpp
// @brief   IRQ handlers and vector table for NXP LPC81x MCU.
// @date    9 July 2018
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

extern "C"
{




#ifdef MCUXPRESSO_MANAGED_LINKER_SCRIPTS
// Reset entry point when using MCUXpresso Managed Linker Scripts
void ResetISR(void) __attribute__ ((alias("Reset_Handler")));
// External declaration for the pointer to the stack top from the Linker Script
extern unsigned int _vStackTop;
#define __STACK_TOP _vStackTop
#else // Custom Linker Scripts
extern unsigned int __stack_top;
#define __STACK_TOP __stack_top
#endif // MCUXPRESSO_MANAGED_LINKER_SCRIPTS




// ----------------------------------------------------------------------------
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this
// will automatically take precedence over these weak definitions.
// ----------------------------------------------------------------------------

void IRQ_DefaultHandler(void);

// Core level (CM0+) exception handlers
void Reset_Handler     (void);
void NMI_Handler       (void);
void HardFault_Handler (void);
void SVC_Handler       (void);
void PendSV_Handler    (void);
void SysTick_Handler   (void);




// ----------------------------------------------------------------------------
// Forward declaration of the specific IRQ handlers. These are
// aliased to the IRQ_DefaultHandler, which is a 'forever' loop. When
// the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions.
// ----------------------------------------------------------------------------

// Chip level (LPC81x) peripheral handlers
void SPI0_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SPI1_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART2_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SCT_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MRT_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ACMP_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WDT_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void BOD_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WKT_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT0_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT1_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT2_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT3_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT4_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT5_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT6_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT7_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));




// External declaration for LPC MCU vector table checksum from the Linker Script
extern void __valid_user_code_checksum(void) __attribute__ ((weak));




// Declaration of IRQ handler pointer type
using IRQ_HandlerPtr = void (*)(void);

__attribute__ ((used, section(".isr_vector")))
const IRQ_HandlerPtr __vectors[] =
{
    // Core level (CM0+) exception handlers
    (IRQ_HandlerPtr)(&__STACK_TOP),         // Initial stack pointer
    Reset_Handler,                          // Reset handler (application entry)
    NMI_Handler,                            // NMI handler
    HardFault_Handler,                      // Hard Fault handler
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    __valid_user_code_checksum,             // LPC MCU Checksum
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    SVC_Handler,                            // SVCall handler
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    PendSV_Handler,                         // PendSV handler
    SysTick_Handler,                        // SysTick handler

    // Chip level (LPC81x) peripheral handlers
    SPI0_IRQHandler,                        // SPI0 handler
    SPI1_IRQHandler,                        // SPI1 handler
    nullptr,                                // RESERVED
    UART0_IRQHandler,                       // UART0 handler
    UART1_IRQHandler,                       // UART1 handler
    UART2_IRQHandler,                       // UART2 handler
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    I2C_IRQHandler,                         // I2C handler
    SCT_IRQHandler,                         // SCT handler
    MRT_IRQHandler,                         // MRT handler
    ACMP_IRQHandler,                        // Analog Comparator handler
    WDT_IRQHandler,                         // Watchdog handler
    BOD_IRQHandler,                         // BOD handler
    nullptr,                                // RESERVED
    WKT_IRQHandler,                         // WKT handler
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    PININT0_IRQHandler,                     // Pin Interrupt 0 handler
    PININT1_IRQHandler,                     // Pin Interrupt 1 handler
    PININT2_IRQHandler,                     // Pin Interrupt 2 handler
    PININT3_IRQHandler,                     // Pin Interrupt 3 handler
    PININT4_IRQHandler,                     // Pin Interrupt 4 handler
    PININT5_IRQHandler,                     // Pin Interrupt 5 handler
    PININT6_IRQHandler,                     // Pin Interrupt 6 handler
    PININT7_IRQHandler,                     // Pin Interrupt 7 handler
};




// Define breakpoint macro for debug
#ifdef DEBUG
#define __DEBUG_BKPT()      asm volatile ("bkpt 0")
#else
#define __DEBUG_BKPT()
#endif




// ----------------------------------------------------------------------------
// Processor ends up here if an unexpected interrupt occurs or
// a specific handler is not present in the application code.
// When in DEBUG, triggers a debug exception to clearly notify
// the user of the exception and help identify the cause.
// ----------------------------------------------------------------------------

__attribute__ ((section(".after_vectors"), noreturn))
void IRQ_DefaultHandler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}




// ----------------------------------------------------------------------------
// Default exception handlers. Override the ones here by
// defining your own handler routines in your application code.
// ----------------------------------------------------------------------------

// Forward declaration of MCU startup function
__attribute__ ((section(".after_vectors"), noreturn))
void mcu_startup();

#ifndef NDEBUG
// If both CRP word placement and LTO are enabled the 'Reset_Handler()'
// function cannot be placed after vectors because it's size overlaps the CRP
// word location. In debug this doesn't happen since you are not supposed to
// enable LTO in debug mode. The attribute effectively reduces the debug build
// size a few bytes.
__attribute__ ((section(".after_vectors"), noreturn))
#else
__attribute__ ((noreturn))
#endif
void Reset_Handler(void)
{
    mcu_startup();
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void NMI_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void HardFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void SVC_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void PendSV_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void SysTick_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}




} // extern "C"

#endif // __LPC81X__
