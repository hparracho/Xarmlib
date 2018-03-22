// ----------------------------------------------------------------------------
// @file    lpc84x_interrupts.cpp
// @brief   IRQ handlers and vector table for NXP LPC84x MCU.
// @date    21 March 2018
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

#ifdef __LPC84X__

#include "xarmlib_config.h"

namespace xarmlib
{
namespace lpc84x
{

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

// Chip level (LPC84x) peripheral handlers
void SPI0_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SPI1_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DAC0_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART2_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FAIM_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C1_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C0_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SCT_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MRT_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WDT_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void BOD_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FLASH_IRQHandler    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WKT_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_SEQA_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_SEQB_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_THCMP_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_OVR_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA_IRQHandler      (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C2_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C3_IRQHandler     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CTIMER0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT2_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT3_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT4_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT5_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT6_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PININT7_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));




// External declaration for LPC MCU vector table checksum from the Linker Script
extern void __valid_user_code_checksum(void) __attribute__ ((weak));




// Declaration of IRQ handler pointer type
using IRQ_HandlerPtr = void (*)(void);

__attribute__ ((used, section(".isr_vector")))
const IRQ_HandlerPtr __interrupts[] =
{
    // Core level (CM0+) exception handlers
    (IRQ_HandlerPtr)(&__STACK_TOP),         // The initial stack pointer
    Reset_Handler,                          // The reset handler (application entry)
    NMI_Handler,                            // The NMI handler
    HardFault_Handler,                      // The hard fault handler
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    __valid_user_code_checksum,             // LPC MCU Checksum
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    SVC_Handler,                            // SVCall handler
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    (IRQ_HandlerPtr)(0UL),                  // Reserved
    PendSV_Handler,                         // The PendSV handler
    SysTick_Handler,                        // The SysTick handler

    // Chip level (LPC84x) peripheral handlers
    SPI0_IRQHandler,                        // SPI0
    SPI1_IRQHandler,                        // SPI1
    DAC0_IRQHandler,                        // DAC0
    UART0_IRQHandler,                       // UART0
    UART1_IRQHandler,                       // UART1
    UART2_IRQHandler,                       // UART2
    FAIM_IRQHandler,                        // FAIM
    I2C1_IRQHandler,                        // I2C1 controller
    I2C0_IRQHandler,                        // I2C0 controller
    SCT_IRQHandler,                         // Smart Counter Timer
    MRT_IRQHandler,                         // Multi-Rate Timer
    CMP_IRQHandler,                         // Comparator shared slot with CAP Touch 
    WDT_IRQHandler,                         // Watchdog
    BOD_IRQHandler,                         // Brown Out Detect
    FLASH_IRQHandler,                       // Flash Interrupt
    WKT_IRQHandler,                         // Wakeup timer
    ADC_SEQA_IRQHandler,                    // ADC sequence A completion
    ADC_SEQB_IRQHandler,                    // ADC sequence B completion
    ADC_THCMP_IRQHandler,                   // ADC threshold compare
    ADC_OVR_IRQHandler,                     // ADC overrun
    DMA_IRQHandler,                         // DMA
    I2C2_IRQHandler,                        // I2C2 controller
    I2C3_IRQHandler,                        // I2C3 controller
    CTIMER0_IRQHandler,                     // Timer0
    PININT0_IRQHandler,                     // PIO INT0
    PININT1_IRQHandler,                     // PIO INT1
    PININT2_IRQHandler,                     // PIO INT2
    PININT3_IRQHandler,                     // PIO INT3
    PININT4_IRQHandler,                     // PIO INT4
    PININT5_IRQHandler,                     // PIO INT5 shared slot with DAC1
    PININT6_IRQHandler,                     // PIO INT6 shared slot with UART3
    PININT7_IRQHandler,                     // PIO INT7 shared slot with UART4
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

// Forward declaration of MCU startup function.
void mcu_startup(void);

__attribute__ ((section(".after_vectors")))
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

} // namespace lpc84x
} // namespace xarmlib

#endif // __LPC84X__
