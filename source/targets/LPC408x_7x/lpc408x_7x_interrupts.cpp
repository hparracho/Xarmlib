// ----------------------------------------------------------------------------
// @file    lpc408x_7x_interrupts.cpp
// @brief   IRQ handlers and vector table for NXP LPC408x_7x MCU.
// @date    19 October 2018
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

#ifdef __LPC408X_7X__

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

// Core level (CM4) exception handlers
void Reset_Handler     (void);
void NMI_Handler       (void);
void HardFault_Handler (void);
void MemManage_Handler (void);
void BusFault_Handler  (void);
void UsageFault_Handler(void);
void SVC_Handler       (void);
void DebugMon_Handler  (void);
void PendSV_Handler    (void);
void SysTick_Handler   (void);




// ----------------------------------------------------------------------------
// Forward declaration of the specific IRQ handlers. These are
// aliased to the IRQ_DefaultHandler, which is a 'forever' loop. When
// the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions.
// ----------------------------------------------------------------------------

// Chip level (LPC408x_7x) peripheral handlers
void WDT_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void TIMER0_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void TIMER1_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void TIMER2_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void TIMER3_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART2_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART3_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C2_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SSP0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SSP1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PLL0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void RTC_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void EINT0_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void EINT1_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void EINT2_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void EINT3_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void BOD_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void USB_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2S_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENET_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SDC_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MCPWM_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void QEI_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PLL1_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void USBACT_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CANACT_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART4_IRQHandler (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SSP2_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void LCD_IRQHandler   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void GPIO_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_IRQHandler  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void EEPROM_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));




// External declaration for LPC MCU vector table checksum from the Linker Script
extern void __valid_user_code_checksum(void) __attribute__ ((weak));




// Declaration of IRQ handler pointer type
using IRQ_HandlerPtr = void (*)(void);

__attribute__ ((used, section(".isr_vector")))
const IRQ_HandlerPtr __vectors[] =
{
    // Core level (CM4) exception handlers
    (IRQ_HandlerPtr)(&__STACK_TOP),         // Initial stack pointer
    Reset_Handler,                          // Reset handler (application entry)
    NMI_Handler,                            // NMI handler
    HardFault_Handler,                      // Hard Fault handler
    MemManage_Handler,                      // MPU Fault handler
    BusFault_Handler,                       // Bus Fault handler
    UsageFault_Handler,                     // Usage Fault handler
    __valid_user_code_checksum,             // LPC MCU Checksum
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    SVC_Handler,                            // SVCall handler
    DebugMon_Handler,                       // Debug Monitor handler
    nullptr,                                // RESERVED
    PendSV_Handler,                         // PendSV handler
    SysTick_Handler,                        // SysTick handler

    // Chip level (LPC408x_7x) peripheral handlers
    WDT_IRQHandler,                         // Watchdog handler
    TIMER0_IRQHandler,                      // Timer 0 handler
    TIMER1_IRQHandler,                      // Timer 1 handler
    TIMER2_IRQHandler,                      // Timer 2 handler
    TIMER3_IRQHandler,                      // Timer 3 handler
    UART0_IRQHandler,                       // UART0 handler
    UART1_IRQHandler,                       // UART1 handler
    UART2_IRQHandler,                       // UART2 handler
    UART3_IRQHandler,                       // UART3 handler
    PWM1_IRQHandler,                        // PWM1 handler
    I2C0_IRQHandler,                        // I2C0 handler
    I2C1_IRQHandler,                        // I2C1 handler
    I2C2_IRQHandler,                        // I2C2 handler
    nullptr,                                // NOT USED
    SSP0_IRQHandler,                        // SSP0 handler
    SSP1_IRQHandler,                        // SSP1 handler
    PLL0_IRQHandler,                        // PLL0 (Main PLL) handler
    RTC_IRQHandler,                         // RTC and Event Monitor/Recorder handler
    EINT0_IRQHandler,                       // External Interrupt 0 handler
    EINT1_IRQHandler,                       // External Interrupt 1 handler
    EINT2_IRQHandler,                       // External Interrupt 2 handler
    EINT3_IRQHandler,                       // External Interrupt 3 handler
    ADC_IRQHandler,                         // ADC handler
    BOD_IRQHandler,                         // BOD handler
    USB_IRQHandler,                         // USB handler
    CAN_IRQHandler,                         // CAN handler
    DMA_IRQHandler,                         // DMA Controller handler
    I2S_IRQHandler,                         // I2S handler
    ENET_IRQHandler,                        // Ethernet handler
    SDC_IRQHandler,                         // SD Card Interface handler
    MCPWM_IRQHandler,                       // Motor Control PWM handler
    QEI_IRQHandler,                         // Quadrature Encoder handler
    PLL1_IRQHandler,                        // PLL1 (USB and/or SPIFI PLL) handler
    USBACT_IRQHandler,                      // USB Activity Interrupt handler
    CANACT_IRQHandler,                      // CAN Activity Interrupt handler
    UART4_IRQHandler,                       // UART4 handler
    SSP2_IRQHandler,                        // SSP2 handler
    LCD_IRQHandler,                         // LCD Controller handler
    GPIO_IRQHandler,                        // GPIO Interrupts handler
    PWM0_IRQHandler,                        // PWM0 handler
    EEPROM_IRQHandler,                      // EEPROM handler
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
void MemManage_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void BusFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), noreturn, weak))
void UsageFault_Handler(void)
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
void DebugMon_Handler(void)
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

#endif // __LPC408X_7X__
