// ----------------------------------------------------------------------------
// @file    kv4x_interrupts.cpp
// @brief   IRQ handlers and vector table for Kinetis KV4x MCUs.
// @date    10 September 2020
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

#include "core/target_specs.hpp"

#ifdef __KV4X__

extern "C"
{




#ifdef MCUXPRESSO_MANAGED_LINKER_SCRIPTS
/*
// ----------------------------------------------------------------------------
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
// ----------------------------------------------------------------------------
__attribute__ ((used, section(".FlashConfig")))
const struct
{
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
    unsigned int word4;
} Flash_Config = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE};
*/
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

// Chip level (KV4x) peripheral handlers
void DMA0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA4_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA5_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA6_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA7_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA8_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA9_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA10_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA11_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA12_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA13_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA14_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA15_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA_Error_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MCM_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTFA_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTFA_Collision_IRQHandler          (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PMC_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void LLWU_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WDOG_EWM_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SPI0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC_ERR_IRQHandler                 (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADCA_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PDB0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void XBARA_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PDB1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DAC0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MCG_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void LPTMR0_IRQHandler                  (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PORTA_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PORTB_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PORTC_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PORTD_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PORTE_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SWI_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC0_COMPARE_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC0_HOME_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC0_WDOG_SAB_IRQHandler           (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC0_INDEX_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADCB_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_ORed_Message_buffer_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Bus_Off_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Tx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Rx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Wake_Up_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_CMP0_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_RELOAD0_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_CMP1_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_RELOAD1_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_CMP2_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_RELOAD2_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_CMP3_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_RELOAD3_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_CAP_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_RERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWMA_FAULT_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_ORed_Message_buffer_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Bus_Off_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Tx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Rx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Wake_Up_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));




// Declaration of IRQ handler pointer type
using IRQ_HandlerPtr = void (*)(void);

__attribute__ ((used, section(".isr_vector")))
const IRQ_HandlerPtr __vectors[] =
{
    // Core level (CM4) exception handlers
    (IRQ_HandlerPtr)(&__STACK_TOP),         // Initial stack pointer
    Reset_Handler,                          // Reset handler (application entry)
    NMI_Handler,                            // NMI handler
    HardFault_Handler,                      // Hard fault handler
    MemManage_Handler,                      // MPU fault handler
    BusFault_Handler,                       // Bus fault handler
    UsageFault_Handler,                     // Usage fault handler
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    nullptr,                                // RESERVED
    SVC_Handler,                            // SVCall handler
    DebugMon_Handler,                       // Debug monitor handler
    nullptr,                                // RESERVED
    PendSV_Handler,                         // PendSV handler
    SysTick_Handler,                        // SysTick handler

    // Chip level (KV4x) peripheral handlers
    DMA0_IRQHandler,                        // 16 : DMA channel 0, 16 transfer complete
    DMA1_IRQHandler,                        // 17 : DMA channel 1, 17 transfer complete
    DMA2_IRQHandler,                        // 18 : DMA channel 2, 18 transfer complete
    DMA3_IRQHandler,                        // 19 : DMA channel 3, 19 transfer complete
    DMA4_IRQHandler,                        // 20 : DMA channel 4, 20 transfer complete
    DMA5_IRQHandler,                        // 21 : DMA channel 5, 21 transfer complete
    DMA6_IRQHandler,                        // 22 : DMA channel 6, 22 transfer complete
    DMA7_IRQHandler,                        // 23 : DMA channel 7, 23 transfer complete
    DMA8_IRQHandler,                        // 24 : DMA channel 8, 24 transfer complete
    DMA9_IRQHandler,                        // 25 : DMA channel 9, 25 transfer complete
    DMA10_IRQHandler,                       // 26 : DMA channel 10, 26 transfer complete
    DMA11_IRQHandler,                       // 27 : DMA channel 11, 27 transfer complete
    DMA12_IRQHandler,                       // 28 : DMA channel 12, 28 transfer complete
    DMA13_IRQHandler,                       // 29 : DMA channel 13, 29 transfer complete
    DMA14_IRQHandler,                       // 30 : DMA channel 14, 30 transfer complete
    DMA15_IRQHandler,                       // 31 : DMA channel 15, 31 transfer complete
    DMA_Error_IRQHandler,                   // 32 : DMA error interrupt channels 0-1531
    MCM_IRQHandler,                         // 33 : MCM interrupt
    FTFA_IRQHandler,                        // 34 : Command complete
    FTFA_Collision_IRQHandler,              // 35 : Read collision
    PMC_IRQHandler,                         // 36 : Low-voltage detect, low-voltage warning
    LLWU_IRQHandler,                        // 37 : Low Leakage Wakeup
    WDOG_EWM_IRQHandler,                    // 38 : Both watchdog modules share this interrupt
    nullptr,                                // 39 : RESERVED
    I2C0_IRQHandler,                        // 40 : I2C0
    nullptr,                                // 41 : RESERVED
    SPI0_IRQHandler,                        // 42 : SPI0
    nullptr,                                // 43 : RESERVED
    nullptr,                                // 44 : RESERVED
    nullptr,                                // 45 : RESERVED
    nullptr,                                // 46 : RESERVED
    UART0_RX_TX_IRQHandler,                 // 47 : UART0 status sources
    UART0_ERR_IRQHandler,                   // 48 : UART0 error sources
    UART1_RX_TX_IRQHandler,                 // 49 : UART1 status sources
    UART1_ERR_IRQHandler,                   // 50 : UART1 error sources
    nullptr,                                // 51 : RESERVED
    nullptr,                                // 52 : RESERVED
    nullptr,                                // 53 : RESERVED
    ADC_ERR_IRQHandler,                     // 54 : ADC_ERR A and B ( zero cross, high/low limit)
    ADCA_IRQHandler,                        // 55 : ADCA Scan complete
    CMP0_IRQHandler,                        // 56 : CMP0
    CMP1_IRQHandler,                        // 57 : CMP1
    FTM0_IRQHandler,                        // 58 : FTM0 8 channels
    FTM1_IRQHandler,                        // 59 : FTM1 2 channels
    nullptr,                                // 60 : RESERVED
    nullptr,                                // 61 : RESERVED
    nullptr,                                // 62 : RESERVED
    nullptr,                                // 63 : RESERVED
    PIT0_IRQHandler,                        // 64 : PIT Channel 0
    PIT1_IRQHandler,                        // 65 : PIT Channel 1
    PIT2_IRQHandler,                        // 66 : PIT Channel 2
    PIT3_IRQHandler,                        // 67 : PIT Channel 3
    PDB0_IRQHandler,                        // 68 : PDB0
    nullptr,                                // 69 : RESERVED
    XBARA_IRQHandler,                       // 70 : XBARA
    PDB1_IRQHandler,                        // 71 : PDB1
    DAC0_IRQHandler,                        // 72 : DAC0
    MCG_IRQHandler,                         // 73 : MCG
    LPTMR0_IRQHandler,                      // 74 : LPTMR0
    PORTA_IRQHandler,                       // 75 : Pin detect (Port A)
    PORTB_IRQHandler,                       // 76 : Pin detect (Port B)
    PORTC_IRQHandler,                       // 77 : Pin detect (Port C)
    PORTD_IRQHandler,                       // 78 : Pin detect (Port D)
    PORTE_IRQHandler,                       // 79 : Pin detect (Port E)
    SWI_IRQHandler,                         // 80 : Software
    nullptr,                                // 81 : RESERVED
    ENC0_COMPARE_IRQHandler,                // 82 : ENC0 Compare
    ENC0_HOME_IRQHandler,                   // 83 : ENC0 Home
    ENC0_WDOG_SAB_IRQHandler,               // 84 : ENC0 Watchdog/Simultaneous A and B change
    ENC0_INDEX_IRQHandler,                  // 85 : ENC0 Index/Roll over/Roll Under
    CMP2_IRQHandler,                        // 86 : CMP2
    FTM3_IRQHandler,                        // 87 : FTM3 8 channels
    nullptr,                                // 88 : RESERVED
    ADCB_IRQHandler,                        // 89 : ADCB Scan complete
    nullptr,                                // 90 : RESERVED
    CAN0_ORed_Message_buffer_IRQHandler,    // 91 : FLexCAN0 OR'ed Message buffer (0-15)
    CAN0_Bus_Off_IRQHandler,                // 92 : FLexCAN0 Bus Off
    CAN0_Error_IRQHandler,                  // 93 : FLexCAN0 Error
    CAN0_Tx_Warning_IRQHandler,             // 94 : FLexCAN0 Transmit Warning
    CAN0_Rx_Warning_IRQHandler,             // 95 : FLexCAN0 Receive Warning
    CAN0_Wake_Up_IRQHandler,                // 96 : FLexCAN0 Wake Up
    PWMA_CMP0_IRQHandler,                   // 97 : eFlexPWM submodule 0 Compare
    PWMA_RELOAD0_IRQHandler,                // 98 : eFlexPWM submodule 0 Reload
    PWMA_CMP1_IRQHandler,                   // 99 : eFlexPWM submodule 1 Compare
    PWMA_RELOAD1_IRQHandler,                // 100: eFlexPWM submodule 1 Reload
    PWMA_CMP2_IRQHandler,                   // 101: eFlexPWM submodule 2 Compare
    PWMA_RELOAD2_IRQHandler,                // 102: eFlexPWM submodule 2 Reload
    PWMA_CMP3_IRQHandler,                   // 103: eFlexPWM submodule 3 Compare
    PWMA_RELOAD3_IRQHandler,                // 104: eFlexPWM submodule 3 Reload
    PWMA_CAP_IRQHandler,                    // 105: eFlexPWM all input captures
    PWMA_RERR_IRQHandler,                   // 106: eFlexPWM reload error
    PWMA_FAULT_IRQHandler,                  // 107: eFlexPWM Fault
    CMP3_IRQHandler,                        // 108: CMP3
    nullptr,                                // 109: RESERVED
    CAN1_ORed_Message_buffer_IRQHandler,    // 110: FLexCAN1 OR'ed Message buffer (0-15)
    CAN1_Bus_Off_IRQHandler,                // 111: FLexCAN1 Bus Off
    CAN1_Error_IRQHandler,                  // 112: FLexCAN1 Error
    CAN1_Tx_Warning_IRQHandler,             // 113: FLexCAN1 Transmit Warning
    CAN1_Rx_Warning_IRQHandler,             // 114: FLexCAN1 Receive Warning
    CAN1_Wake_Up_IRQHandler,                // 115: FLexCAN1 Wake Up
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

__attribute__ ((weak, section(".after_vectors")))
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

__attribute__ ((section(".after_vectors.reset")))
void Reset_Handler(void)
{
    mcu_startup();
}

__attribute__ ((weak, section(".after_vectors")))
void NMI_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void HardFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void MemManage_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void BusFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void UsageFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void SVC_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void DebugMon_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void PendSV_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((weak, section(".after_vectors")))
void SysTick_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}




} // extern "C"

#endif // __KV4X__
