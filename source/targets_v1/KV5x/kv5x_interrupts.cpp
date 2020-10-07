// ----------------------------------------------------------------------------
// @file    kv5x_interrupts.cpp
// @brief   IRQ handlers and vector table for Kinetis KV5x MCUs.
// @date    11 September 2020
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

#ifdef __KV5X__

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
void ResetISR(void) __attribute__ ((noreturn, alias("Reset_Handler")));
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

// Core level (CM7) exception handlers
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

// Chip level (KV5x) peripheral handlers
void DMA0_DMA16_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA1_DMA17_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA2_DMA18_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA3_DMA19_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA4_DMA20_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA5_DMA21_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA6_DMA22_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA7_DMA23_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA8_DMA24_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA9_DMA25_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA10_DMA26_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA11_DMA27_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA12_DMA28_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA13_DMA29_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA14_DMA30_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA15_DMA31_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void DMA_Error_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void MCM_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTFE_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void Read_Collision_IRQHandler          (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PMC_IRQHandler                     (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void LLWU_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void WDOG_EWM_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void TRNG0_IRQHandler                   (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void I2C1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SPI0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void SPI1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART5_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART5_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART0_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART1_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART2_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART2_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ADC0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void HSADC_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void HSADC0_CCA_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART3_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART3_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART4_RX_TX_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void UART4_ERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT1_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PIT3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PDB0_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
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
void SPI2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC_COMPARE_IRQHandler             (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC_HOME_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC_WDOG_SAB_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENC_INDEX_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP2_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void FTM3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void HSADC0_CCB_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void HSADC1_CCA_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_ORed_Message_buffer_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Bus_Off_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Tx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Rx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN0_Wake_Up_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_CMP0_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_RELOAD0_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_CMP1_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_RELOAD1_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_CMP2_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_RELOAD2_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_CMP3_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_RELOAD3_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_CAP_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_RERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM0_FAULT_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CMP3_IRQHandler                    (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void HSADC1_CCB_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_ORed_Message_buffer_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Bus_Off_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Tx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Rx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN1_Wake_Up_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENET_1588_Timer_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENET_Transmit_IRQHandler           (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENET_Receive_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void ENET_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_CMP0_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_RELOAD0_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_CMP1_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_RELOAD1_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_CMP2_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_RELOAD2_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_CMP3_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_RELOAD3_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_CAP_IRQHandler                (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_RERR_IRQHandler               (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void PWM1_FAULT_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_ORed_Message_buffer_IRQHandler(void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_Bus_Off_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_Error_IRQHandler              (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_Tx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_Rx_Warning_IRQHandler         (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));
void CAN2_Wake_Up_IRQHandler            (void) __attribute__ ((weak, alias("IRQ_DefaultHandler")));




// Declaration of IRQ handler pointer type
using IRQ_HandlerPtr = void (*)(void);

__attribute__ ((used, section(".isr_vector")))
const IRQ_HandlerPtr __vectors[] =
{
    // Core level (CM7) exception handlers
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

    // Chip level (KV5x) peripheral handlers
    DMA0_DMA16_IRQHandler,                  // 16 : DMA channel 0/16 transfer complete
    DMA1_DMA17_IRQHandler,                  // 17 : DMA channel 1/17 transfer complete
    DMA2_DMA18_IRQHandler,                  // 18 : DMA channel 2/18 transfer complete
    DMA3_DMA19_IRQHandler,                  // 19 : DMA channel 3/19 transfer complete
    DMA4_DMA20_IRQHandler,                  // 20 : DMA channel 4/20 transfer complete
    DMA5_DMA21_IRQHandler,                  // 21 : DMA channel 5/21 transfer complete
    DMA6_DMA22_IRQHandler,                  // 22 : DMA channel 6/22 transfer complete
    DMA7_DMA23_IRQHandler,                  // 23 : DMA channel 7/23 transfer complete
    DMA8_DMA24_IRQHandler,                  // 24 : DMA channel 8/24 transfer complete
    DMA9_DMA25_IRQHandler,                  // 25 : DMA channel 9/25 transfer complete
    DMA10_DMA26_IRQHandler,                 // 26 : DMA channel 10/26 transfer complete
    DMA11_DMA27_IRQHandler,                 // 27 : DMA channel 11/27 transfer complete
    DMA12_DMA28_IRQHandler,                 // 28 : DMA channel 12/28 transfer complete
    DMA13_DMA29_IRQHandler,                 // 29 : DMA channel 13/29 transfer complete
    DMA14_DMA30_IRQHandler,                 // 30 : DMA channel 14/30 transfer complete
    DMA15_DMA31_IRQHandler,                 // 31 : DMA channel 15/31 transfer complete
    DMA_Error_IRQHandler,                   // 32 : DMA error interrupt channels 0-31
    MCM_IRQHandler,                         // 33 : MCM normal interrupt
    FTFE_IRQHandler,                        // 34 : FTFL command complete
    Read_Collision_IRQHandler,              // 35 : FTFL read collision
    PMC_IRQHandler,                         // 36 : PMC controller low-voltage detect, low-voltage warning
    LLWU_IRQHandler,                        // 37 : Low leakage wakeup
    WDOG_EWM_IRQHandler,                    // 38 : Single interrupt vector for  WDOG and EWM
    TRNG0_IRQHandler,                       // 39 : True randon number generator
    I2C0_IRQHandler,                        // 40 : Inter-integrated circuit 0
    I2C1_IRQHandler,                        // 41 : Inter-integrated circuit 1
    SPI0_IRQHandler,                        // 42 : Serial peripheral Interface 0
    SPI1_IRQHandler,                        // 43 : Serial peripheral Interface 1
    UART5_RX_TX_IRQHandler,                 // 44 : UART5 receive/transmit interrupt
    UART5_ERR_IRQHandler,                   // 45 : UART5 error interrupt
    nullptr,                                // 46 : Reserved interrupt
    UART0_RX_TX_IRQHandler,                 // 47 : UART0 receive/transmit interrupt
    UART0_ERR_IRQHandler,                   // 48 : UART0 error interrupt
    UART1_RX_TX_IRQHandler,                 // 49 : UART1 receive/transmit interrupt
    UART1_ERR_IRQHandler,                   // 50 : UART1 error interrupt
    UART2_RX_TX_IRQHandler,                 // 51 : UART2 receive/transmit interrupt
    UART2_ERR_IRQHandler,                   // 52 : UART2 error interrupt
    ADC0_IRQHandler,                        // 53 : Analog-to-digital converter 0
    HSADC_ERR_IRQHandler,                   // 54 : High speed analog-to-digital converter zero cross
    HSADC0_CCA_IRQHandler,                  // 55 : High speed analog-to-digital converter 0 submodule A scan complete
    CMP0_IRQHandler,                        // 56 : Comparator 0
    CMP1_IRQHandler,                        // 57 : Comparator 1
    FTM0_IRQHandler,                        // 58 : FlexTimer module 0 fault, overflow and channels interrupt
    FTM1_IRQHandler,                        // 59 : FlexTimer module 1 fault, overflow and channels interrupt
    UART3_RX_TX_IRQHandler,                 // 60 : UART3 receive/transmit interrupt
    UART3_ERR_IRQHandler,                   // 61 : UART3 error interrupt
    UART4_RX_TX_IRQHandler,                 // 62 : UART4 receive/transmit interrupt
    UART4_ERR_IRQHandler,                   // 63 : UART4 error interrupt
    PIT0_IRQHandler,                        // 64 : Periodic interrupt timer channel 0
    PIT1_IRQHandler,                        // 65 : Periodic interrupt timer channel 1
    PIT2_IRQHandler,                        // 66 : Periodic interrupt timer channel 2
    PIT3_IRQHandler,                        // 67 : Periodic interrupt timer channel 3
    PDB0_IRQHandler,                        // 68 : Programmable delay block 0
    FTM2_IRQHandler,                        // 69 : FlexTimer module 2 fault, overflow and channels interrupt
    XBARA_IRQHandler,                       // 70 : Inter-peripheral crossbar switch A
    PDB1_IRQHandler,                        // 71 : Programmable delay block 1
    DAC0_IRQHandler,                        // 72 : Digital-to-analog converter 0
    MCG_IRQHandler,                         // 73 : Multipurpose clock generator
    LPTMR0_IRQHandler,                      // 74 : Low power timer interrupt
    PORTA_IRQHandler,                       // 75 : Port A interrupt
    PORTB_IRQHandler,                       // 76 : Port B interrupt
    PORTC_IRQHandler,                       // 77 : Port C interrupt
    PORTD_IRQHandler,                       // 78 : Port D interrupt
    PORTE_IRQHandler,                       // 79 : Port E interrupt
    SWI_IRQHandler,                         // 80 : Software interrupt
    SPI2_IRQHandler,                        // 81 : Serial peripheral Interface 2
    ENC_COMPARE_IRQHandler,                 // 82 : ENC Compare
    ENC_HOME_IRQHandler,                    // 83 : ENC Home
    ENC_WDOG_SAB_IRQHandler,                // 84 : ENC Wdog/SAB
    ENC_INDEX_IRQHandler,                   // 85 : ENC Index/Roll over/Roll Under
    CMP2_IRQHandler,                        // 86 : Comparator 2
    FTM3_IRQHandler,                        // 87 : FlexTimer module 3 fault, overflow and channels
    nullptr,                                // 88 : Reserved interrupt
    HSADC0_CCB_IRQHandler,                  // 89 : High speed analog-to-digital converter 0 submodule B scan complete
    HSADC1_CCA_IRQHandler,                  // 90 : High speed analog-to-digital converter 1 submodule A scan complete
    CAN0_ORed_Message_buffer_IRQHandler,    // 91 : Flex controller area network 0 message buffer
    CAN0_Bus_Off_IRQHandler,                // 92 : Flex controller area network 0 bus off
    CAN0_Error_IRQHandler,                  // 93 : Flex controller area network 0 error
    CAN0_Tx_Warning_IRQHandler,             // 94 : Flex controller area network 0 transmit
    CAN0_Rx_Warning_IRQHandler,             // 95 : Flex controller area network 0 receive
    CAN0_Wake_Up_IRQHandler,                // 96 : Flex controller area network 0 wake up
    PWM0_CMP0_IRQHandler,                   // 97 : Pulse width modulator 0 channel 0 compare
    PWM0_RELOAD0_IRQHandler,                // 98 : Pulse width modulator 0 channel 0 reload
    PWM0_CMP1_IRQHandler,                   // 99 : Pulse width modulator 0 channel 1 compare
    PWM0_RELOAD1_IRQHandler,                // 100: Pulse width modulator 0 channel 1 reload
    PWM0_CMP2_IRQHandler,                   // 101: Pulse width modulator 0 channel 2 compare
    PWM0_RELOAD2_IRQHandler,                // 102: Pulse width modulator 0 channel 2 reload
    PWM0_CMP3_IRQHandler,                   // 103: Pulse width modulator 0 channel 3 compare
    PWM0_RELOAD3_IRQHandler,                // 104: Pulse width modulator 0 channel 3 reload
    PWM0_CAP_IRQHandler,                    // 105: Pulse width modulator 0 capture
    PWM0_RERR_IRQHandler,                   // 106: Pulse width modulator 0 reload error
    PWM0_FAULT_IRQHandler,                  // 107: Pulse width modulator 0 fault
    CMP3_IRQHandler,                        // 108: Comparator 3
    HSADC1_CCB_IRQHandler,                  // 109: High speed analog-to-digital converter 1 submodule B scan complete
    CAN1_ORed_Message_buffer_IRQHandler,    // 110: Flex controller area network 1 message buffer
    CAN1_Bus_Off_IRQHandler,                // 111: Flex controller area network 1 bus off
    CAN1_Error_IRQHandler,                  // 112: Flex controller area network 1 error
    CAN1_Tx_Warning_IRQHandler,             // 113: Flex controller area network 1 transmit
    CAN1_Rx_Warning_IRQHandler,             // 114: Flex controller area network 1 receive
    CAN1_Wake_Up_IRQHandler,                // 115: Flex controller area network 1 wake up
    ENET_1588_Timer_IRQHandler,             // 116: Ethernet MAC IEEE 1588 timer
    ENET_Transmit_IRQHandler,               // 117: Ethernet MAC transmit
    ENET_Receive_IRQHandler,                // 118: Ethernet MAC receive
    ENET_Error_IRQHandler,                  // 119: Ethernet MAC error and miscelaneous
    PWM1_CMP0_IRQHandler,                   // 120: Pulse width modulator 1 channel 0 compare
    PWM1_RELOAD0_IRQHandler,                // 121: Pulse width modulator 1 channel 0 reload
    PWM1_CMP1_IRQHandler,                   // 122: Pulse width modulator 1 channel 1 compare
    PWM1_RELOAD1_IRQHandler,                // 123: Pulse width modulator 1 channel 1 reload
    PWM1_CMP2_IRQHandler,                   // 124: Pulse width modulator 1 channel 2 compare
    PWM1_RELOAD2_IRQHandler,                // 125: Pulse width modulator 1 channel 2 reload
    PWM1_CMP3_IRQHandler,                   // 126: Pulse width modulator 1 channel 3 compare
    PWM1_RELOAD3_IRQHandler,                // 127: Pulse width modulator 1 channel 3 reload
    PWM1_CAP_IRQHandler,                    // 128: Pulse width modulator 1 capture
    PWM1_RERR_IRQHandler,                   // 129: Pulse width modulator 1 reload error
    PWM1_FAULT_IRQHandler,                  // 130: Pulse width modulator 1 fault
    CAN2_ORed_Message_buffer_IRQHandler,    // 131: Flex controller area network 2 message buffer
    CAN2_Bus_Off_IRQHandler,                // 132: Flex controller area network 2 bus off
    CAN2_Error_IRQHandler,                  // 133: Flex controller area network 2 error
    CAN2_Tx_Warning_IRQHandler,             // 134: Flex controller area network 2 transmit
    CAN2_Rx_Warning_IRQHandler,             // 135: Flex controller area network 2 receive
    CAN2_Wake_Up_IRQHandler,                // 136: Flex controller area network 2 wake up
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

__attribute__ ((section(".after_vectors"), weak))
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

__attribute__ ((section(".after_vectors"), noreturn))
void Reset_Handler(void)
{
    mcu_startup();
}

__attribute__ ((section(".after_vectors"), weak))
void NMI_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void HardFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void MemManage_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void BusFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void UsageFault_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void SVC_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void DebugMon_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void PendSV_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}

__attribute__ ((section(".after_vectors"), weak))
void SysTick_Handler(void)
{
    __DEBUG_BKPT();

    while(true)
    {}
}




} // extern "C"

#endif // __KV5X__
