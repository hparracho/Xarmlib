// ----------------------------------------------------------------------------
// @file    lpc84x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for NXP LPC84x MCUs.
// @date    29 June 2018
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

// ----------------------------------------------------------------------------
// This file is based on the code from LPC845 Example Code Bundle for
// MCUXpresso REV 1.2 provided by NXP.
// https://www.nxp.com/downloads/en/software/LPC845-Example-Code-Bundle-MCUXpresso.zip
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_TARGETS_LPC84X_CMSIS_HPP
#define __XARMLIB_TARGETS_LPC84X_CMSIS_HPP

#include "cmsis_compiler.h"
#include "cmsis_version.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// ISO C++ prohibits anonymous structs [-Wpedantic]




// ----------------------------------------------------------------------------
// Interrupt Number Definition
// ----------------------------------------------------------------------------
typedef enum IRQn
{
    // Cortex-M0+ Processor Exceptions Numbers
    Reset_IRQn                  = -15,      // 1 Reset Vector, invoked on Power up and warm reset
    NonMaskableInt_IRQn         = -14,      // 2 Non Maskable interrupt
    HardFault_IRQn              = -13,      // 3 Cortex-M0 Hard Fault interrupt
    SVCall_IRQn                 = -5,       // 11 Cortex-M0 SV Call interrupt
    PendSV_IRQn                 = -2,       // 14 Cortex-M0 Pend SV interrupt
    SysTick_IRQn                = -1,       // 15 Cortex-M0 System Tick interrupt

    // LPC84x Specific Interrupt Numbers
    SPI0_IRQn                   = 0,        // SPI0 interrupt
    SPI1_IRQn                   = 1,        // SPI1 interrupt
    DAC0_IRQn                   = 2,        // DAC0 Interrupt
    USART0_IRQn                 = 3,        // USART0 interrupt
    USART1_IRQn                 = 4,        // USART1 interrupt
    USART2_IRQn                 = 5,        // USART2 interrupt
                                            // RESERVED
    I2C1_IRQn                   = 7,        // I2C1 interrupt
    I2C0_IRQn                   = 8,        // I2C0 interrupt
    SCT_IRQn                    = 9,        // SCT interrupt
    MRT_IRQn                    = 10,       // MRT interrupt
    CMP_CAPT_IRQn               = 11,       // Analog Comparator / Cap Touch shared interrupt
    WDT_IRQn                    = 12,       // WDT interrupt
    BOD_IRQn                    = 13,       // BOD interrupt
    FLASH_IRQn                  = 14,       // FLASH interrupt
    WKT_IRQn                    = 15,       // WKT interrupt
    ADC_SEQA_IRQn               = 16,       // ADC sequence A completion interrupt
    ADC_SEQB_IRQn               = 17,       // ADC sequence B completion interrupt
    ADC_THCMP_IRQn              = 18,       // ADC threshold compare interrupt
    ADC_OVR_IRQn                = 19,       // ADC overrun interrupt
    DMA_IRQn                    = 20,       // DMA interrupt
    I2C2_IRQn                   = 21,       // I2C2 interrupt
    I2C3_IRQn                   = 22,       // I2C3 interrupt
    CTIMER_IRQn                 = 23,       // Standard Counter / Timer interrupt
    PININT0_IRQn                = 24,       // Pin Interrupt 0
    PININT1_IRQn                = 25,       // Pin Interrupt 1
    PININT2_IRQn                = 26,       // Pin Interrupt 2
    PININT3_IRQn                = 27,       // Pin Interrupt 3
    PININT4_IRQn                = 28,       // Pin Interrupt 4
    PININT5_DAC1_IRQn           = 29,       // Pin Interrupt 5 / DAC1 shared interrupt
    PININT6_USART3_IRQn         = 30,       // Pin Interrupt 6 / USART3 shared interrupt
    PININT7_USART4_IRQn         = 31,       // PIn Interrupt 7 / USART4 shared interrupt
} IRQn_Type;




// ----------------------------------------------------------------------------
// Processor and Core Peripheral Section
// ----------------------------------------------------------------------------

// Configuration of the Cortex-M0+ Processor and Core Peripherals
#define __CM0PLUS_REV           0x0001      // Core revision r0p1
#define __MPU_PRESENT           0           // Defines if an MPU is present or not
#define __VTOR_PRESENT          1           // Defines if VTOR is present or not
#define __NVIC_PRIO_BITS        2           // Number of priority bits implemented in the NVIC
#define __Vendor_SysTickConfig  0           // Vendor specific implementation of SysTickConfig is defined




extern uint32_t SystemCoreClock;            // CMSIS system core clock variable definition

// Update system core clock frequency
// NOTE: This function is called in startup functions but should be
//       called every time the system has a clock frequency change.
void SystemCoreClockUpdate(void);




#include "core_cm0plus.h"                   // Cortex-M0+ processor and core peripherals




// ----------------------------------------------------------------------------
// Device Specific Peripheral Registers structures
// ----------------------------------------------------------------------------



// ------------ System Control (SYSCON) ---------------------------------------
typedef struct
{
    __IO uint32_t SYSMEMREMAP;              // (offset: 0x000) System memory remap
         uint32_t RESERVED0;                // (offset: 0x004) RESERVED
    __IO uint32_t SYSPLLCTRL;               // (offset: 0x008) System PLL control
    __I  uint32_t SYSPLLSTAT;               // (offset: 0x00C) System PLL status
         uint32_t RESERVED1[4];             // (offset: 0x010) RESERVED
    __IO uint32_t SYSOSCCTRL;               // (offset: 0x020) System oscillator control
    __IO uint32_t WDTOSCCTRL;               // (offset: 0x024) Watchdog oscillator control
    __IO uint32_t FROOSCCTRL;               // (offset: 0x028) FRO oscillator control
         uint32_t RESERVED2;                // (offset: 0x02C) RESERVED
    __IO uint32_t FRODIRECTCLKUEN;          // (offset: 0x030) FRO direct clock source update
         uint32_t RESERVED3;                // (offset: 0x034) RESERVED
    __IO uint32_t SYSRSTSTAT;               // (offset: 0x038) System reset status 0
    __IO uint32_t FAIMROWPROTECTCTRL;       // (offset: 0x03C) FAIM row protect control
    __IO uint32_t SYSPLLCLKSEL;             // (offset: 0x040) System PLL clock source select 0
    __IO uint32_t SYSPLLCLKUEN;             // (offset: 0x044) System PLL clock source update
    __IO uint32_t MAINCLKPLLSEL;            // (offset: 0x048) Main clock pll source select 0
    __IO uint32_t MAINCLKPLLUEN;            // (offset: 0x04C) Main clock pll source update enable
    __IO uint32_t MAINCLKSEL;               // (offset: 0x050) Main clock source select
    __IO uint32_t MAINCLKUEN;               // (offset: 0x054) Main clock source update enable
    __IO uint32_t SYSAHBCLKDIV;             // (offset: 0x058) System clock divider
         uint32_t RESERVED4;                // (offset: 0x05C) RESERVED
    __IO uint32_t CAPTCLKSEL;               // (offset: 0x060) CAPT clock source select
    __IO uint32_t ADCCLKSEL;                // (offset: 0x064) ADC clock source select
    __IO uint32_t ADCCLKDIV;                // (offset: 0x068) ADC clock divider
    __IO uint32_t SCTCLKSEL;                // (offset: 0x06C) SCT clock source select
    __IO uint32_t SCTCLKDIV;                // (offset: 0x070) SCT clock divider
    __IO uint32_t EXTCLKSEL;                // (offset: 0x074) External clock source select
         uint32_t RESERVED5[2];             // (offset: 0x078) RESERVED
    union
    {
        __IO uint32_t SYSAHBCLKCTRL[2];
        struct
        {
            __IO uint32_t SYSAHBCLKCTRL0;   // (offset: 0x080) System clock group 0 control
            __IO uint32_t SYSAHBCLKCTRL1;   // (offset: 0x084) System clock group 1 control
        };
    };
    union
    {
        __IO uint32_t PRESETCTRL[2];
        struct
        {
            __IO uint32_t PRESETCTRL0;      // (offset: 0x088) Peripheral reset group 0 control
            __IO uint32_t PRESETCTRL1;      // (offset: 0x08C) Peripheral reset group 1 control
        };
    };
    union
    {
        __IO uint32_t FCLKSEL[11];
        struct
        {
            __IO uint32_t UART0CLKSEL;      // (offset: 0x090) FCLK0 clock source select
            __IO uint32_t UART1CLKSEL;      // (offset: 0x094) FCLK1 clock source select
            __IO uint32_t UART2CLKSEL;      // (offset: 0x098) FCLK2 clock source select
            __IO uint32_t UART3CLKSEL;      // (offset: 0x09C) FCLK3 clock source select
            __IO uint32_t UART4CLKSEL;      // (offset: 0x0A0) FCLK4 clock source select
            __IO uint32_t I2C0CLKSEL;       // (offset: 0x0A4) FCLK5 clock source select
            __IO uint32_t I2C1CLKSEL;       // (offset: 0x0A8) FCLK6 clock source select
            __IO uint32_t I2C2CLKSEL;       // (offset: 0x0AC) FCLK7 clock source select
            __IO uint32_t I2C3CLKSEL;       // (offset: 0x0B0) FCLK8 clock source select
            __IO uint32_t SPI0CLKSEL;       // (offset: 0x0B4) FCLK9 clock source select
            __IO uint32_t SPI1CLKSEL;       // (offset: 0x0B8) FCLK10 clock source select
        };
    };
         uint32_t RESERVED6;                // (offset: 0x0BC) RESERVED
    __IO uint32_t EFLASHREFCLKDIV;          // (offset: 0x0C0) EFLASH REF clock divider
    __IO uint32_t FAIMREFCLKDIV;            // (offset: 0x0C4) FAIM REF clock divider
         uint32_t RESERVED7[2];             // (offset: 0x0C8) RESERVED
    __IO uint32_t FRG0DIV;                  // (offset: 0x0D0) FRG0 divider value
    __IO uint32_t FRG0MULT;                 // (offset: 0x0D4) FRG0 multiplier value
    __IO uint32_t FRG0CLKSEL;               // (offset: 0x0D8) FRG0 clock source select
         uint32_t RESERVED8;                // (offset: 0x0DC) RESERVED
    __IO uint32_t FRG1DIV;                  // (offset: 0x0E0) FRG1 divider value
    __IO uint32_t FRG1MULT;                 // (offset: 0x0E4) FRG1 multiplier value
    __IO uint32_t FRG1CLKSEL;               // (offset: 0x0E8) FRG1 clock source select
         uint32_t RESERVED9;                // (offset: 0x0EC) RESERVED
    __IO uint32_t CLKOUTSEL;                // (offset: 0x0F0) CLKOUT clock source select
    __IO uint32_t CLKOUTDIV;                // (offset: 0x0F4) CLKOUT clock divider
         uint32_t RESERVED10;               // (offset: 0x0F8) RESERVED
    __IO uint32_t EXTTRACECMD;              // (offset: 0x0FC) External trace buffer command
    __I  uint32_t PIOPORCAP0;               // (offset: 0x100) POR captured PIO0 status 0
    __I  uint32_t PIOPORCAP1;               // (offset: 0x104) POR captured PIO1 status 0
         uint32_t RESERVED11[11];           // (offset: 0x108) RESERVED
    union
    {
        __IO uint32_t IOCONCLKDIV[7];
        struct
        {
            __IO uint32_t IOCONCLKDIV6;     // (offset: 0x134) Peripheral clock 6 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV4;     // (offset: 0x13C) Peripheral clock 4 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV5;     // (offset: 0x138) Peripheral clock 5 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV3;     // (offset: 0x140) Peripheral clock 3 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV2;     // (offset: 0x144) Peripheral clock 2 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV1;     // (offset: 0x148) Peripheral clock 1 to the IOCON block for programmable glitch filter
            __IO uint32_t IOCONCLKDIV0;     // (offset: 0x14C) Peripheral clock 0 to the IOCON block for programmable glitch filter
        };
    };
    __IO uint32_t BODCTRL;                  // (offset: 0x150) Brown-Out Detect
    __IO uint32_t SYSTCKCAL;                // (offset: 0x154) System tick counter calibration
         uint32_t RESERVED12[6];            // (offset: 0x158) RESERVED
    __IO uint32_t IRQLATENCY;               // (offset: 0x170) IRQ delay. Allows trade-off between interrupt latency and determinism.
    __IO uint32_t NMISRC;                   // (offset: 0x174) NMI Source Control
    union
    {
        __IO uint32_t PINTSEL[8];
        struct
        {
            __IO uint32_t PINTSEL0;         // (offset: 0x178) GPIO Pin Interrupt Select 0
            __IO uint32_t PINTSEL1;         // (offset: 0x17C) GPIO Pin Interrupt Select 1
            __IO uint32_t PINTSEL2;         // (offset: 0x180) GPIO Pin Interrupt Select 2
            __IO uint32_t PINTSEL3;         // (offset: 0x184) GPIO Pin Interrupt Select 3
            __IO uint32_t PINTSEL4;         // (offset: 0x188) GPIO Pin Interrupt Select 4
            __IO uint32_t PINTSEL5;         // (offset: 0x18C) GPIO Pin Interrupt Select 5
            __IO uint32_t PINTSEL6;         // (offset: 0x190) GPIO Pin Interrupt Select 6
            __IO uint32_t PINTSEL7;         // (offset: 0x194) GPIO Pin Interrupt Select 7
        };
    };
         uint32_t RESERVED13[27];           // (offset: 0x198) RESERVED
    __IO uint32_t STARTERP0;                // (offset: 0x204) Start logic 0 pin wake-up enable
         uint32_t RESERVED14[3];            // (offset: 0x208) RESERVED
    __IO uint32_t STARTERP1;                // (offset: 0x214) Start logic 1 interrupt wake-up enable
         uint32_t RESERVED15[6];            // (offset: 0x218) RESERVED
    __IO uint32_t PDSLEEPCFG;               // (offset: 0x230) Power-down states in deep-sleep mode
    __IO uint32_t PDAWAKECFG;               // (offset: 0x234) Power-down states for wake-up from deep-sleep
    __IO uint32_t PDRUNCFG;                 // (offset: 0x238) Power configuration
         uint32_t RESERVED16[111];          // (offset: 0x23C) RESERVED
    __I  uint32_t DEVICE_ID;                // (offset: 0x3F8) Device ID
} LPC_SYSCON_T;




// ------------ I/O Configuration (IOCON) -------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PIO[56];          // Digital I/O control for ports
        struct
        {
            __IO uint32_t PIO0_17;          // (offset: 0x00)
            __IO uint32_t PIO0_13;          // (offset: 0x04)
            __IO uint32_t PIO0_12;          // (offset: 0x08)
            __IO uint32_t PIO0_5;           // (offset: 0x0C)
            __IO uint32_t PIO0_4;           // (offset: 0x10)
            __IO uint32_t PIO0_3;           // (offset: 0x14)
            __IO uint32_t PIO0_2;           // (offset: 0x18)
            __IO uint32_t PIO0_11;          // (offset: 0x1C)
            __IO uint32_t PIO0_10;          // (offset: 0x20)
            __IO uint32_t PIO0_16;          // (offset: 0x24)
            __IO uint32_t PIO0_15;          // (offset: 0x28)
            __IO uint32_t PIO0_1;           // (offset: 0x2C)
                 uint32_t RESERVED0;        // (offset: 0x30) RESERVED
            __IO uint32_t PIO0_9;           // (offset: 0x34)
            __IO uint32_t PIO0_8;           // (offset: 0x38)
            __IO uint32_t PIO0_7;           // (offset: 0x3C)
            __IO uint32_t PIO0_6;           // (offset: 0x40)
            __IO uint32_t PIO0_0;           // (offset: 0x44)
            __IO uint32_t PIO0_14;          // (offset: 0x48)
                 uint32_t RESERVED1;        // (offset: 0x4C) RESERVED
            __IO uint32_t PIO0_28;          // (offset: 0x50)
            __IO uint32_t PIO0_27;          // (offset: 0x54)
            __IO uint32_t PIO0_26;          // (offset: 0x58)
            __IO uint32_t PIO0_25;          // (offset: 0x5C)
            __IO uint32_t PIO0_24;          // (offset: 0x60)
            __IO uint32_t PIO0_23;          // (offset: 0x64)
            __IO uint32_t PIO0_22;          // (offset: 0x68)
            __IO uint32_t PIO0_21;          // (offset: 0x6C)
            __IO uint32_t PIO0_20;          // (offset: 0x70)
            __IO uint32_t PIO0_19;          // (offset: 0x74)
            __IO uint32_t PIO0_18;          // (offset: 0x78)
            __IO uint32_t PIO1_8;           // (offset: 0x7C)
            __IO uint32_t PIO1_9;           // (offset: 0x80)
            __IO uint32_t PIO1_12;          // (offset: 0x84)
            __IO uint32_t PIO1_13;          // (offset: 0x88)
            __IO uint32_t PIO0_31;          // (offset: 0x8C)
            __IO uint32_t PIO1_0;           // (offset: 0x90)
            __IO uint32_t PIO1_1;           // (offset: 0x94)
            __IO uint32_t PIO1_2;           // (offset: 0x98)
            __IO uint32_t PIO1_14;          // (offset: 0x9C)
            __IO uint32_t PIO1_15;          // (offset: 0xA0)
            __IO uint32_t PIO1_3;           // (offset: 0xA4)
            __IO uint32_t PIO1_4;           // (offset: 0xA8)
            __IO uint32_t PIO1_5;           // (offset: 0xAC)
            __IO uint32_t PIO1_16;          // (offset: 0xB0)
            __IO uint32_t PIO1_17;          // (offset: 0xB4)
            __IO uint32_t PIO1_6;           // (offset: 0xB8)
            __IO uint32_t PIO1_18;          // (offset: 0xBC)
            __IO uint32_t PIO1_19;          // (offset: 0xC0)
            __IO uint32_t PIO1_7;           // (offset: 0xC4)
            __IO uint32_t PIO0_29;          // (offset: 0xC8)
            __IO uint32_t PIO0_30;          // (offset: 0xCC)
            __IO uint32_t PIO1_20;          // (offset: 0xD0)
            __IO uint32_t PIO1_21;          // (offset: 0xD4)
            __IO uint32_t PIO1_11;          // (offset: 0xD8)
            __IO uint32_t PIO1_10;          // (offset: 0xDC)
        };
    };
} LPC_IOCON_T;




// ------------ FLASH Memory Controller (FMC) ---------------------------------
typedef struct
{
         uint32_t RESERVED0[4];             // (offset: 0x000) RESERVED
    __IO uint32_t FLASHCFG;                 // (offset: 0x010) Flash configuration register
         uint32_t RESERVED1[3];             // (offset: 0x014) RESERVED
    __IO uint32_t FMSSTART;                 // (offset: 0x020) Signature start address register
    __IO uint32_t FMSSTOP;                  // (offset: 0x024) Signature stop-address register
         uint32_t RESERVED2;                // (offset: 0x028) RESERVED
    __I  uint32_t FMSW0;                    // (offset: 0x02C) Signature word
         uint32_t RESERVED3[1004];          // (offset: 0x030) RESERVED
    __I  uint32_t FMSTAT;                   // (offset: 0xFE0) Signature generation status register
         uint32_t RESERVED4;                // (offset: 0xFE4) RESERVED
    __O  uint32_t FMSTATCLR;                // (offset: 0xFE8) Signature generation status clear register
} LPC_FMC_T;




// ------------ Power Management Unit (PMU) -----------------------------------
typedef struct
{
    __IO uint32_t PCON;                     // (offset: 0x00) Power control Register
    union
    {
        __IO uint32_t GPREG[4];
        struct
        {
            __IO uint32_t GPREG0;           // (offset: 0x04) General purpose register 0
            __IO uint32_t GPREG1;           // (offset: 0x08) General purpose register 1
            __IO uint32_t GPREG2;           // (offset: 0x0C) General purpose register 2
            __IO uint32_t GPREG3;           // (offset: 0x10) General purpose register 3
        };
    };
    __IO uint32_t DPDCTRL;                  // (offset: 0x14) Deep power-down control register
} LPC_PMU_T;




// ------------ Switch Matrix (SWM) -------------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PINASSIGN[15];
        struct
        {
            __IO uint32_t PINASSIGN0;       // (offset: 0x000) Pin assign register 0
            __IO uint32_t PINASSIGN1;       // (offset: 0x004) Pin assign register 1
            __IO uint32_t PINASSIGN2;       // (offset: 0x008) Pin assign register 2
            __IO uint32_t PINASSIGN3;       // (offset: 0x00C) Pin assign register 3
            __IO uint32_t PINASSIGN4;       // (offset: 0x010) Pin assign register 4
            __IO uint32_t PINASSIGN5;       // (offset: 0x014) Pin assign register 5
            __IO uint32_t PINASSIGN6;       // (offset: 0x018) Pin assign register 6
            __IO uint32_t PINASSIGN7;       // (offset: 0x01C) Pin assign register 7
            __IO uint32_t PINASSIGN8;       // (offset: 0x020) Pin assign register 8
            __IO uint32_t PINASSIGN9;       // (offset: 0x024) Pin assign register 9
            __IO uint32_t PINASSIGN10;      // (offset: 0x028) Pin assign register 10
            __IO uint32_t PINASSIGN11;      // (offset: 0x02C) Pin assign register 11
            __IO uint32_t PINASSIGN12;      // (offset: 0x030) Pin assign register 12
            __IO uint32_t PINASSIGN13;      // (offset: 0x034) Pin assign register 13
            __IO uint32_t PINASSIGN14;      // (offset: 0x038) Pin assign register 14
        };
    };
         uint32_t RESERVED[97];             // (offset: 0x03C) RESERVED
    __IO uint32_t PINENABLE0;               // (offset: 0x1C0) Pin enable register 0
    __IO uint32_t PINENABLE1;               // (offset: 0x1C4) Pin enable register 0
} LPC_SWM_T;




// ------------ General Purpose I/O (GPIO) ------------------------------------
typedef struct
{
    union
    {
        __IO uint8_t B[2][32];
        struct
        {
            __IO uint8_t  B0[32];           // (offset: 0x0000) Byte pin registers P0.0 - P0.31
            __IO uint8_t  B1[32];           // (offset: 0x0020) Byte pin registers P1.0 - P1.31
        };
    };
    uint32_t RESERVED0[1008];               // (offset: 0x0040) RESERVED
    union
    {
        __IO uint32_t W[2][32];
        struct
        {
            __IO uint32_t W0[32];           // (offset: 0x1000) Word pin registers P0.0 - P0.31
            __IO uint32_t W1[32];           // (offset: 0x1080) Word pin registers P1.0 - P1.31
        };
    };
    uint32_t RESERVED1[960];                // (offset: 0x1100) RESERVED
    union
    {
        __IO uint32_t DIR[2];
        struct
        {
            __IO uint32_t DIR0;             // (offset: 0x2000) Port 0 direction register
            __IO uint32_t DIR1;             // (offset: 0x2004) Port 1 direction register
        };
    };
    uint32_t RESERVED2[30];                 // (offset: 0x2008) RESERVED
    union
    {
        __IO uint32_t MASK[2];
        struct
        {
            __IO uint32_t MASK0;            // (offset: 0x2080) Port 0 mask register
            __IO uint32_t MASK1;            // (offset: 0x2084) Port 1 mask register
        };
    };
    uint32_t RESERVED3[30];                 // (offset: 0x2088) RESERVED
    union
    {
        __IO uint32_t PIN[2];
        struct
        {
            __IO uint32_t PIN0;             // (offset: 0x2100) Port 0 pin register
            __IO uint32_t PIN1;             // (offset: 0x2104) Port 1 pin register
        };
    };
    uint32_t RESERVED4[30];                 // (offset: 0x2108) RESERVED
    union
    {
        __IO uint32_t MPIN[2];
        struct
        {
            __IO uint32_t MPIN0;            // (offset: 0x2180) Masked port 0 register
            __IO uint32_t MPIN1;            // (offset: 0x2184) Masked port 1 register
        };
    };
    uint32_t RESERVED5[30];                 // (offset: 0x2188) RESERVED
    union
    {
        __IO uint32_t SET[2];
        struct
        {
            __IO uint32_t SET0;             // (offset: 0x2200) Set port 0 register
            __IO uint32_t SET1;             // (offset: 0x2204) Set port 1 register
        };
    };
    uint32_t RESERVED6[30];                 // (offset: 0x2208) RESERVED
    union
    {
        __O  uint32_t CLR[2];
        struct
        {
            __O  uint32_t CLR0;             // (offset: 0x2280) Clear port 0 register
            __O  uint32_t CLR1;             // (offset: 0x2284) Clear port 1 register
        };
    };
    uint32_t RESERVED7[30];                 // (offset: 0x2288) RESERVED
    union
    {
        __O  uint32_t NOT[2];
        struct
        {
            __O  uint32_t NOT0;             // (offset: 0x2300) Toggle port 0 register
            __O  uint32_t NOT1;             // (offset: 0x2304) Toggle port 1 register
        };
    };
    uint32_t RESERVED8[30];                 // (offset: 0x2308) RESERVED
    union
    {
        __O  uint32_t DIRSET[2];
        struct
        {
            __O  uint32_t DIRSET0;          // (offset: 0x2380) Set port 0 direction register
            __O  uint32_t DIRSET1;          // (offset: 0x2384) Set port 1 direction register
        };
    };
    uint32_t RESERVED9[30];                 // (offset: 0x2388) RESERVED
    union
    {
        __O  uint32_t DIRCLR[2];
        struct
        {
            __O  uint32_t DIRCLR0;          // (offset: 0x2400) Clear port 0 direction register
            __O  uint32_t DIRCLR1;          // (offset: 0x2404) Clear port 1 direction register
        };
    };
    uint32_t RESERVED10[30];                // (offset: 0x2408) RESERVED
    union
    {
        __O  uint32_t DIRNOT[2];
        struct
        {
            __O  uint32_t DIRNOT0;          // (offset: 0x2480) Toggle port 0 direction register
            __O  uint32_t DIRNOT1;          // (offset: 0x2484) Toggle port 1 direction register
        };
    };
} LPC_GPIO_T;




// ------------ Pin Interrupts and Pattern Match (PIN_INT) --------------------
typedef struct
{
    __IO uint32_t ISEL;                     // (offset: 0x00) Pin Interrupt Mode register
    __IO uint32_t IENR;                     // (offset: 0x04) Pin Interrupt Enable (Rising) register
    __O  uint32_t SIENR;                    // (offset: 0x08) Set Pin Interrupt Enable (Rising) register
    __O  uint32_t CIENR;                    // (offset: 0x0C) Clear Pin Interrupt Enable (Rising) register
    __IO uint32_t IENF;                     // (offset: 0x10) Pin Interrupt Enable Falling Edge / Active Level register
    __O  uint32_t SIENF;                    // (offset: 0x14) Set Pin Interrupt Enable Falling Edge / Active Level register
    __O  uint32_t CIENF;                    // (offset: 0x18) Clear Pin Interrupt Enable Falling Edge / Active Level address
    __IO uint32_t RISE;                     // (offset: 0x1C) Pin Interrupt Rising Edge register
    __IO uint32_t FALL;                     // (offset: 0x20) Pin Interrupt Falling Edge register
    __IO uint32_t IST;                      // (offset: 0x24) Pin Interrupt Status register
    __IO uint32_t PMCTRL;                   // (offset: 0x28) GPIO pattern match interrupt control register
    __IO uint32_t PMSRC;                    // (offset: 0x2C) GPIO pattern match interrupt bit-slice source register
    __IO uint32_t PMCFG;                    // (offset: 0x30) GPIO pattern match interrupt bit slice configuration register
} LPC_PIN_INT_T;




// ------------ CRC Engine (CRC) ----------------------------------------------
typedef struct
{
    __IO uint32_t MODE;                     // (offset: 0x00) CRC mode register
    __IO uint32_t SEED;                     // (offset: 0x04) CRC seed register
    union
    {
        __I  uint32_t SUM;                  // (offset: 0x08) CRC checksum register
        __O  uint32_t WR_DATA;              // (offset: 0x08) CRC data register
    };
} LPC_CRC_T;




// ------------ Analog Comparator (ACMP) --------------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00) Comparator control register
    __IO uint32_t LAD;                      // (offset: 0x04) Voltage ladder register
} LPC_ACMP_T;




// ------------ Self Wakeup Timer (WKT) ---------------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00) Alarm / Wakeup Timer Control register
         uint32_t RESERVED[2];              // (offset: 0x04) RESERVED
    __IO uint32_t COUNT;                    // (offset: 0x0C) Alarm / Wakeup TImer counter register
} LPC_WKT_T;




// ------------ Multi-Rate Timer(MRT) -----------------------------------------
typedef struct
{
    __IO uint32_t INTVAL;                   // MRT Time interval value register
    __I  uint32_t TIMER;                    // MRT Timer register
    __IO uint32_t CTRL;                     // MRT Control register
    __IO uint32_t STAT;                     // MRT Status register
} LPC_MRT_CHANNEL_T;

typedef struct
{
    LPC_MRT_CHANNEL_T   CHANNEL[4];         // Channels

         uint32_t       RESERVED[45];       // (offset: 0x40) RESERVED
    __I  uint32_t       IDLE_CH;            // (offset: 0xF4) Idle channel register
    __IO uint32_t       IRQ_FLAG;           // (offset: 0xF8) Global interrupt flag register
} LPC_MRT_T;




// ------ Universal Synchronous Asynchronous Receiver Transmitter (USART) -----
typedef struct
{
    __IO uint32_t CFG;                      // (offset: 0x00) USART Configuration register
    __IO uint32_t CTL;                      // (offset: 0x04) USART Control register
    __IO uint32_t STAT;                     // (offset: 0x08) USART Status register
    __IO uint32_t INTENSET;                 // (offset: 0x0C) Interrupt Enable read and Set register
    __O  uint32_t INTENCLR;                 // (offset: 0x10) Interrupt Enable Clear register
    __I  uint32_t RXDAT;                    // (offset: 0x14) Receiver Data register
    __I  uint32_t RXDATSTAT;                // (offset: 0x18) Receiver Data with Status register
    __IO uint32_t TXDAT;                    // (offset: 0x1C) Transmit Data register
    __IO uint32_t BRG;                      // (offset: 0x20) Baud Rate Generator register
    __I  uint32_t INTSTAT;                  // (offset: 0x24) Interrupt status register
    __IO uint32_t OSR;                      // (offset: 0x28) Oversample selection register for asynchronous communication
    __IO uint32_t ADDR;                     // (offset: 0x2C) Address register for automatic address matching
} LPC_USART_T;




// ------------ Serial Peripheral Interface (SPI) -----------------------------
typedef struct
{
    __IO uint32_t CFG;                      // (offset: 0x00) SPI Configuration register
    __IO uint32_t DLY;                      // (offset: 0x04) SPI Delay register
    __IO uint32_t STAT;                     // (offset: 0x08) SPI Status register
    __IO uint32_t INTENSET;                 // (offset: 0x0C) SPI Interrupt Enable read and Set register
    __O  uint32_t INTENCLR;                 // (offset: 0x10) SPI Interrupt Enable Clear register
    __I  uint32_t RXDAT;                    // (offset: 0x14) SPI Receive Data register
    __IO uint32_t TXDATCTL;                 // (offset: 0x18) SPI Transmit Data with Control register
    __IO uint32_t TXDAT;                    // (offset: 0x1C) SPI Transmit Data register
    __IO uint32_t TXCTL;                    // (offset: 0x20) SPI Transmit Control register
    __IO uint32_t DIV;                      // (offset: 0x24) SPI Clock Divider register
    __I  uint32_t INTSTAT;                  // (offset: 0x28) SPI Interrupt Status register
} LPC_SPI_T;




// ------------ Inter-Integrated Circuit (I2C) --------------------------------
typedef struct
{
    __IO uint32_t CFG;                      // (offset: 0x00) Configuration for shared functions
    __IO uint32_t STAT;                     // (offset: 0x04) Status register for Master, Slave, and Monitor functions
    __IO uint32_t INTENSET;                 // (offset: 0x08) Interrupt Enable Set and read register
    __O  uint32_t INTENCLR;                 // (offset: 0x0C) Interrupt Enable Clear register
    __IO uint32_t TIMEOUT;                  // (offset: 0x10) Time-out value register
    __IO uint32_t CLKDIV;                   // (offset: 0x14) Clock pre-divider for the entire I2C interface
    __I  uint32_t INTSTAT;                  // (offset: 0x18) Interrupt Status register for Master, Slave, and Monitor functions
         uint32_t RESERVED0;                // (offset: 0x1C) RESERVED
    __IO uint32_t MSTCTL;                   // (offset: 0x20) Master control register
    __IO uint32_t MSTTIME;                  // (offset: 0x24) Master timing configuration
    __IO uint32_t MSTDAT;                   // (offset: 0x28) Combined Master receiver and transmitter data register
         uint32_t RESERVED1[5];             // (offset: 0x2C) RESERVED
    __IO uint32_t SLVCTL;                   // (offset: 0x40) Slave control register
    __IO uint32_t SLVDAT;                   // (offset: 0x44) Combined Slave receiver and transmitter data register
    union
    {
        __IO uint32_t SLVADR[4];
        struct
        {
            __IO uint32_t SLVADR0;          // (offset: 0x48) Slave address 0 register
            __IO uint32_t SLVADR1;          // (offset: 0x4C) Slave address 1 register
            __IO uint32_t SLVADR2;          // (offset: 0x50) Slave address 2 register
            __IO uint32_t SLVADR3;          // (offset: 0x54) Slave address 3 register
        };
    };
    __IO uint32_t SLVQUAL0;                 // (offset: 0x58) Slave Qualification for address 0
         uint32_t RESERVED2[9];             // (offset: 0x5C) RESERVED
    __I  uint32_t MONRXDAT;                 // (offset: 0x80) Monitor receiver data register
} LPC_I2C_T;




// ------------ State Configurable Timer (SCT) --------------------------------
#define SCT_NUM_EVENTS      (8)             // Number of events
#define SCT_NUM_REGISTERS   (8)             // Number of match/compare registers
#define SCT_NUM_OUTPUTS     (7)             // Number of outputs

typedef struct
{
    __IO uint32_t CONFIG;                   // (offset: 0x000) SCT configuration register
    union
    {
        __IO uint32_t CTRL;                 // (offset: 0x004) SCT control register
        struct
        {
            __IO uint16_t CTRL_L;
            __IO uint16_t CTRL_H;
        };
    };
    union
    {
        __IO uint32_t LIMIT;                // (offset: 0x008) SCT limit event select register
        struct
        {
            __IO uint16_t LIMIT_L;
            __IO uint16_t LIMIT_H;
        };
    };
    union
    {
        __IO uint32_t HALT;                 // (offset: 0x00C) SCT halt events elect register
        struct
        {
            __IO uint16_t HALT_L;
            __IO uint16_t HALT_H;
        };
    };
    union
    {
        __IO uint32_t STOP;                 // (offset: 0x010) SCT stop event select register
        struct
        {
            __IO uint16_t STOP_L;
            __IO uint16_t STOP_H;
        };
    };
    union
    {
        __IO uint32_t START;                // (offset: 0x014) SCT start event select register
        struct
        {
            __IO uint16_t START_L;
            __IO uint16_t START_H;
        };
    };
    uint32_t RESERVED0[10];                 // (offset: 0x018) RESERVED
    union
    {
        __IO uint32_t COUNT;                // (offset: 0x040) SCT counter register
        struct
        {
            __IO uint16_t COUNT_L;
            __IO uint16_t COUNT_H;
        };
    };
    union
    {
        __IO uint32_t STATE;                // (offset: 0x044) SCT state register
        struct
        {
            __IO uint16_t STATE_L;
            __IO uint16_t STATE_H;
        };
    };
    __I  uint32_t INPUT;                    // (offset: 0x048) SCT input register
    union
    {
        __IO uint32_t REGMODE;              // (offset: 0x04C) SCT match/capture mode register
        struct
        {
            __IO uint16_t REGMODE_L;
            __IO uint16_t REGMODE_H;
        };
    };
    __IO uint32_t OUTPUT;                   // (offset: 0x050) SCT output register
    __IO uint32_t OUTPUTDIRCTRL;            // (offset: 0x054) SCT output counter direction control register
    __IO uint32_t RES;                      // (offset: 0x058) SCT conflict resolution register
    __IO uint32_t DMAREQ0;                  // (offset: 0x05C) SCT DMA request 0 register
    __IO uint32_t DMAREQ1;                  // (offset: 0x060) SCT DMA request 1 register
         uint32_t RESERVED1[35];            // (offset: 0x064) RESERVED
    __IO uint32_t EVEN;                     // (offset: 0x0F0) SCT event interrupt enable register
    __IO uint32_t EVFLAG;                   // (offset: 0x0F4) SCT event flag register
    __IO uint32_t CONEN;                    // (offset: 0x0F8) SCT conflict interrupt enable register
    __IO uint32_t CONFLAG;                  // (offset: 0x0FC) SCT conflict flag register
    union                                   // (offset: 0x100) Match / Capture channel
    {
        __IO union
        {
            uint32_t U;                     // MATCH[n].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 // MATCH[n].L  Access to L value
                uint16_t H;                 // MATCH[n].H  Access to H value
            };
        } MATCH[SCT_NUM_REGISTERS];

        __I union
        {
            uint32_t U;                     // CAP[n].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 // CAP[n].L  Access to L value
                uint16_t H;                 // CAP[n].H  Access to H value
            };
        } CAP[SCT_NUM_REGISTERS];
    };
    uint32_t RESERVED2[0x100 - (4 * SCT_NUM_REGISTERS)];    // (offset: 0x120) RESERVED
    union                                                   // (offset: 0x200) Match Reload / Capture Control value
    {
        union
        {
            __IO uint32_t U;                // MATCHREL[n].U  Unified 32-bit register
            struct
            {
                __IO uint16_t L;            // MATCHREL[n].L  Access to L value
                __IO uint16_t H;            // MATCHREL[n].H  Access to H value
            };
        } MATCHREL[SCT_NUM_REGISTERS];
        union
        {
            __IO uint32_t U;                // CAPCTRL[n].U  Unified 32-bit register
            struct
            {
                __IO uint16_t L;            // CAPCTRL[n].L     Access to L value
                __IO uint16_t H;            // SCTCAPCTRL[n].H  Access to H value
            };
        } CAPCTRL[SCT_NUM_REGISTERS];
    };
    uint32_t RESERVED5[0x100 - (4 * SCT_NUM_REGISTERS)];    // (offset: 0x220) RESERVED
    struct                                                  // (offset: 0x300) State / Control
    {
        __IO uint32_t STATE;                // EVENT[n].STATE  SCT Event n State register
        __IO uint32_t CTRL;                 // EVENT[n].CTRL   SCT Event n Control register
    } EVENT[SCT_NUM_EVENTS];
    uint32_t RESERVED8[0x200 - (8 * SCT_NUM_EVENTS)];       // (offset: 0x490) RESERVED
    struct                                                  // (offset: 0x500) Output Set / Clear
    {
        __IO uint32_t SET;                  // OUT[n].SET  Output n Set Register
        __IO uint32_t CLR;                  // OUT[n].CLR  Output n Clear Register
    } OUT[SCT_NUM_OUTPUTS];
} LPC_SCT_T;




// ------------ Standard Counter/Timer (CTIMER) -------------------------------
typedef struct
{
    __IO uint32_t IR;                       // (offset: 0x00) Interrupt Register
    __IO uint32_t TCR;                      // (offset: 0x04) Timer Control Register
    __IO uint32_t TC;                       // (offset: 0x08) Timer Counter
    __IO uint32_t PR;                       // (offset: 0x0C) Prescale Register
    __IO uint32_t PC;                       // (offset: 0x10) Prescale Counter
    __IO uint32_t MCR;                      // (offset: 0x14) Match Control Register
    __IO uint32_t MR[4];                    // (offset: 0x18) Match Register
    __IO uint32_t CCR;                      // (offset: 0x28) Capture Control Register
    __I  uint32_t CR[4];                    // (offset: 0x2C) Capture Register
    __IO uint32_t EMR;                      // (offset: 0x3C) External Match Register
         uint32_t RESERVED[12];             // (offset: 0x40) RESERVED
    __IO uint32_t CTCR;                     // (offset: 0x70) Count Control Register
    __IO uint32_t PWMC;                     // (offset: 0x74) PWM Control Register
    __IO uint32_t MSR[4];                   // (offset: 0x78) Match Shadow Register
} LPC_CTIMER_T;




// ------------ Windowed Watchdog Timer (WWDT) --------------------------------
typedef struct
{
    __IO uint32_t MOD;                      // (offset: 0x00) Watchdog mode register
    __IO uint32_t TC;                       // (offset: 0x04) Watchdog timer constant register
    __O  uint32_t FEED;                     // (offset: 0x08) Watchdog feed sequence register
    __I  uint32_t TV;                       // (offset: 0x0C) Watchdog timer value register
         uint32_t RESERVED;                 // (offset: 0x10) RESERVED
    __IO uint32_t WARNINT;                  // (offset: 0x14) Watchdog timer warning int. register
    __IO uint32_t WINDOW;                   // (offset: 0x18) Watchdog timer window value register
} LPC_WWDT_T;




// ----------------------------------------------------------------------------
// Input multiplexing and DMA trigger multiplexing (INPUT MUX, DMA TRIGMUX)
// ----------------------------------------------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t DMA_INMUX_INMUX[2];
        struct
        {
            __IO uint32_t DMA_INMUX_INMUX0; // (offset: 0x00) DMA input trigger input mux input register 0
            __IO uint32_t DMA_INMUX_INMUX1; // (offset: 0x04) DMA input trigger input mux input register 1
        };
    };
    uint32_t RESERVED0[6];                  // (offset: 0x08) RESERVED
    union
    {
        __IO uint32_t SCT0_INMUX[4];
        struct
        {
            __IO uint32_t SCT0_INMUX0;      // (offset: 0x20) Input mux register for SCT input 0
            __IO uint32_t SCT0_INMUX1;      // (offset: 0x24) Input mux register for SCT input 1
            __IO uint32_t SCT0_INMUX2;      // (offset: 0x28) Input mux register for SCT input 2
            __IO uint32_t SCT0_INMUX3;      // (offset: 0x2C) Input mux register for SCT input 3
        };
    };
    uint32_t RESERVED1[4];                  // (offset: 0x30) RESERVED
    union
    {
        __IO uint32_t DMA_ITRIG_INMUX[25];
        struct
        {
            __IO uint32_t DMA_ITRIG_INMUX0;     // (offset: 0x40) DMA input trigger Input mux register 0
            __IO uint32_t DMA_ITRIG_INMUX1;     // (offset: 0x44) DMA input trigger Input mux register 1
            __IO uint32_t DMA_ITRIG_INMUX2;     // (offset: 0x48) DMA input trigger Input mux register 2
            __IO uint32_t DMA_ITRIG_INMUX3;     // (offset: 0x4C) DMA input trigger Input mux register 3
            __IO uint32_t DMA_ITRIG_INMUX4;     // (offset: 0x50) DMA input trigger Input mux register 4
            __IO uint32_t DMA_ITRIG_INMUX5;     // (offset: 0x54) DMA input trigger Input mux register 5
            __IO uint32_t DMA_ITRIG_INMUX6;     // (offset: 0x58) DMA input trigger Input mux register 6
            __IO uint32_t DMA_ITRIG_INMUX7;     // (offset: 0x5C) DMA input trigger Input mux register 7
            __IO uint32_t DMA_ITRIG_INMUX8;     // (offset: 0x60) DMA input trigger Input mux register 8
            __IO uint32_t DMA_ITRIG_INMUX9;     // (offset: 0x64) DMA input trigger Input mux register 9
            __IO uint32_t DMA_ITRIG_INMUX10;    // (offset: 0x68) DMA input trigger Input mux register 10
            __IO uint32_t DMA_ITRIG_INMUX11;    // (offset: 0x6C) DMA input trigger Input mux register 11
            __IO uint32_t DMA_ITRIG_INMUX12;    // (offset: 0x70) DMA input trigger Input mux register 12
            __IO uint32_t DMA_ITRIG_INMUX13;    // (offset: 0x74) DMA input trigger Input mux register 13
            __IO uint32_t DMA_ITRIG_INMUX14;    // (offset: 0x78) DMA input trigger Input mux register 14
            __IO uint32_t DMA_ITRIG_INMUX15;    // (offset: 0x7C) DMA input trigger Input mux register 15
            __IO uint32_t DMA_ITRIG_INMUX16;    // (offset: 0x80) DMA input trigger Input mux register 16
            __IO uint32_t DMA_ITRIG_INMUX17;    // (offset: 0x84) DMA input trigger Input mux register 17
            __IO uint32_t DMA_ITRIG_INMUX18;    // (offset: 0x88) DMA input trigger Input mux register 18
            __IO uint32_t DMA_ITRIG_INMUX19;    // (offset: 0x8C) DMA input trigger Input mux register 19
            __IO uint32_t DMA_ITRIG_INMUX20;    // (offset: 0x90) DMA input trigger Input mux register 20
            __IO uint32_t DMA_ITRIG_INMUX21;    // (offset: 0x94) DMA input trigger Input mux register 21
            __IO uint32_t DMA_ITRIG_INMUX22;    // (offset: 0x98) DMA input trigger Input mux register 22
            __IO uint32_t DMA_ITRIG_INMUX23;    // (offset: 0x9C) DMA input trigger Input mux register 23
            __IO uint32_t DMA_ITRIG_INMUX24;    // (offset: 0xA0) DMA input trigger Input mux register 24
        };
    };
} LPC_INMUX_TRIGMUX_T;




// ------------ Analog-to-Digital Converter (ADC) -----------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00) ADC Control register
         uint32_t RESERVED0;                // (offset: 0x04) RESERVED
    __IO uint32_t SEQA_CTRL;                // (offset: 0x08) ADC Conversion Sequence-A control register
    __IO uint32_t SEQB_CTRL;                // (offset: 0x0C) ADC Conversion Sequence-B control register
    __I  uint32_t SEQA_GDAT;                // (offset: 0x10) ADC Sequence-A Global Data register
    __I  uint32_t SEQB_GDAT;                // (offset: 0x14) ADC Sequence-B Global Data register
         uint32_t RESERVED1[2];             // (offset: 0x18) RESERVED
    __I  uint32_t DAT[12];                  // (offset: 0x20) ADC Channel N Data register
    __IO uint32_t THR0_LOW;                 // (offset: 0x50) ADC Low Compare Threshold register 0
    __IO uint32_t THR1_LOW;                 // (offset: 0x54) ADC Low Compare Threshold register 1
    __IO uint32_t THR0_HIGH;                // (offset: 0x58) ADC High Compare Threshold register 0
    __IO uint32_t THR1_HIGH;                // (offset: 0x5C) ADC High Compare Threshold register 1
    __IO uint32_t CHAN_THRSEL;              // (offset: 0x60) ADC Channel-Threshold Select register
    __IO uint32_t INTEN;                    // (offset: 0x64) ADC Interrupt Enable register
    __IO uint32_t FLAGS;                    // (offset: 0x68) ADC Flags register
    __IO uint32_t TRM;                      // (offset: 0x6C) ADC Trim register
} LPC_ADC_T;




// ------------ Direct memory access (DMA) ------------------------------------
#define DMA_NUM_CHANNELS        25

typedef struct
{
    __IO uint32_t CFG;                      // Configuration register for DMA channel N
    __I  uint32_t CTLSTAT;                  // Control and status register for DMA channel N
    __IO uint32_t XFERCFG;                  // Transfer configuration register for DMA channel N
         uint32_t RESERVED;                 // RESERVED
} LPC_DMA_CHANNEL_T;

typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x000) DMA control
    __I  uint32_t INTSTAT;                  // (offset: 0x004) Interrupt status
    __IO uint32_t SRAMBASE;                 // (offset: 0x008) SRAM address of the channel configuration table
         uint32_t RESERVED0[5];             // (offset: 0x00C) RESERVED
    __IO uint32_t ENABLESET;                // (offset: 0x020) Channel Enable read and Set for all DMA channels
         uint32_t RESERVED1;                // (offset: 0x024) RESERVED
    __O  uint32_t ENABLECLR;                // (offset: 0x028) Channel Enable Clear for all DMA channels
         uint32_t RESERVED2;                // (offset: 0x02C) RESERVED
    __I  uint32_t ACTIVE;                   // (offset: 0x030) Channel Active status for all DMA channels
         uint32_t RESERVED3;                // (offset: 0x034) RESERVED
    __I  uint32_t BUSY;                     // (offset: 0x038) Channel Busy status for all DMA channels
         uint32_t RESERVED4;                // (offset: 0x03C) RESERVED
    __IO uint32_t ERRINT;                   // (offset: 0x040) Error Interrupt status for all DMA channels
         uint32_t RESERVED5;                // (offset: 0x044) RESERVED
    __IO uint32_t INTENSET;                 // (offset: 0x048) Interrupt Enable read and Set for all DMA channels
         uint32_t RESERVED6;                // (offset: 0x04C) RESERVED
    __O  uint32_t INTENCLR;                 // (offset: 0x050) Interrupt Enable Clear for all DMA channels
         uint32_t RESERVED7;                // (offset: 0x054) RESERVED
    __IO uint32_t INTA;                     // (offset: 0x058) Interrupt A status for all DMA channels
         uint32_t RESERVED8;                // (offset: 0x05C) RESERVED
    __IO uint32_t INTB;                     // (offset: 0x060) Interrupt B status for all DMA channels
         uint32_t RESERVED9;                // (offset: 0x064) RESERVED
    __O  uint32_t SETVALID;                 // (offset: 0x068) Set ValidPending control bits for all DMA channels
         uint32_t RESERVED10;               // (offset: 0x06C) RESERVED
    __O  uint32_t SETTRIG;                  // (offset: 0x070) Set Trigger control bits for all DMA channels
         uint32_t RESERVED11;               // (offset: 0x074) RESERVED
    __O  uint32_t ABORT;                    // (offset: 0x078) Channel Abort control for all DMA channels
         uint32_t RESERVED12[225];          // (offset: 0x07C) RESERVED

    LPC_DMA_CHANNEL_T CHANNEL[DMA_NUM_CHANNELS];    // (offset: 0x400) Channels
} LPC_DMA_T;




// ------------ Digital-to-Analog Converter (DAC) -----------------------------
typedef struct
{
    __IO uint32_t CR;                       // (offset: 0x00) D/A Converter Register
    __IO uint32_t CTRL;                     // (offset: 0x04) DAC Control register
    __IO uint32_t CNTVAL;                   // (offset: 0x08) DAC Counter Value register
} LPC_DAC_T;




// ------------ Capacitive Touch (CAPT) ---------------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x000) Control register
    __IO uint32_t STATUS;                   // (offset: 0x004) Status register
    __IO uint32_t POLL_TCNT;                // (offset: 0x008) Poll and measurement counter register
         uint32_t RESERVED0;                // (offset: 0x00C) RESERVED
    __IO uint32_t INTENSET;                 // (offset: 0x010) Interrupt Enable Read and Set register
    __O  uint32_t INTENCLR;                 // (offset: 0x014) Interrupt Enable Clear register
    __I  uint32_t INTSTAT;                  // (offset: 0x018) Interrupt Status register
         uint32_t RESERVED1;                // (offset: 0x01C) RESERVED
    __I  uint32_t TOUCH;                    // (offset: 0x020) Touch data register
         uint32_t RESERVED2[1014];          // (offset: 0x024) RESERVED
    __I  uint32_t ID;                       // (offset: 0xFFC) Block ID
} LPC_CAPT_T;




// ------------ Fast Initialization Memory (FAIM) -----------------------------
typedef struct
{
    union
    {
        __IO uint32_t WORD[8];
        struct
        {
            __IO uint32_t WORD0;            // (offset: 0x000) FAIM word 0
            __IO uint32_t WORD1;            // (offset: 0x004) FAIM word 1
            __IO uint32_t WORD2;            // (offset: 0x008) FAIM word 2 (reserved)
            __IO uint32_t WORD3;            // (offset: 0x00C) FAIM word 3 (reserved)
            __IO uint32_t WORD4;            // (offset: 0x010) FAIM word 4
            __IO uint32_t WORD5;            // (offset: 0x014) FAIM word 5
            __IO uint32_t WORD6;            // (offset: 0x018) FAIM word 6
            __IO uint32_t WORD7;            // (offset: 0x01C) FAIM word 7
        };
    };
} LPC_FAIM_T;




// ------------ ROM API -------------------------------------------------------
// Power API functions
typedef struct
{
    void (*set_pll) (uint32_t cmd[], uint32_t resp[]);
    void (*set_power)(uint32_t cmd[], uint32_t resp[]);
    void (*set_fro_frequency)(uint32_t frequency);
    void (*power_mode_configure)(uint32_t power_mode, uint32_t peripheral_ctrl);
    void (*set_aclkgate)(uint32_t aclkgate);
    uint32_t (*get_aclkgate)(void);
} LPC_ROM_PWR_API_T;

// Integer divide API functions
typedef struct
{
    int32_t quot;   // Quotient
    int32_t rem;    // Remainder
} LPC_IDIV_RETURN_T;

typedef struct
{
    uint32_t quot;  // Quotient
    uint32_t rem;   // Reminder
} LPC_UIDIV_RETURN_T;

typedef struct
{
     int32_t (* idiv)( int32_t numerator,  int32_t denominator);                // Signed integer division
    uint32_t (*uidiv)(uint32_t numerator, uint32_t denominator);                // Unsigned integer division

    LPC_IDIV_RETURN_T  (* idivmod)( int32_t numerator,  int32_t denominator);   // Signed integer division with remainder
    LPC_UIDIV_RETURN_T (*uidivmod)(uint32_t numerator, uint32_t denominator);   // Unsigned integer division with remainder
} LPC_ROM_DIV_API_T;

// The master structure that defines the table of all ROM APIs on the device - ROM Driver table
typedef struct
{
  const uint32_t            RESERVED0[3];   // (offset: 0x000) RESERVED
  const LPC_ROM_PWR_API_T*  PWR_BASE_PTR;   // (offset: 0x00C) Power APIs function table base address
  const LPC_ROM_DIV_API_T*  DIV_BASE_PTR;   // (offset: 0x010) Integer division routines function table base address
  const uint32_t            RESERVED1[7];   // (Offsets 0x014) RESERVED
} LPC_ROM_API_T;




// ROM IAP entry function pointer declaration
typedef void (*LPC_ROM_IAP_ENTRY_T)(uint32_t command[], uint32_t result[]);




// ISO C++ prohibits anonymous structs [-Wpedantic]
// #pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop




//-------------------------------------------------------------------------
// Peripheral memory map
//-------------------------------------------------------------------------
// Base addresses
#define LPC_FLASH_BASE          (0x00000000UL)
#define LPC_ROM_BASE            (0x0F000000UL)
#define LPC_RAM_BASE            (0x10000000UL)
#define LPC_APB_BASE            (0x40000000UL)
#define LPC_AHB_BASE            (0x50000000UL)
#define LPC_GPIO_BASE           (0xA0000000UL)

// ROM Driver table
#define LPC_ROM_DRIVER_BASE     (LPC_ROM_BASE + 0x1FF8)
#define LPC_ROM_IAP_BASE        (LPC_ROM_BASE + 0x1FF1)

// APB0 peripherals
#define LPC_WWDT_BASE           (LPC_APB_BASE + 0x00000)
#define LPC_MRT_BASE            (LPC_APB_BASE + 0x04000)
#define LPC_WKT_BASE            (LPC_APB_BASE + 0x08000)
#define LPC_SWM_BASE            (LPC_APB_BASE + 0x0C000)
#define LPC_DAC0_BASE           (LPC_APB_BASE + 0x14000)
#define LPC_DAC1_BASE           (LPC_APB_BASE + 0x18000)
#define LPC_ADC_BASE            (LPC_APB_BASE + 0x1C000)
#define LPC_PMU_BASE            (LPC_APB_BASE + 0x20000)
#define LPC_CMP_BASE            (LPC_APB_BASE + 0x24000)
// RESERVED                     (LPC_APB_BASE + 0x28000)
#define LPC_INMUX_TRIGMUX_BASE  (LPC_APB_BASE + 0x2C000)
#define LPC_I2C2_BASE           (LPC_APB_BASE + 0x30000)
#define LPC_I2C3_BASE           (LPC_APB_BASE + 0x34000)
#define LPC_CTIMER_BASE         (LPC_APB_BASE + 0x38000)
// RESERVED                     (LPC_APB_BASE + 0x38000)
#define LPC_FMC_BASE            (LPC_APB_BASE + 0x40000)
#define LPC_IOCON_BASE          (LPC_APB_BASE + 0x44000)
#define LPC_SYSCON_BASE         (LPC_APB_BASE + 0x48000)
#define LPC_I2C0_BASE           (LPC_APB_BASE + 0x50000)
#define LPC_I2C1_BASE           (LPC_APB_BASE + 0x54000)
#define LPC_SPI0_BASE           (LPC_APB_BASE + 0x58000)
#define LPC_SPI1_BASE           (LPC_APB_BASE + 0x5C000)
#define LPC_CAPT_BASE           (LPC_APB_BASE + 0x60000)
#define LPC_USART0_BASE         (LPC_APB_BASE + 0x64000)
#define LPC_USART1_BASE         (LPC_APB_BASE + 0x68000)
#define LPC_USART2_BASE         (LPC_APB_BASE + 0x6C000)
#define LPC_USART3_BASE         (LPC_APB_BASE + 0x70000)
#define LPC_USART4_BASE         (LPC_APB_BASE + 0x74000)

// AHB peripherals
#define LPC_CRC_BASE            (LPC_AHB_BASE + 0x00000)
#define LPC_SCT_BASE            (LPC_AHB_BASE + 0x04000)
#define LPC_DMA_BASE            (LPC_AHB_BASE + 0x08000)
#define LPC_MTB_SFR_BASE        (LPC_AHB_BASE + 0x0C000)
#define LPC_FAIM_BASE           (LPC_AHB_BASE + 0x10000)

// GPIO interrupts
#define LPC_PIN_INT_BASE        (LPC_GPIO_BASE + 0x4000)




//-------------------------------------------------------------------------
// Peripheral declarations
//-------------------------------------------------------------------------
// ROM API
#define LPC_ROM_API            (*(LPC_ROM_API_T    * *) LPC_ROM_DRIVER_BASE)
#define LPC_ROM_PWR_API         ((LPC_ROM_PWR_API_T  *)(LPC_ROM_API->PWR_BASE_PTR))
#define LPC_ROM_DIV_API         ((LPC_ROM_DIV_API_T  *)(LPC_ROM_API->DIV_BASE_PTR))

// IAP entry function pointer
static const LPC_ROM_IAP_ENTRY_T iap_entry = (LPC_ROM_IAP_ENTRY_T)(LPC_ROM_IAP_BASE);

// APB0 peripherals
#define LPC_WWDT                ((LPC_WWDT_T         *) LPC_WWDT_BASE)
#define LPC_MRT                 ((LPC_MRT_T          *) LPC_MRT_BASE)
#define LPC_WKT                 ((LPC_WKT_T          *) LPC_WKT_BASE)
#define LPC_SWM                 ((LPC_SWM_T          *) LPC_SWM_BASE)
#define LPC_DAC0                ((LPC_DAC_T          *) LPC_DAC0_BASE)
#define LPC_DAC1                ((LPC_DAC_T          *) LPC_DAC1_BASE)
#define LPC_ADC                 ((LPC_ADC_T          *) LPC_ADC_BASE)
#define LPC_PMU                 ((LPC_PMU_T          *) LPC_PMU_BASE)
#define LPC_CMP                 ((LPC_CMP_T          *) LPC_CMP_BASE)
#define LPC_INMUX_TRIGMUX       ((LPC_INMUX_TRIGMUX_T*) LPC_INMUX_TRIGMUX_BASE)
#define LPC_I2C2                ((LPC_I2C_T          *) LPC_I2C2_BASE)
#define LPC_I2C3                ((LPC_I2C_T          *) LPC_I2C3_BASE)
#define LPC_CTIMER              ((LPC_CTIMER_T       *) LPC_CTIMER_BASE)
#define LPC_FMC                 ((LPC_FMC_T          *) LPC_FMC_BASE)
#define LPC_IOCON               ((LPC_IOCON_T        *) LPC_IOCON_BASE)
#define LPC_SYSCON              ((LPC_SYSCON_T       *) LPC_SYSCON_BASE)
#define LPC_I2C0                ((LPC_I2C_T          *) LPC_I2C0_BASE)
#define LPC_I2C1                ((LPC_I2C_T          *) LPC_I2C1_BASE)
#define LPC_SPI0                ((LPC_SPI_T          *) LPC_SPI0_BASE)
#define LPC_SPI1                ((LPC_SPI_T          *) LPC_SPI1_BASE)
#define LPC_CAPT                ((LPC_CAPT_T         *) LPC_CAPT_BASE)
#define LPC_USART0              ((LPC_USART_T        *) LPC_USART0_BASE)
#define LPC_USART1              ((LPC_USART_T        *) LPC_USART1_BASE)
#define LPC_USART2              ((LPC_USART_T        *) LPC_USART2_BASE)
#define LPC_USART3              ((LPC_USART_T        *) LPC_USART3_BASE)
#define LPC_USART4              ((LPC_USART_T        *) LPC_USART4_BASE)

// AHB peripherals
#define LPC_CRC                 ((LPC_CRC_T          *) LPC_CRC_BASE)
#define LPC_SCT                 ((LPC_SCT_T          *) LPC_SCT_BASE)
#define LPC_DMA                 ((LPC_DMA_T          *) LPC_DMA_BASE)
#define LPC_FAIM                ((LPC_FAIM_T         *) LPC_FAIM_BASE)

// GPIO peripheral and interrupts
#define LPC_GPIO                ((LPC_GPIO_T         *) LPC_GPIO_BASE)
#define LPC_PIN_INT             ((LPC_PIN_INT_T      *) LPC_PIN_INT_BASE)




//-------------------------------------------------------------------------
// Other chip-specific macro definitions
//-------------------------------------------------------------------------

// CAP Touch pins to IOCON mapping
#define CAPTOUCH_X0_PORT        PIO0_31
#define CAPTOUCH_X1_PORT        PIO1_0
#define CAPTOUCH_X2_PORT        PIO1_1
#define CAPTOUCH_X3_PORT        PIO1_2
#define CAPTOUCH_X4_PORT        PIO1_3
#define CAPTOUCH_X5_PORT        PIO1_4
#define CAPTOUCH_X6_PORT        PIO1_5
#define CAPTOUCH_X7_PORT        PIO1_6
#define CAPTOUCH_X8_PORT        PIO1_7
#define CAPTOUCH_YL_PORT        PIO1_8
#define CAPTOUCH_YH_PORT        PIO1_9

// ACMP_I-to-IOCON mapping
#define ACMP_I1_PORT            PIO0_0
#define ACMP_I2_PORT            PIO0_1
#define ACMP_I3_PORT            PIO0_14
#define ACMP_I4_PORT            PIO0_23
#define ACMP_I5_PORT            PIO0_30




#ifdef __cplusplus
} // extern "C"
#endif

#endif // __XARMLIB_TARGETS_LPC84X_CMSIS_HPP
