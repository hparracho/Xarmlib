// ----------------------------------------------------------------------------
// @file    lpc84x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for NXP LPC84x MCUs.
// @date    28 June 2018
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
    DebugMonitor_IRQn           = -4,       // 12 Debug Monitor interrupt
    PendSV_IRQn                 = -2,       // 14 Cortex-M0 Pend SV interrupt
    SysTick_IRQn                = -1,       // 15 Cortex-M0 System Tick interrupt

    // LPC84x Specific Interrupt Numbers
    SPI0_IRQn                   = 0,        // SPI0 interrupt
    SPI1_IRQn                   = 1,        // SPI1 interrupt
    DAC0_IRQn                   = 2,        // DAC0 Interrupt
    USART0_IRQn                 = 3,        // USART0 interrupt
    USART1_IRQn                 = 4,        // USART1 interrupt
    USART2_IRQn                 = 5,        // USART2 interrupt
    FAIM_IRQn                   = 6,        // FAIM interrupt
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
#define __CM0PLUS_REV           0x0001
#define __MPU_PRESENT           0           // MPU present or not
#define __NVIC_PRIO_BITS        2           // Number of Bits used for Priority Levels
#define __Vendor_SysTickConfig  0           // Set to 1 if different SysTick Config is used




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
    __IO uint32_t SYSMEMREMAP;              // (offset: 0x000)       System memory remap
         uint32_t RESERVED0;                // (offset: 0x004)       RESERVED
    __IO uint32_t SYSPLLCTRL;               // (offset: 0x008)       System PLL control
    __I  uint32_t SYSPLLSTAT;               // (offset: 0x00C)       System PLL status
         uint32_t RESERVED1[4];             // (offset: 0x010-0x01C) RESERVED
    __IO uint32_t SYSOSCCTRL;               // (offset: 0x020)       System oscillator control
    __IO uint32_t WDTOSCCTRL;               // (offset: 0x024)       Watchdog oscillator control
    __IO uint32_t FROOSCCTRL;               // (offset: 0x028)       FRO oscillator control
         uint32_t RESERVED2;                // (offset: 0x02C)       RESERVED
    __IO uint32_t FRODIRECTCLKUEN;          // (offset: 0x030)       FRO direct clock source update
         uint32_t RESERVED3;                // (offset: 0x034)       RESERVED
    __IO uint32_t SYSRSTSTAT;               // (offset: 0x038)       System reset status 0
    __IO uint32_t FAIMROWPROTECTCTRL;       // (offset: 0x03C)       FAIM row protect control
    __IO uint32_t SYSPLLCLKSEL;             // (offset: 0x040)       System PLL clock source select 0
    __IO uint32_t SYSPLLCLKUEN;             // (offset: 0x044)       System PLL clock source update
    __IO uint32_t MAINCLKPLLSEL;            // (offset: 0x048)       Main clock pll source select 0
    __IO uint32_t MAINCLKPLLUEN;            // (offset: 0x04C)       Main clock pll source update enable
    __IO uint32_t MAINCLKSEL;               // (offset: 0x050)       Main clock source select
    __IO uint32_t MAINCLKUEN;               // (offset: 0x054)       Main clock source update enable
    __IO uint32_t SYSAHBCLKDIV;             // (offset: 0x058)       System clock divider
         uint32_t RESERVED4;                // (offset: 0x05C)       RESERVED
    __IO uint32_t CAPTCLKSEL;               // (offset: 0x060)       CAPT clock source select
    __IO uint32_t ADCCLKSEL;                // (offset: 0x064)       ADC clock source select
    __IO uint32_t ADCCLKDIV;                // (offset: 0x068)       ADC clock divider
    __IO uint32_t SCTCLKSEL;                // (offset: 0x06C)       SCT clock source select
    __IO uint32_t SCTCLKDIV;                // (offset: 0x070)       SCT clock divider
    __IO uint32_t EXTCLKSEL;                // (offset: 0x074)       External clock source select
         uint32_t RESERVED5[2];             // (offset: 0x078-0x07C) RESERVED
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
         uint32_t RESERVED6;                // (offset: 0x0BC)       RESERVED
    __IO uint32_t EFLASHREFCLKDIV;          // (offset: 0x0C0)       EFLASH REF clock divider
    __IO uint32_t FAIMREFCLKDIV;            // (offset: 0x0C4)       FAIM REF clock divider
         uint32_t RESERVED7[2];             // (offset: 0x0C8-0x0CC) RESERVED
    __IO uint32_t FRG0DIV;                  // (offset: 0x0D0)       FRG0 divider value
    __IO uint32_t FRG0MULT;                 // (offset: 0x0D4)       FRG0 multiplier value
    __IO uint32_t FRG0CLKSEL;               // (offset: 0x0D8)       FRG0 clock source select
         uint32_t RESERVED8;                // (offset: 0x0DC)       RESERVED
    __IO uint32_t FRG1DIV;                  // (offset: 0x0E0)       FRG1 divider value
    __IO uint32_t FRG1MULT;                 // (offset: 0x0E4)       FRG1 multiplier value
    __IO uint32_t FRG1CLKSEL;               // (offset: 0x0E8)       FRG1 clock source select
         uint32_t RESERVED9;                // (offset: 0x0EC)       RESERVED
    __IO uint32_t CLKOUTSEL;                // (offset: 0x0F0)       CLKOUT clock source select
    __IO uint32_t CLKOUTDIV;                // (offset: 0x0F4)       CLKOUT clock divider
         uint32_t RESERVED10;               // (offset: 0x0F8)       RESERVED
    __IO uint32_t EXTTRACECMD;              // (offset: 0x0FC)       External trace buffer command
    __I  uint32_t PIOPORCAP0;               // (offset: 0x100)       POR captured PIO0 status 0
    __I  uint32_t PIOPORCAP1;               // (offset: 0x104)       POR captured PIO1 status 0
         uint32_t RESERVED11[11];           // (offset: 0x108-0x130) RESERVED
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
    __IO uint32_t BODCTRL;                  // (offset: 0x150)       Brown-Out Detect
    __IO uint32_t SYSTCKCAL;                // (offset: 0x154)       System tick counter calibration
         uint32_t RESERVED12[6];            // (offset: 0x158-0x16C) RESERVED
    __IO uint32_t IRQLATENCY;               // (offset: 0x170)       IRQ delay. Allows trade-off between interrupt latency and determinism.
    __IO uint32_t NMISRC;                   // (offset: 0x174)       NMI Source Control
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
         uint32_t RESERVED13[27];           // (offset: 0x198-0x200) RESERVED
    __IO uint32_t STARTERP0;                // (offset: 0x204)       Start logic 0 pin wake-up enable
         uint32_t RESERVED14[3];            // (offset: 0x208-0x210) RESERVED
    __IO uint32_t STARTERP1;                // (offset: 0x214)       Start logic 1 interrupt wake-up enable
         uint32_t RESERVED15[6];            // (offset: 0x218-0x22C) RESERVED
    __IO uint32_t PDSLEEPCFG;               // (offset: 0x230)       Power-down states in deep-sleep mode
    __IO uint32_t PDAWAKECFG;               // (offset: 0x234)       Power-down states for wake-up from deep-sleep
    __IO uint32_t PDRUNCFG;                 // (offset: 0x238)       Power configuration
         uint32_t RESERVED16[111];          // (offset: 0x23C-0x3F4) RESERVED
    __I  uint32_t DEVICE_ID;                // (offset: 0x3F8)       Device ID
} LPC_SYSCON_T;




// ------------ I/O Configuration (IOCON) -------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PIO[56];
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
         uint32_t RESERVED0[4];             // (offset: 0x00-0x0C) RESERVED
    __IO uint32_t FLASHCFG;                 // (offset: 0x10)      Flash configuration register
         uint32_t RESERVED1[3];             // (offset: 0x14-0x1C) RESERVED
    __IO uint32_t FMSSTART;                 // (offset: 0x20)      Signature start address register
    __IO uint32_t FMSSTOP;                  // (offset: 0x24)      Signature stop-address register
         uint32_t RESERVED2;                // (offset: 0x28)      RESERVED
    __I  uint32_t FMSW0;                    // (offset: 0x2C)      Signature word
} LPC_FMC_T;




// ------------ Power Management Unit (PMU) -----------------------------------
typedef struct
{
    __IO uint32_t PCON;                     // (offset: 0x000) Power control Register
    __IO uint32_t GPREG0;                   // (offset: 0x004) General purpose Register 0
    __IO uint32_t GPREG1;                   // (offset: 0x008) General purpose Register 1
    __IO uint32_t GPREG2;                   // (offset: 0x00C) General purpose Register 2
    __IO uint32_t GPREG3;                   // (offset: 0x010) General purpose Register 3
    __IO uint32_t DPDCTRL;                  // (offset: 0x014) Deep power-down control register
} LPC_PMU_T;




// ------------ Switch Matrix (SWM) -------------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PINASSIGN[15];
        struct
        {
            __IO uint32_t PINASSIGN0;       // (offset: 0x000)
            __IO uint32_t PINASSIGN1;       // (offset: 0x004)
            __IO uint32_t PINASSIGN2;       // (offset: 0x008)
            __IO uint32_t PINASSIGN3;       // (offset: 0x00C)
            __IO uint32_t PINASSIGN4;       // (offset: 0x010)
            __IO uint32_t PINASSIGN5;       // (offset: 0x014)
            __IO uint32_t PINASSIGN6;       // (offset: 0x018)
            __IO uint32_t PINASSIGN7;       // (offset: 0x01C)
            __IO uint32_t PINASSIGN8;       // (offset: 0x020)
            __IO uint32_t PINASSIGN9;       // (offset: 0x024)
            __IO uint32_t PINASSIGN10;      // (offset: 0x028)
            __IO uint32_t PINASSIGN11;      // (offset: 0x02C)
            __IO uint32_t PINASSIGN12;      // (offset: 0x030)
            __IO uint32_t PINASSIGN13;      // (offset: 0x034)
            __IO uint32_t PINASSIGN14;      // (offset: 0x038)
        };
    };
         uint32_t RESERVED[97];             // (offset: 0x03C-0x1BC) RESERVED
    __IO uint32_t PINENABLE0;               // (offset: 0x1C0)
    __IO uint32_t PINENABLE1;               // (offset: 0x1C4)
} LPC_SWM_T;




// ------------ General Purpose I/O (GPIO) ------------------------------------
typedef struct
{
    __IO uint8_t  B0[32];                   // (offset: 0x0000-0x001F) Byte pin registers P0.0 - P0.31
    __IO uint8_t  B1[32];                   // (offset: 0x0020-0x003F) Byte pin registers P1.0 - P1.31
         uint8_t  RESERVED0[4032];          // (offset: 0x0040-0x0FFF) RESERVED
    __IO uint32_t W0[32];                   // (offset: 0x1000-0x107C) Word pin registers P0.0 - P0.31
    __IO uint32_t W1[32];                   // (offset: 0x1080-0x10FC) Word pin registers P1.0 - P1.31
         uint32_t RESERVED1[960];           // (offset: 0x1100-0x1FFC) (960d = 0x3C0) RESERVED
    union
    {
        __IO uint32_t DIR[2];               // (offset: 0x2000-0x2004)
        struct
        {
            __IO uint32_t DIR0;             // (offset: 0x2000)
            __IO uint32_t DIR1;             // (offset: 0x2004)
        };
    };
    uint32_t RESERVED2[30];                 // (offset: 0x2008-0x207C) RESERVED
    union
    {
        __IO uint32_t MASK[2];              // (offset: 0x2080-0x2084)
        struct
        {
            __IO uint32_t MASK0;            // (offset: 0x2080)
            __IO uint32_t MASK1;            // (offset: 0x2084)
        };
    };
    uint32_t RESERVED3[30];                 // (offset: 0x2088-0x20FC) RESERVED
    union
    {
        __IO uint32_t PIN[2];               // (offset: 0x2100-0x2104)
        struct
        {
            __IO uint32_t PIN0;             // (offset: 0x2100)
            __IO uint32_t PIN1;             // (offset: 0x2104)
        };
    };
    uint32_t RESERVED4[30];                 // (offset: 0x2108-0x217C) RESERVED
    union
    {
        __IO uint32_t MPIN[2];              // (offset: 0x22180-0x2184)
        struct
        {
            __IO uint32_t MPIN0;            // (offset: 0x2180)
            __IO uint32_t MPIN1;            // (offset: 0x2184)
        };
    };
    uint32_t RESERVED5[30];                 // (offset: 0x2188-0x21FC) RESERVED
    union
    {
        __IO uint32_t SET[2];               // (offset: 0x2200-0x2204)
        struct
        {
            __IO uint32_t SET0;             // (offset: 0x2200)
            __IO uint32_t SET1;             // (offset: 0x2204)
        };
    };
    uint32_t RESERVED6[30];                 // (offset: 0x2208-0x227C) RESERVED
    union
    {
        __O  uint32_t CLR[2];               // (offset: 0x2280-0x2284)
        struct
        {
            __O  uint32_t CLR0;             // (offset: 0x2280)
            __O  uint32_t CLR1;             // (offset: 0x2284)
        };
    };
    uint32_t RESERVED7[30];                 // (offset: 0x2288-0x22FC) RESERVED
    union
    {
        __O  uint32_t NOT[2];               // (offset: 0x2300-0x2304)
        struct
        {
            __O  uint32_t NOT0;             // (offset: 0x2300)
            __O  uint32_t NOT1;             // (offset: 0x2304)
        };
    };
    uint32_t RESERVED8[30];                 // (offset: 0x2308-0x237C) RESERVED
    union
    {
        __O  uint32_t DIRSET[2];            // (offset: 0x2380-0x2384)
        struct
        {
            __O  uint32_t DIRSET0;          // (offset: 0x2380)
            __O  uint32_t DIRSET1;          // (offset: 0x2384)
        };
    };
    uint32_t RESERVED9[30];                 // (offset: 0x2388-0x23FC) RESERVED
    union
    {
        __O  uint32_t DIRCLR[2];            // (offset: 0x2400-0x2404)
        struct
        {
            __O  uint32_t DIRCLR0;          // (offset: 0x2400)
            __O  uint32_t DIRCLR1;          // (offset: 0x2404)
        };
    };
    uint32_t RESERVED10[30];                // (offset: 0x2408-0x247C) RESERVED
    union
    {
        __O  uint32_t DIRNOT[2];            // (offset: 0x2480-0x2484)
        struct
        {
            __O  uint32_t DIRNOT0;          // (offset: 0x2480)
            __O  uint32_t DIRNOT1;          // (offset: 0x2484)
        };
    };
} LPC_GPIO_T;




// ------------ Pin Interrupts and Pattern Match (PIN_INT) --------------------
typedef struct
{
    __IO uint32_t ISEL;                     // (offset: 0x00) Pin Interrupt Mode register
    __IO uint32_t IENR;                     // (offset: 0x04) Pin Interrupt Enable (Rising) register
    __IO uint32_t SIENR;                    // (offset: 0x08) Set Pin Interrupt Enable (Rising) register
    __IO uint32_t CIENR;                    // (offset: 0x0C) Clear Pin Interrupt Enable (Rising) register
    __IO uint32_t IENF;                     // (offset: 0x10) Pin Interrupt Enable Falling Edge / Active Level register
    __IO uint32_t SIENF;                    // (offset: 0x14) Set Pin Interrupt Enable Falling Edge / Active Level register
    __IO uint32_t CIENF;                    // (offset: 0x18) Clear Pin Interrupt Enable Falling Edge / Active Level address
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
    __IO uint32_t MODE;
    __IO uint32_t SEED;
    union
    {
        __I  uint32_t SUM;
        __O  uint32_t WR_DATA;
    };
} LPC_CRC_T;




// ------------ Comparator (CMP) ----------------------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00) Comparator control register
    __IO uint32_t LAD;                      // (offset: 0x04) Voltage ladder register
} LPC_CMP_T;






// ------------ Self Wakeup Timer (WKT) ---------------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00)      Alarm / Wakeup Timer Control register
         uint32_t RESERVED[2];              // (offset: 0x04-0x08) RESERVED
    __IO uint32_t COUNT;                    // (offset: 0x0C)      Alarm / Wakeup TImer counter register
} LPC_WKT_T;




// ------------ Multi-Rate Timer(MRT) -----------------------------------------
typedef struct
{
    __IO uint32_t INTVAL;
    __IO uint32_t TIMER;
    __IO uint32_t CTRL;
    __IO uint32_t STAT;
} LPC_MRT_CHANNEL_T;

typedef struct
{
    LPC_MRT_CHANNEL_T   CHANNEL[4];         // (offset: 0x00-0x3C)

         uint32_t       RESERVED[45];       // (offset: 0x40-0xF0) RESERVED
    __IO uint32_t       IDLE_CH;            // (offset: 0xF4)
    __IO uint32_t       IRQ_FLAG;           // (offset: 0xF8)
} LPC_MRT_T;




// ------ Universal Synchronous Asynchronous Receiver Transmitter (USART) -----
typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t CTL;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __I  uint32_t RXDAT;
    __I  uint32_t RXDATSTAT;
    __IO uint32_t TXDAT;
    __IO uint32_t BRG;
    __I  uint32_t INTSTAT;
    __IO uint32_t OSR;
    __IO uint32_t ADDR;
} LPC_USART_T;




// ------------ Serial Peripheral Interface (SPI) -----------------------------
typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t DLY;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __I  uint32_t RXDAT;
    __IO uint32_t TXDATCTL;
    __IO uint32_t TXDAT;
    __IO uint32_t TXCTL;
    __IO uint32_t DIV;
    __I  uint32_t INTSTAT;
} LPC_SPI_T;




// ------------ Inter-Integrated Circuit (I2C) --------------------------------
typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __IO uint32_t TIMEOUT;
    union
    {
        __IO uint32_t CLKDIV;
        __IO uint32_t DIV;
    };
    __IO uint32_t INTSTAT;
         uint32_t RESERVED0;
    __IO uint32_t MSTCTL;
    __IO uint32_t MSTTIME;
    __IO uint32_t MSTDAT;
         uint32_t RESERVED1[5];
    __IO uint32_t SLVCTL;
    __IO uint32_t SLVDAT;
    __IO uint32_t SLVADR0;
    __IO uint32_t SLVADR1;
    __IO uint32_t SLVADR2;
    __IO uint32_t SLVADR3;
    __IO uint32_t SLVQUAL0;
         uint32_t RESERVED2[9];
    __I  uint32_t MONRXDAT;
} LPC_I2C_T;




// ------------ State Configurable Timer (SCT) --------------------------------
#define SCT_NUM_EVENTS      (8)             // Number of events
#define SCT_NUM_REGISTERS   (8)             // Number of match/compare registers
#define SCT_NUM_OUTPUTS     (7)             // Number of outputs

typedef struct
{
    __IO  uint32_t CONFIG;                  // (offset: 0x00)
    union
    {
        __IO uint32_t CTRL;                 // (offset: 0x04)
        struct
        {
            __IO uint16_t CTRL_L;
            __IO uint16_t CTRL_H;
        };
    };
    union
    {
        __IO uint32_t LIMIT;                // (offset: 0x08)
        struct
        {
            __IO uint16_t LIMIT_L;
            __IO uint16_t LIMIT_H;
        };
    };
    union
    {
        __IO uint32_t HALT;                 // (offset: 0x0C)
        struct
        {
            __IO uint16_t HALT_L;
            __IO uint16_t HALT_H;
        };
    };
    union
    {
        __IO uint32_t STOP;                 // (offset: 0x10)
        struct
        {
            __IO uint16_t STOP_L;
            __IO uint16_t STOP_H;
        };
    };
    union
    {
        __IO uint32_t START;                // (offset: 0x14)
        struct
        {
            __IO uint16_t START_L;
            __IO uint16_t START_H;
        };
    };
    uint32_t RESERVED0[10];                 // (offset: 0x18-0x3C) RESERVED
    union
    {
        __IO uint32_t COUNT;                // (offset: 0x40)
        struct
        {
            __IO uint16_t COUNT_L;
            __IO uint16_t COUNT_H;
        };
    };
    union
    {
        __IO uint32_t STATE;                // (offset: 0x44)
        struct
        {
            __IO uint16_t STATE_L;
            __IO uint16_t STATE_H;
        };
    };
    __I  uint32_t INPUT;                    // (offset: 0x48)
    union
    {
        __IO uint32_t REGMODE;              // (offset: 0x4C)
        struct
        {
            __IO uint16_t REGMODE_L;
            __IO uint16_t REGMODE_H;
        };
    };
    __IO uint32_t OUTPUT;                   // (offset: 0x050)
    __IO uint32_t OUTPUTDIRCTRL;            // (offset: 0x054)
    __IO uint32_t RES;                      // (offset: 0x058)
    __IO uint32_t DMAREQ0;                  // (offset: 0x05C)
    __IO uint32_t DMAREQ1;                  // (offset: 0x060)
         uint32_t RESERVED1[35];            // (offset: 0x064-0x0EC) RESERVED
    __IO uint32_t EVEN;                     // (offset: 0x0F0)
    __IO uint32_t EVFLAG;                   // (offset: 0x0F4)
    __IO uint32_t CONEN;                    // (offset: 0x0F8)
    __IO uint32_t CONFLAG;                  // (offset: 0x0FC)
    union                                   // (offset: 0x100-0x13C) Match / Capture
    {
        __IO union
        {
            uint32_t U;                     // MATCH[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 // MATCH[i].L  Access to L value
                uint16_t H;                 // MATCH[i].H  Access to H value
            };
        } MATCH[SCT_NUM_REGISTERS];

        __I union
        {
            uint32_t U;                     // CAP[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 // CAP[i].L  Access to L value
                uint16_t H;                 // CAP[i].H  Access to H value
            };
        } CAP[SCT_NUM_REGISTERS];
    };
    uint32_t RESERVED2[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x17C) RESERVED
    union
    {
        __IO uint16_t MATCH_L[SCT_NUM_REGISTERS];       // (offset: 0x180-0x...) Match Value L counter
        __I  uint16_t CAP_L[SCT_NUM_REGISTERS];         // (offset: 0x180-0x...) Capture Value L counter
    };
    uint16_t RESERVED3[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x1BE) RESERVED
    union
    {
        __IO uint16_t MATCH_H[SCT_NUM_REGISTERS];       // (offset: 0x1C0-0x...) Match Value H counter
        __I  uint16_t CAP_H[SCT_NUM_REGISTERS];         // (offset: 0x1C0-0x...) Capture Value H counter
    };
    uint16_t RESERVED4[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x1FE) RESERVED
    union                                               // (offset: 0x200-0x...) Match Reload / Capture Control value
    {
        __IO union
        {
            uint32_t U;                                 // MATCHREL[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                             // MATCHREL[i].L  Access to L value
                uint16_t H;                             // MATCHREL[i].H  Access to H value
            };
        } MATCHREL[SCT_NUM_REGISTERS];
        __IO union
        {
            uint32_t U;                                 // CAPCTRL[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                             // CAPCTRL[i].L  Access to L value
                uint16_t H;                             // SCTCAPCTRL[i].H  Access to H value
            };
        } CAPCTRL[SCT_NUM_REGISTERS];
    };
    uint32_t RESERVED5[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x27C) RESERVED
    union
    {
        __IO uint16_t MATCHREL_L[SCT_NUM_REGISTERS];    // (offset: 0x280-0x...) Match Reload value L counter
        __IO uint16_t CAPCTRL_L[SCT_NUM_REGISTERS];     // (offset: 0x280-0x...) Capture Control value L counter
    };
    uint16_t RESERVED6[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x2BE) RESERVED
    union
    {
        __IO uint16_t MATCHREL_H[SCT_NUM_REGISTERS];    // (offset: 0x2C0-0x...) Match Reload value H counter
        __IO uint16_t CAPCTRL_H[SCT_NUM_REGISTERS];     // (offset: 0x2C0-0x...) Capture Control value H counter
    };
    uint16_t RESERVED7[32 - SCT_NUM_REGISTERS];         // (offset: 0x...-0x2FE) RESERVED
    __IO struct                                         // (offset: 0x300-0x3FC) EVENT[i].STATE / EVENT[i].CTRL
    {
        uint32_t STATE;
        uint32_t CTRL;
    } EVENT[SCT_NUM_EVENTS];
    uint32_t RESERVED8[128 - (2 * SCT_NUM_EVENTS)];     // (offset: 0x...-0x4FC) RESERVED
    __IO struct                                         // (offset: 0x500-0x57C) OUT[n].SET / OUT[n].CLR
    {
        uint32_t SET;                                   // Output n Set Register
        uint32_t CLR;                                   // Output n Clear Register
    } OUT[SCT_NUM_OUTPUTS];
    uint32_t RESERVED9[((0x300 / 4) - 1) - (2 * SCT_NUM_OUTPUTS)];  // (offset: 0x...-0x7F8) RESERVED
    __I  uint32_t MODULECONTENT;                                    // (offset: 0x7FC)       Module Content
} LPC_SCT_T;




// ------------ Standard Counter/Timer (CTIMER) -------------------------------
typedef struct
{
    __IO uint32_t IR;                       // (offset: 0x00)
    __IO uint32_t TCR;                      // (offset: 0x04)
    __IO uint32_t TC;                       // (offset: 0x08)
    __IO uint32_t PR;                       // (offset: 0x0C)
    __IO uint32_t PC;                       // (offset: 0x10)
    __IO uint32_t MCR;                      // (offset: 0x14)
    __IO uint32_t MR[4];                    // (offset: 0x18-0x24)
    __IO uint32_t CCR;                      // (offset: 0x28)
    __IO uint32_t CR[4];                    // (offset: 0x2C-0x38)
    __IO uint32_t EMR;                      // (offset: 0x3C)
         uint32_t RESERVED[12];             // (offset: 0x40-0x6C) RESERVED
    __IO uint32_t CTCR;                     // (offset: 0x70)
    __IO uint32_t PWMC;                     // (offset: 0x74)
} LPC_CTIMER_T;




// ------------ Windowed Watchdog Timer (WWDT) --------------------------------
typedef struct
{
    __IO uint32_t MOD;                      // (offset: 0x000) Watchdog mode register
    __IO uint32_t TC;                       // (offset: 0x004) Watchdog timer constant register
    __O  uint32_t FEED;                     // (offset: 0x008) Watchdog feed sequence register
    __I  uint32_t TV;                       // (offset: 0x00C) Watchdog timer value register
         uint32_t RESERVED;                 // (offset: 0x010) RESERVED
    __IO uint32_t WARNINT;                  // (offset: 0x014) Watchdog timer warning int. register
    __IO uint32_t WINDOW;                   // (offset: 0x018) Watchdog timer window value register
} LPC_WWDT_T;




// ----------------------------------------------------------------------------
// Input multiplexing and DMA trigger multiplexing (INPUT MUX, DMA TRIGMUX)
// ----------------------------------------------------------------------------
typedef struct
{
    __IO uint32_t DMA_INMUX_INMUX0;         // (offset: 0x00)
    __IO uint32_t DMA_INMUX_INMUX1;         // (offset: 0x04)
         uint32_t RESERVED0[6];             // (offset: 0x08-0x1C) RESERVED
    __IO uint32_t SCT0_INMUX0;              // (offset: 0x20)
    __IO uint32_t SCT0_INMUX1;              // (offset: 0x24)
    __IO uint32_t SCT0_INMUX2;              // (offset: 0x28)
    __IO uint32_t SCT0_INMUX3;              // (offset: 0x2C)
         uint32_t RESERVED1[4];             // (offset: 0x30-0x3C) RESERVED
    __IO uint32_t DMA_ITRIG_INMUX0;         // (offset: 0x40)
    __IO uint32_t DMA_ITRIG_INMUX1;         // (offset: 0x44)
    __IO uint32_t DMA_ITRIG_INMUX2;         // (offset: 0x48)
    __IO uint32_t DMA_ITRIG_INMUX3;         // (offset: 0x4C)
    __IO uint32_t DMA_ITRIG_INMUX4;         // (offset: 0x50)
    __IO uint32_t DMA_ITRIG_INMUX5;         // (offset: 0x54)
    __IO uint32_t DMA_ITRIG_INMUX6;         // (offset: 0x58)
    __IO uint32_t DMA_ITRIG_INMUX7;         // (offset: 0x5C)
    __IO uint32_t DMA_ITRIG_INMUX8;         // (offset: 0x60)
    __IO uint32_t DMA_ITRIG_INMUX9;         // (offset: 0x64)
    __IO uint32_t DMA_ITRIG_INMUX10;        // (offset: 0x68)
    __IO uint32_t DMA_ITRIG_INMUX11;        // (offset: 0x6C)
    __IO uint32_t DMA_ITRIG_INMUX12;        // (offset: 0x70)
    __IO uint32_t DMA_ITRIG_INMUX13;        // (offset: 0x74)
    __IO uint32_t DMA_ITRIG_INMUX14;        // (offset: 0x78)
    __IO uint32_t DMA_ITRIG_INMUX15;        // (offset: 0x7C)
    __IO uint32_t DMA_ITRIG_INMUX16;        // (offset: 0x80)
    __IO uint32_t DMA_ITRIG_INMUX17;        // (offset: 0x84)
    __IO uint32_t DMA_ITRIG_INMUX18;        // (offset: 0x88)
    __IO uint32_t DMA_ITRIG_INMUX19;        // (offset: 0x8C)
    __IO uint32_t DMA_ITRIG_INMUX20;        // (offset: 0x90)
    __IO uint32_t DMA_ITRIG_INMUX21;        // (offset: 0x94)
    __IO uint32_t DMA_ITRIG_INMUX22;        // (offset: 0x98)
    __IO uint32_t DMA_ITRIG_INMUX23;        // (offset: 0x9C)
    __IO uint32_t DMA_ITRIG_INMUX24;        // (offset: 0xA0)
} LPC_INMUX_TRIGMUX_T;




// ------------ Analog-to-Digital Converter (ADC) -----------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x00)
         uint32_t RESERVED0;                // (offset: 0x04) RESERVED
    __IO uint32_t SEQA_CTRL;                // (offset: 0x08)
    __IO uint32_t SEQB_CTRL;                // (offset: 0x0C)
    __IO uint32_t SEQA_GDAT;                // (offset: 0x10)
    __IO uint32_t SEQB_GDAT;                // (offset: 0x14)
         uint32_t RESERVED1[2];             // (offset: 0x18-0x1C) RESERVED
    __IO uint32_t DAT[12];                  // (offset: 0x20-0x4C)
    __IO uint32_t THR0_LOW;                 // (offset: 0x50)
    __IO uint32_t THR1_LOW;                 // (offset: 0x54)
    __IO uint32_t THR0_HIGH;                // (offset: 0x58)
    __IO uint32_t THR1_HIGH;                // (offset: 0x5C)
    __IO uint32_t CHAN_THRSEL;              // (offset: 0x60)
    __IO uint32_t INTEN;                    // (offset: 0x64)
    __IO uint32_t FLAGS;                    // (offset: 0x68)
    __IO uint32_t TRM;                      // (offset: 0x6C)
} LPC_ADC_T;




// ------------ Direct memory access (DMA) ------------------------------------
#define DMA_NUM_CHANNELS        25

typedef struct
{
    __IO uint32_t CFG;
    __I  uint32_t CTLSTAT;
    __IO uint32_t XFERCFG;
         uint32_t RESERVED;
} LPC_DMA_CHANNEL_T;

typedef struct
{
    __IO uint32_t       CTRL;                       // (offset: 0x000)
    __I  uint32_t       INTSTAT;                    // (offset: 0x004)
    __IO uint32_t       SRAMBASE;                   // (offset: 0x008)
         uint32_t       RESERVED0[5];               // (offset: 0x010-0x01C) RESERVED
    __IO uint32_t       ENABLESET0;                 // (offset: 0x020)
         uint32_t       RESERVED1;                  // (offset: 0x024)       RESERVED
    __O  uint32_t       ENABLECLR0;                 // (offset: 0x028)
         uint32_t       RESERVED2;                  // (offset: 0x02C)       RESERVED
    __I  uint32_t       ACTIVE0;                    // (offset: 0x030)
         uint32_t       RESERVED3;                  // (offset: 0x034)       RESERVED
    __I  uint32_t       BUSY0;                      // (offset: 0x038)
         uint32_t       RESERVED4;                  // (offset: 0x03C)       RESERVED
    __IO uint32_t       ERRINT0;                    // (offset: 0x040)
         uint32_t       RESERVED5;                  // (offset: 0x044)       RESERVED
    __IO uint32_t       INTENSET0;                  // (offset: 0x048)
         uint32_t       RESERVED6;                  // (offset: 0x04C)       RESERVED
    __O  uint32_t       INTENCLR0;                  // (offset: 0x050)
         uint32_t       RESERVED7;                  // (offset: 0x054)       RESERVED
    __IO uint32_t       INTA0;                      // (offset: 0x058)
         uint32_t       RESERVED8;                  // (offset: 0x05C)       RESERVED
    __IO uint32_t       INTB0;                      // (offset: 0x060)
         uint32_t       RESERVED9;                  // (offset: 0x064)       RESERVED
    __O  uint32_t       SETVALID0;                  // (offset: 0x068)
         uint32_t       RESERVED10;                 // (offset: 0x06C)       RESERVED
    __O  uint32_t       SETTRIG0;                   // (offset: 0x070)
         uint32_t       RESERVED11;                 // (offset: 0x074)       RESERVED
    __O  uint32_t       ABORT0;                     // (offset: 0x078)
         uint32_t       RESERVED12[225];            // (offset: 0x07C-0x3FC) RESERVED

    LPC_DMA_CHANNEL_T   CHANNEL[DMA_NUM_CHANNELS];  // (offset: 0x400-0x...)
} LPC_DMA_T;




// ------------ Digital-to-Analog Converter (DAC) -----------------------------
typedef struct
{
    __IO uint32_t CR;                       // (offset: 0x00)
    __IO uint32_t CTRL;                     // (offset: 0x04)
    __IO uint32_t CNTVAL;                   // (offset: 0x08)
} LPC_DAC_T;




// ------------ Capacitive Touch module (CAPT) --------------------------------
typedef struct
{
    __IO uint32_t CTRL;                     // (offset: 0x000)
    __IO uint32_t STATUS;                   // (offset: 0x004)
    __IO uint32_t POLL_TCNT;                // (offset: 0x008)
         uint32_t RESERVED0;                // (offset: 0x00C)
    __IO uint32_t INTENSET;                 // (offset: 0x010)
    __O  uint32_t INTENCLR;                 // (offset: 0x014)
    __I  uint32_t INTSTAT;                  // (offset: 0x018)
         uint32_t RESERVED1;                // (offset: 0x01C)       RESERVED
    __I  uint32_t TOUCH;                    // (offset: 0x020)
         uint32_t RESERVED2[1014];          // (offset: 0x024-0xFF8) RESERVED
    __I  uint32_t ID;                       // (offset: 0xFFC)
} LPC_CAPT_T;




// ------------ Fast Initialization Memory (FAIM) -----------------------------
typedef struct
{
    union
    {
        __IO uint32_t WORD[8];
        struct
        {
            __IO uint32_t WORD0;
            __IO uint32_t WORD1;
            __IO uint32_t WORD2;
            __IO uint32_t WORD3;
            __IO uint32_t WORD4;
            __IO uint32_t WORD5;
            __IO uint32_t WORD6;
            __IO uint32_t WORD7;
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
  const uint32_t            RESERVED0[3];   // (offset: 0x00-0x08) RESERVED
  const LPC_ROM_PWR_API_T*  PWR_BASE_PTR;   // (offset: 0x0C)      Power APIs function table base address
  const LPC_ROM_DIV_API_T*  DIV_BASE_PTR;   // (offset: 0x10)      Integer division routines function table base address
  const uint32_t            RESERVED1[7];   // (Offsets 0x14-0x2C) RESERVED
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
