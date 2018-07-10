// ----------------------------------------------------------------------------
// @file    lpc81x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for NXP LPC81x MCUs.
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

// ----------------------------------------------------------------------------
// This file is based on the code from LPCOpen v3.02 for
// LPCXpresso v8.2.0 (LPCXpresso812 board (V1)) provided by NXP.
// https://www.nxp.com/downloads/en/libraries/lpcopen_3_02_lpcxpresso_nxp_lpcxpresso_812.zip
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_TARGETS_LPC81X_CMSIS_HPP
#define __XARMLIB_TARGETS_LPC81X_CMSIS_HPP

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

    // LPC81x Specific Interrupt Numbers
    SPI0_IRQn                   = 0,        // SPI0 interrupt
    SPI1_IRQn                   = 1,        // SPI1 interrupt
    Reserved0_IRQn              = 2,        // Reserved interrupt
    USART0_IRQn                 = 3,        // USART0 interrupt
    USART1_IRQn                 = 4,        // USART1 interrupt
    USART2_IRQn                 = 5,        // USART2 interrupt
    Reserved1_IRQn              = 6,        // Reserved interrupt
    Reserved2_IRQn              = 7,        // Reserved interrupt
    I2C_IRQn                    = 8,        // I2C interrupt
    SCT_IRQn                    = 9,        // SCT interrupt
    MRT_IRQn                    = 10,       // MRT interrupt
    ACMP_IRQn                   = 11,       // Analog Comparator interrupt
    WDT_IRQn                    = 12,       // WDT interrupt
    BOD_IRQn                    = 13,       // BOD interrupt
    Reserved3_IRQn              = 14,       // Reserved interrupt
    WKT_IRQn                    = 15,       // WKT interrupt
    Reserved4_IRQn              = 16,       // Reserved interrupt
    Reserved5_IRQn              = 17,       // Reserved interrupt
    Reserved6_IRQn              = 18,       // Reserved interrupt
    Reserved7_IRQn              = 19,       // Reserved interrupt
    Reserved8_IRQn              = 20,       // Reserved interrupt
    Reserved9_IRQn              = 21,       // Reserved interrupt
    Reserved10_IRQn             = 22,       // Reserved interrupt
    Reserved11_IRQn             = 23,       // Reserved interrupt
    PININT0_IRQn                = 24,       // Pin interrupt 0
    PININT1_IRQn                = 25,       // Pin interrupt 1
    PININT2_IRQn                = 26,       // Pin interrupt 2
    PININT3_IRQn                = 27,       // Pin interrupt 3
    PININT4_IRQn                = 28,       // Pin interrupt 4
    PININT5_IRQn                = 29,       // Pin interrupt 5
    PININT6_IRQn                = 30,       // Pin interrupt 6
    PININT7_IRQn                = 31,       // Pin interrupt 7
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
    __IO uint32_t PRESETCTRL;               // (offset: 0x004) Peripheral reset control
    __IO uint32_t SYSPLLCTRL;               // (offset: 0x008) System PLL control
    __I  uint32_t SYSPLLSTAT;               // (offset: 0x00C) System PLL status
         uint32_t RESERVED0[4];
    __IO uint32_t SYSOSCCTRL;               // (offset: 0x020) System oscillator control
    __IO uint32_t WDTOSCCTRL;               // (offset: 0x024) Watchdog oscillator control
         uint32_t RESERVED1[2];
    __IO uint32_t SYSRSTSTAT;               // (offset: 0x030) System reset status register
         uint32_t RESERVED2[3];
    __IO uint32_t SYSPLLCLKSEL;             // (offset: 0x040) System PLL clock source select
    __IO uint32_t SYSPLLCLKUEN;             // (offset: 0x044) System PLL clock source update enable
         uint32_t RESERVED3[10];
    __IO uint32_t MAINCLKSEL;               // (offset: 0x070) Main clock source select
    __IO uint32_t MAINCLKUEN;               // (offset: 0x074) Main clock source update enable
    __IO uint32_t SYSAHBCLKDIV;             // (offset: 0x078) System AHB clock divider
         uint32_t RESERVED4;
    __IO uint32_t SYSAHBCLKCTRL;            // (offset: 0x080) System AHB clock control
         uint32_t RESERVED5[4];
    __IO uint32_t UARTCLKDIV;               // (offset: 0x094) USART clock divider
         uint32_t RESERVED6[18];
    __IO uint32_t CLKOUTSEL;                // (offset: 0x0E0) CLKOUT clock source select
    __IO uint32_t CLKOUTUEN;                // (offset: 0x0E4) CLKOUT clock source update enable
    __IO uint32_t CLKOUTDIV;                // (offset: 0x0E8) CLKOUT clock divider
         uint32_t RESERVED7;
    __IO uint32_t UARTFRGDIV;               // (offset: 0x0F0) UART fractional divider SUB
    __IO uint32_t UARTFRGMULT;              // (offset: 0x0F4) UART fractional divider ADD
         uint32_t RESERVED8;
    __IO uint32_t EXTTRACECMD;              // (offset: 0x0FC) External trace buffer command register
    __IO uint32_t PIOPORCAP0;               // (offset: 0x100) POR captured PIO status 0
         uint32_t RESERVED9[12];
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
         uint32_t RESERVED10[6];            // (offset: 0x158) RESERVED
    __IO uint32_t IRQLATENCY;               // (offset: 0x170) IRQ delay
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
         uint32_t RESERVED11[27];           // (offset: 0x198) RESERVED
    __IO uint32_t STARTERP0;                // (offset: 0x204) Start logic 0 pin wake-up enable
         uint32_t RESERVED12[3];            // (offset: 0x208) RESERVED
    __IO uint32_t STARTERP1;                // (offset: 0x214) Start logic 1 interrupt wake-up enable
         uint32_t RESERVED13[6];            // (offset: 0x218) RESERVED
    __IO uint32_t PDSLEEPCFG;               // (offset: 0x230) Power-down states in deep-sleep mode
    __IO uint32_t PDAWAKECFG;               // (offset: 0x234) Power-down states for wake-up from deep-sleep
    __IO uint32_t PDRUNCFG;                 // (offset: 0x238) Power configuration
         uint32_t RESERVED14[111];          // (offset: 0x23C) RESERVED
    __I  uint32_t DEVICEID;                 // (offset: 0x3F8) Device ID
} LPC_SYSCON_T;




// ------------ I/O Configuration (IOCON) -------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PIO[19];              // Digital I/O control for ports
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
        };
    };
} LPC_IOCON_T;




// ------------ FLASH Memory Controller (FMC) ---------------------------------
typedef struct
{
         uint32_t RESERVED0[4];             // (offset: 0x00) RESERVED
    __IO uint32_t FLASHCFG;                 // (offset: 0x10) Flash configuration register
         uint32_t RESERVED1[3];             // (offset: 0x14) RESERVED
    __IO uint32_t FMSSTART;                 // (offset: 0x20) Signature start address register
    __IO uint32_t FMSSTOP;                  // (offset: 0x24) Signature stop-address register
         uint32_t RESERVED2;                // (offset: 0x28) RESERVED
    __I  uint32_t FMSW0;                    // (offset: 0x2C) Signature word
} LPC_FMC_T;




// ------------ Power Management Unit (PMU) -----------------------------------
typedef struct
{
    __IO uint32_t PCON;                     // (offset: 0x00) Power control register
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
        __IO uint32_t PINASSIGN[9];         // Pin Assign register array
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
        };
    };
         uint32_t RESERVED0[103];           // RESERVED
    __IO uint32_t PINENABLE0;               // (offset: 0x1C0) Pin Enable register
} LPC_SWM_T;




// ------------ General Purpose I/O (GPIO) ------------------------------------
typedef struct
{
    union
    {
        __IO uint8_t B[1][18];              // (offset: 0x0000) Byte pin registers P0.0 - P0.17
        __IO uint8_t B0[18];
    };
    uint32_t RESERVED0[1019];               // (offset: 0x0012) RESERVED
    union
    {
        __IO uint32_t W[1][18];             // (offset: 0x1000) Word pin registers P0.0 - P0.17
        __IO uint32_t W0[18];
    };
    uint32_t RESERVED1[1006];               // (offset: 0x1060) RESERVED
    union
    {
        __IO uint32_t DIR[1];               // (offset: 0x2000) Port 0 direction register
        __IO uint32_t DIR0;
    };
    uint32_t RESERVED2[31];                 // (offset: 0x2004) RESERVED
    union
    {
        __IO uint32_t MASK[1];              // (offset: 0x2080) Port 0 mask register
        __IO uint32_t MASK0;
    };
    uint32_t RESERVED3[31];                 // (offset: 0x2084) RESERVED
    union
    {
        __IO uint32_t PIN[1];               // (offset: 0x2100) Port 0 pin register
        __IO uint32_t PIN0;
    };
    uint32_t RESERVED4[31];                 // (offset: 0x2104) RESERVED
    union
    {
        __IO uint32_t MPIN[1];              // (offset: 0x2180) Masked port 0 register
        __IO uint32_t MPIN0;
    };
    uint32_t RESERVED5[31];                 // (offset: 0x2184) RESERVED
    union
    {
        __IO uint32_t SET[1];               // (offset: 0x2200) Set port 0 register
        __IO uint32_t SET0;
    };
    uint32_t RESERVED6[31];                 // (offset: 0x2204) RESERVED
    union
    {
        __O  uint32_t CLR[1];               // (offset: 0x2280) Clear port 0 register
        __O  uint32_t CLR0;
    };
    uint32_t RESERVED7[31];                 // (offset: 0x2284) RESERVED
    union
    {
        __O  uint32_t NOT[1];               // (offset: 0x2300) Toggle port 0 register
        __O  uint32_t NOT0;
    };
} LPC_GPIO_T;




// ------------ Pin Interrupts and Pattern Match (PININT) ---------------------
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
} LPC_PININT_T;




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
    __IO uint32_t COUNT;                    // (offset: 0x0C) Alarm / Wakeup Timer counter register
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
    LPC_MRT_CHANNEL_T   CHANNEL[4];         // (offset: 0x00-0x3C) Channels

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
#define SCT_NUM_EVENTS      (6)             // Number of events
#define SCT_NUM_REGISTERS   (5)             // Number of match/compare registers
#define SCT_NUM_OUTPUTS     (4)             // Number of outputs

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
         uint32_t RESERVED1[37];            // (offset: 0x05C) RESERVED
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
    uint32_t RESERVED2[0x100 - (4 * SCT_NUM_REGISTERS)];    // (offset: 0x114) RESERVED
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
    uint32_t RESERVED5[0x100 - (4 * SCT_NUM_REGISTERS)];    // (offset: 0x214) RESERVED
    struct                                                  // (offset: 0x300) State / Control
    {
        __IO uint32_t STATE;                // EVENT[n].STATE  SCT Event n State register
        __IO uint32_t CTRL;                 // EVENT[n].CTRL   SCT Event n Control register
    } EVENT[SCT_NUM_EVENTS];
    uint32_t RESERVED8[0x200 - (8 * SCT_NUM_EVENTS)];       // (offset: 0x330) RESERVED
    struct                                                  // (offset: 0x500) Output Set / Clear
    {
        __IO uint32_t SET;                  // OUT[n].SET  Output n Set Register
        __IO uint32_t CLR;                  // OUT[n].CLR  Output n Clear Register
    } OUT[SCT_NUM_OUTPUTS];
} LPC_SCT_T;




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




// ------------ ROM API -------------------------------------------------------

// Error code returned by Boot ROM drivers/library functions enumeration
// NOTE: Error codes are a 32-bit value with:
//       - The 16 MSB contains the peripheral code number
//       - The 16 LSB contains an error code number associated to that peripheral
typedef enum
{
  /*0x00000000*/ LPC_OK = 0,                            // Enum value returned on Successful completion
  /*0x00000001*/ LPC_ERROR,                             // Enum value returned on general error (I2C)

  // ISP related errors
  ERR_ISP_BASE = 0x00000000,
  /*0x00000001*/ ERR_ISP_INVALID_COMMAND = ERR_ISP_BASE + 1,
  /*0x00000002*/ ERR_ISP_SRC_ADDR_ERROR,                // Source address not on word boundary
  /*0x00000003*/ ERR_ISP_DST_ADDR_ERROR,                // Destination address not on word or 256 byte boundary
  /*0x00000004*/ ERR_ISP_SRC_ADDR_NOT_MAPPED,
  /*0x00000005*/ ERR_ISP_DST_ADDR_NOT_MAPPED,
  /*0x00000006*/ ERR_ISP_COUNT_ERROR,                   // Byte count is not multiple of 4 or is not a permitted value
  /*0x00000007*/ ERR_ISP_INVALID_SECTOR,
  /*0x00000008*/ ERR_ISP_SECTOR_NOT_BLANK,
  /*0x00000009*/ ERR_ISP_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
  /*0x0000000A*/ ERR_ISP_COMPARE_ERROR,
  /*0x0000000B*/ ERR_ISP_BUSY,                          // Flash programming hardware interface is busy
  /*0x0000000C*/ ERR_ISP_PARAM_ERROR,                   // Insufficient number of parameters
  /*0x0000000D*/ ERR_ISP_ADDR_ERROR,                    // Address not on word boundary
  /*0x0000000E*/ ERR_ISP_ADDR_NOT_MAPPED,
  /*0x0000000F*/ ERR_ISP_CMD_LOCKED,                    // Command is locked
  /*0x00000010*/ ERR_ISP_INVALID_CODE,                  // Unlock code is invalid
  /*0x00000011*/ ERR_ISP_INVALID_BAUD_RATE,
  /*0x00000012*/ ERR_ISP_INVALID_STOP_BIT,
  /*0x00000013*/ ERR_ISP_CODE_READ_PROTECTION_ENABLED,

  // I2C related errors
  ERR_I2C_BASE = 0x00060000,
  /*0x00060001*/ ERR_I2C_NAK = ERR_I2C_BASE + 1,        // NAK
  /*0x00060002*/ ERR_I2C_BUFFER_OVERFLOW,               // Buffer overflow
  /*0x00060003*/ ERR_I2C_BYTE_COUNT_ERR,                // Byte count error
  /*0x00060004*/ ERR_I2C_LOSS_OF_ARBRITRATION,          // Loss of arbitration
  /*0x00060005*/ ERR_I2C_SLAVE_NOT_ADDRESSED,           // Slave not addressed
  /*0x00060006*/ ERR_I2C_LOSS_OF_ARBRITRATION_NAK_BIT,  // Loss arbritation NAK
  /*0x00060007*/ ERR_I2C_GENERAL_FAILURE,               // General failure
  /*0x00060008*/ ERR_I2C_REGS_SET_TO_DEFAULT,           // Set to default
  /*0x00060009*/ ERR_I2C_TIMEOUT,                       // I2C Timeout

  // UART related errors
  /*0x00080001*/ ERR_NO_ERROR = LPC_OK,                 // Receive is busy
  ERR_UART_BASE = 0x00080000,
  /*0x00080001*/ ERR_UART_RXD_BUSY = ERR_UART_BASE + 1, // Receive is busy
  /*0x00080002*/ ERR_UART_TXD_BUSY,                     // Transmit is busy
  /*0x00080003*/ ERR_UART_OVERRUN_FRAME_PARITY_NOISE,   // Overrun, Frame, Parity , Receive Noise error
  /*0x00080004*/ ERR_UART_UNDERRUN,                     // Underrun
  /*0x00080005*/ ERR_UART_PARAM,                        // Parameter error
} LPC_ROM_ERROR_CODE_E;




// Power API functions
typedef struct
{
    void (*set_pll)  (uint32_t cmd[], uint32_t resp[]);
    void (*set_power)(uint32_t cmd[], uint32_t resp[]);
} LPC_ROM_PWR_API_T;




// I2C ROM driver handle structure
typedef void *LPC_ROM_I2C_HANDLE_T;

// I2C ROM driver callback function
typedef void (*LPC_ROM_I2C_CALLBACK_T)(uint32_t err_code, uint32_t n);

// I2C ROM driver parameter structure
typedef struct
{
    uint32_t                num_bytes_send;     // No. of bytes to send
    uint32_t                num_bytes_rec;      // No. of bytes to receive
    uint8_t*                buffer_ptr_send;    // Pointer to send buffer
    uint8_t*                buffer_ptr_rec;     // Pointer to receive buffer
    LPC_ROM_I2C_CALLBACK_T  func_pt;            // Callback function
    uint8_t                 stop_flag;          // Stop flag
    uint8_t                 dummy[3];
} LPC_ROM_I2C_PARAM_T;

// I2C ROM driver result structure
typedef struct
{
    uint32_t    n_bytes_sent;  // No. of bytes sent
    uint32_t    n_bytes_recd;  // No. of bytes received
} LPC_ROM_I2C_RESULT_T;

// I2C ROM driver modes enumeration
typedef enum
{
    IDLE,           // IDLE state
    MASTER_SEND,    // Master send state
    MASTER_RECEIVE, // Master Receive state
    SLAVE_SEND,     // Slave send state
    SLAVE_RECEIVE   // Slave receive state
} LPC_ROM_I2C_MODE_E;

// I2C API functions
typedef struct
{
    // Interrupt Support Routine
    void (*i2c_isr_handler)(LPC_ROM_I2C_HANDLE_T *handle);
    // MASTER functions
    LPC_ROM_ERROR_CODE_E    (*i2c_master_transmit_poll)(LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_master_receive_poll) (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_master_tx_rx_poll)   (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_master_transmit_intr)(LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_master_receive_intr) (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_master_tx_rx_intr)   (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    // SLAVE functions
    LPC_ROM_ERROR_CODE_E    (*i2c_slave_receive_poll)  (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_slave_transmit_poll) (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_slave_receive_intr)  (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_slave_transmit_intr) (LPC_ROM_I2C_HANDLE_T *handle, LPC_ROM_I2C_PARAM_T *param, LPC_ROM_I2C_RESULT_T *result);
    LPC_ROM_ERROR_CODE_E    (*i2c_set_slave_addr)      (LPC_ROM_I2C_HANDLE_T *handle, uint32_t slave_addr_0_3, uint32_t slave_mask_0_3);
    // OTHER support functions
    uint32_t                (*i2c_get_mem_size)        (void);
    LPC_ROM_I2C_HANDLE_T*   (*i2c_setup)               (uint32_t i2c_base_addr, uint32_t * start_of_ram);
    LPC_ROM_ERROR_CODE_E    (*i2c_set_bitrate)         (LPC_ROM_I2C_HANDLE_T *handle, uint32_t  p_clk_in_hz, uint32_t bitrate_in_bps);
    uint32_t                (*i2c_get_firmware_version)(void);
    LPC_ROM_I2C_MODE_E      (*i2c_get_status)          (LPC_ROM_I2C_HANDLE_T *handle);
    LPC_ROM_ERROR_CODE_E    (*i2c_set_timeout)         (LPC_ROM_I2C_HANDLE_T *handle, uint32_t timeout);
} LPC_ROM_I2C_API_T;




// UART ROM driver UART handle
typedef void LPC_ROM_UART_HANDLE_T;

// UART ROM driver UART callback function
typedef void (*LPC_ROM_UART_CALLBACK_T)(uint32_t err_code, uint32_t n);

// UART ROM driver UART DMA callback function
typedef void (*LPC_ROM_UART_DMA_REQ_T)(uint32_t src_adr, uint32_t dst_adr, uint32_t size);

// UART ROM driver configuration structure
typedef struct
{
    uint32_t sys_clk_in_hz;     // main clock in Hz
    uint32_t baudrate_in_hz;    // Baud rate in Hz
    uint8_t  config;            // Configuration value
                                //  bit1:0  Data Length: 00: 7 bits length, 01: 8 bits length, others: reserved
                                //  bit3:2  Parity: 00: No Parity, 01: reserved, 10: Even, 11: Odd
                                //  bit4:   Stop Bit(s): 0: 1 Stop bit, 1: 2 Stop bits
    uint8_t  sync_mod;          // Sync mode settings
                                //  bit0:  Mode: 0: Asynchronous mode, 1: Synchronous  mode
                                //  bit1:  0: Un_RXD is sampled on the falling edge of SCLK
                                //         1: Un_RXD is sampled on the rising edge of SCLK
                                //  bit2:  0: Start and stop bits are transmitted as in asynchronous mode)
                                //         1: Start and stop bits are not transmitted)
                                //  bit3:  0: The UART is a  slave in Synchronous mode
                                //         1: The UART is a master in Synchronous mode
    uint16_t error_en;          // Errors to be enabled
                                //  bit0: Overrun Errors Enabled
                                //  bit1: Underrun Errors Enabled
                                //  bit2: FrameErr Errors Enabled
                                //  bit3: ParityErr Errors Enabled
                                //  bit4: RxNoise Errors Enabled
} LPC_ROM_UART_CONFIG_T;

// UART ROM driver parameter structure
typedef struct
{
    uint8_t*    buffer;         // Pointer to data buffer
    uint32_t    size;           // Size of the buffer
    uint16_t    transfer_mode;  // Transfer mode settings
                                //   0x00: uart_get_line: stop transfer when the buffer is full
                                //   0x00: uart_put_line: stop transfer when the buffer is empty
                                //   0x01: uart_get_line: stop transfer when CRLF are received
                                //   0x01: uart_put_line: transfer stopped after reaching \0 and CRLF is sent out after that
                                //   0x02: uart_get_line: stop transfer when LF are received
                                //   0x02: uart_put_line: transfer stopped after reaching \0 and LF is sent out after that
                                //   0x03: uart_get_line: RESERVED
                                //   0x03: uart_put_line: transfer stopped after reaching \0
    uint16_t    driver_mode;    // Driver mode
                                //  0x00: Polling mode, function blocked until transfer completes
                                //  0x01: Interrupt mode, function immediately returns, callback invoked when transfer completes
                                //  0x02: DMA mode, in case DMA block is available, DMA req function is called for UART DMA channel setup,
                                //          then callback function indicate that transfer completes
    LPC_ROM_UART_CALLBACK_T callback_func_pt;   // callback function pointer
    LPC_ROM_UART_DMA_REQ_T  dma_req_func_pt;    // UART DMA channel setup function pointer, not applicable on LPC8xx
} LPC_ROM_UART_PARAM_T;

// UART API functions
typedef struct
{
    // Configuration functions
    uint32_t                (*uart_get_mem_size)(void); // Get the memory size needed by one Min UART instance
    LPC_ROM_UART_HANDLE_T*  (*uart_setup)(uint32_t base_addr, uint8_t * ram);   // Setup Min UART instance with provided memory and return the handle to this instance
    uint32_t                (*uart_init) (LPC_ROM_UART_HANDLE_T *handle, LPC_ROM_UART_CONFIG_T *set);   // Setup baud rate and operation mode for uart, then enable uart

    // Polling functions block until completed
    uint8_t                 (*uart_get_char)(LPC_ROM_UART_HANDLE_T *handle);                                // Receive one Char from uart.
                                                                                                            // This functions is only returned after Char is received.
                                                                                                            // In case Echo is enabled, the received data is sent out immediately
    void                    (*uart_put_char)(LPC_ROM_UART_HANDLE_T *handle, uint8_t data);                  // Send one Char through uart.
                                                                                                            // This function is only returned after data is sent
    uint32_t                (*uart_get_line)(LPC_ROM_UART_HANDLE_T *handle, LPC_ROM_UART_PARAM_T *param);   // Receive multiple bytes from UART
    uint32_t                (*uart_put_line)(LPC_ROM_UART_HANDLE_T *handle, LPC_ROM_UART_PARAM_T *param);   // Send string (end with \0) or raw data through UART

    // Interrupt functions return immediately and callback when completed
    void                    (*uart_isr)(LPC_ROM_UART_HANDLE_T *handle); // UART interrupt service routine.
                                                                        // To use this routine, the corresponding USART interrupt must be enabled.
                                                                        // This function is invoked by the user ISR
} LPC_ROM_UART_API_T;




// The master structure that defines the table of all ROM APIs on the device - ROM Driver table
typedef struct
{
    const uint32_t              RESERVED0[3];   // RESERVED
    const LPC_ROM_PWR_API_T*    PWR_BASE_PTR;   // Power APIs function table base address
    const uint32_t              RESERVED1;      // RESERVED
    const LPC_ROM_I2C_API_T*    I2C_BASE_PTR;   // I2C APIs function table base address
    const uint32_t              RESERVED2[3];   // RESERVED
    const LPC_ROM_UART_API_T*   UART_BASE_PTR;  // UART APIs function table base address
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
#define LPC_RAM_BASE            (0x10000000UL)
#define LPC_ROM_BASE            (0x1FFF0000UL)
#define LPC_APB_BASE            (0x40000000UL)
#define LPC_AHB_BASE            (0x50000000UL)
#define LPC_GPIO_BASE           (0xA0000000UL)

// ROM Driver table
#define LPC_ROM_DRIVER_BASE     (LPC_ROM_BASE + 0x1FF8)
#define LPC_ROM_IAP_BASE        (LPC_ROM_BASE + 0x1FF1)

// APB peripherals
#define LPC_WWDT_BASE           (LPC_APB_BASE + 0x00000)
#define LPC_MRT_BASE            (LPC_APB_BASE + 0x04000)
#define LPC_WKT_BASE            (LPC_APB_BASE + 0x08000)
#define LPC_SWM_BASE            (LPC_APB_BASE + 0x0C000)
#define LPC_PMU_BASE            (LPC_APB_BASE + 0x20000)
#define LPC_ACMP_BASE           (LPC_APB_BASE + 0x24000)
#define LPC_FMC_BASE            (LPC_APB_BASE + 0x40000)
#define LPC_IOCON_BASE          (LPC_APB_BASE + 0x44000)
#define LPC_SYSCON_BASE         (LPC_APB_BASE + 0x48000)
#define LPC_I2C_BASE            (LPC_APB_BASE + 0x50000)
#define LPC_SPI0_BASE           (LPC_APB_BASE + 0x58000)
#define LPC_SPI1_BASE           (LPC_APB_BASE + 0x5C000)
#define LPC_USART0_BASE         (LPC_APB_BASE + 0x64000)
#define LPC_USART1_BASE         (LPC_APB_BASE + 0x68000)
#define LPC_USART2_BASE         (LPC_APB_BASE + 0x6C000)

// AHB peripherals
#define LPC_CRC_BASE            (LPC_AHB_BASE + 0x00000)
#define LPC_SCT_BASE            (LPC_AHB_BASE + 0x04000)

// GPIO interrupts
#define LPC_PININT_BASE         (LPC_GPIO_BASE + 0x4000)




//-------------------------------------------------------------------------
// Peripheral declarations
//-------------------------------------------------------------------------
// ROM API
#define LPC_ROM_API             (*(LPC_ROM_API_T    * *) LPC_ROM_DRIVER_BASE)
#define LPC_ROM_PWR_API          ((LPC_ROM_PWR_API_T  *)(LPC_ROM_API->PWR_BASE_PTR))
#define LPC_ROM_I2C_API          ((LPC_ROM_I2C_API_T  *)(LPC_ROM_API->I2C_BASE_PTR))
#define LPC_ROM_UART_API         ((LPC_ROM_UART_API_T *)(LPC_ROM_API->UART_BASE_PTR))

// IAP entry function pointer
static const LPC_ROM_IAP_ENTRY_T iap_entry = (LPC_ROM_IAP_ENTRY_T)(LPC_ROM_IAP_BASE);

// APB0 peripherals
#define LPC_WWDT                ((LPC_WWDT_T   *) LPC_WWDT_BASE)
#define LPC_MRT                 ((LPC_MRT_T    *) LPC_MRT_BASE)
#define LPC_WKT                 ((LPC_WKT_T    *) LPC_WKT_BASE)
#define LPC_SWM                 ((LPC_SWM_T    *) LPC_SWM_BASE)
#define LPC_PMU                 ((LPC_PMU_T    *) LPC_PMU_BASE)
#define LPC_ACMP                ((LPC_ACMP_T   *) LPC_ACMP_BASE)
#define LPC_FMC                 ((LPC_FMC_T    *) LPC_FMC_BASE)
#define LPC_IOCON               ((LPC_IOCON_T  *) LPC_IOCON_BASE)
#define LPC_SYSCON              ((LPC_SYSCON_T *) LPC_SYSCON_BASE)
#define LPC_I2C                 ((LPC_I2C_T    *) LPC_I2C_BASE)
#define LPC_SPI0                ((LPC_SPI_T    *) LPC_SPI0_BASE)
#define LPC_SPI1                ((LPC_SPI_T    *) LPC_SPI1_BASE)
#define LPC_USART0              ((LPC_USART_T  *) LPC_USART0_BASE)
#define LPC_USART1              ((LPC_USART_T  *) LPC_USART1_BASE)
#define LPC_USART2              ((LPC_USART_T  *) LPC_USART2_BASE)

// AHB peripherals
#define LPC_CRC                 ((LPC_CRC_T    *) LPC_CRC_BASE)
#define LPC_SCT                 ((LPC_SCT_T    *) LPC_SCT_BASE)

// GPIO peripheral and interrupts
#define LPC_GPIO                ((LPC_GPIO_T   *) LPC_GPIO_BASE)
#define LPC_PININT              ((LPC_PININT_T *) LPC_PININT_BASE)




#ifdef __cplusplus
} // extern "C"
#endif

#endif // __XARMLIB_TARGETS_LPC81X_CMSIS_HPP
