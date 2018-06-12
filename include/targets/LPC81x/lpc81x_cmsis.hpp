// ----------------------------------------------------------------------------
// @file    lpc81x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for NXP LPC81x MCUs.
// @date    12 June 2018
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
    Reset_IRQn                    = -15,    // 1 Reset Vector, invoked on Power up and warm reset
    NonMaskableInt_IRQn           = -14,    // 2 Non Maskable Interrupt
    HardFault_IRQn                = -13,    // 3 Cortex-M0 Hard Fault Interrupt
    SVCall_IRQn                   = -5,     // 11 Cortex-M0 SV Call Interrupt
    PendSV_IRQn                   = -2,     // 14 Cortex-M0 Pend SV Interrupt
    SysTick_IRQn                  = -1,     // 15 Cortex-M0 System Tick Interrupt

    // LPC8xx Specific Interrupt Numbers
    SPI0_IRQn                     = 0,      // SPI0
    SPI1_IRQn                     = 1,      // SPI1
    Reserved0_IRQn                = 2,      // Reserved Interrupt
    UART0_IRQn                    = 3,      // USART0
    UART1_IRQn                    = 4,      // USART1
    UART2_IRQn                    = 5,      // USART2
    Reserved1_IRQn                = 6,      // Reserved Interrupt
    Reserved2_IRQn                = 7,      // Reserved Interrupt
    I2C_IRQn                      = 8,      // I2C0
    SCT_IRQn                      = 9,      // SCT
    MRT_IRQn                      = 10,     // MRT
    CMP_IRQn                      = 11,     // CMP
    WDT_IRQn                      = 12,     // WDT
    BOD_IRQn                      = 13,     // BOD
    Reserved3_IRQn                = 14,     // Reserved Interrupt
    WKT_IRQn                      = 15,     // WKT Interrupt
    Reserved4_IRQn                = 16,     // Reserved Interrupt
    Reserved5_IRQn                = 17,     // Reserved Interrupt
    Reserved6_IRQn                = 18,     // Reserved Interrupt
    Reserved7_IRQn                = 19,     // Reserved Interrupt
    Reserved8_IRQn                = 20,     // Reserved Interrupt
    Reserved9_IRQn                = 21,     // Reserved Interrupt
    Reserved10_IRQn               = 22,     // Reserved Interrupt
    Reserved11_IRQn               = 23,     // Reserved Interrupt
    PININT0_IRQn                  = 24,     // External Interrupt 0
    PININT1_IRQn                  = 25,     // External Interrupt 1
    PININT2_IRQn                  = 26,     // External Interrupt 2
    PININT3_IRQn                  = 27,     // External Interrupt 3
    PININT4_IRQn                  = 28,     // External Interrupt 4
    PININT5_IRQn                  = 29,     // External Interrupt 5
    PININT6_IRQn                  = 30,     // External Interrupt 6
    PININT7_IRQn                  = 31,     // External Interrupt 7
} IRQn_Type;




// ----------------------------------------------------------------------------
// Processor and Core Peripheral Section
// ----------------------------------------------------------------------------

// Configuration of the Cortex-M0+ Processor and Core Peripherals
#define __CM0PLUS_REV             0x0001    // Cortex-M0+ Core Revision
#define __MPU_PRESENT             0         // MPU present or not
#define __VTOR_PRESENT            1         // VTOR is present in this implementation
#define __NVIC_PRIO_BITS          2         // Number of Bits used for Priority Levels
#define __Vendor_SysTickConfig    0         // Set to 1 if different SysTick Config is used




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
    __IO uint32_t SYSMEMREMAP;              // (offset: 0x000) System memory remap (R/W)
    __IO uint32_t PRESETCTRL;               // (offset: 0x004) Peripheral reset control (R/W)
    __IO uint32_t SYSPLLCTRL;               // (offset: 0x008) System PLL control (R/W)
    __IO uint32_t SYSPLLSTAT;               // (offset: 0x00C) System PLL status (R/W )
         uint32_t RESERVED0[4];
    __IO uint32_t SYSOSCCTRL;               // (offset: 0x020) System oscillator control (R/W)
    __IO uint32_t WDTOSCCTRL;               // (offset: 0x024) Watchdog oscillator control (R/W)
    __IO uint32_t IRCCTRL;                  // (offset: 0x028) IRC Control Register (Available only in LPC82X)
         uint32_t RESERVED1[1];
    __IO uint32_t SYSRSTSTAT;               // (offset: 0x030) System reset status Register (R/W )
         uint32_t RESERVED2[3];
    __IO uint32_t SYSPLLCLKSEL;             // (offset: 0x040) System PLL clock source select (R/W)
    __IO uint32_t SYSPLLCLKUEN;             // (offset: 0x044) System PLL clock source update enable (R/W)
         uint32_t RESERVED3[10];
    __IO uint32_t MAINCLKSEL;               // (offset: 0x070) Main clock source select (R/W)
    __IO uint32_t MAINCLKUEN;               // (offset: 0x074) Main clock source update enable (R/W)
    __IO uint32_t SYSAHBCLKDIV;             // (offset: 0x078) System AHB clock divider (R/W)
         uint32_t RESERVED4[1];
    __IO uint32_t SYSAHBCLKCTRL;            // (offset: 0x080) System AHB clock control (R/W)
         uint32_t RESERVED5[4];
    __IO uint32_t UARTCLKDIV;               // (offset: 0x094) UART clock divider (R/W)
         uint32_t RESERVED6[18];
    __IO uint32_t CLKOUTSEL;                // (offset: 0x0E0) CLKOUT clock source select (R/W)
    __IO uint32_t CLKOUTUEN;                // (offset: 0x0E4) CLKOUT clock source update enable (R/W)
    __IO uint32_t CLKOUTDIV;                // (offset: 0x0E8) CLKOUT clock divider (R/W)
         uint32_t RESERVED7;
    __IO uint32_t UARTFRGDIV;               // (offset: 0x0F0) UART fractional divider SUB(R/W)
    __IO uint32_t UARTFRGMULT;              // (offset: 0x0F4) UART fractional divider ADD(R/W)
         uint32_t RESERVED8[1];
    __IO uint32_t EXTTRACECMD;              // (offset: 0x0FC) External trace buffer command register
    __IO uint32_t PIOPORCAP0;               // (offset: 0x100) POR captured PIO status 0 (R/ )
         uint32_t RESERVED9[12];
    __IO uint32_t IOCONCLKDIV[7];           // (offset: 0x134) Peripheral clock x to the IOCON block for programmable glitch filter
    __IO uint32_t BODCTRL;                  // (offset: 0x150) BOD control (R/W)
    __IO uint32_t SYSTCKCAL;                // (offset: 0x154) System tick counter calibration (R/W)
         uint32_t RESERVED10[6];
    __IO uint32_t IRQLATENCY;               // (offset: 0x170) IRQ delay
    __IO uint32_t NMISRC;                   // (offset: 0x174) NMI Source Control
    __IO uint32_t PINTSEL[8];               // (offset: 0x178) GPIO Pin Interrupt Select register 0
         uint32_t RESERVED11[27];
    __IO uint32_t STARTERP0;                // (offset: 0x204) Start logic signal enable Register 0 (R/W)
         uint32_t RESERVED12[3];
    __IO uint32_t STARTERP1;                // (offset: 0x214) Start logic signal enable Register 0 (R/W)
         uint32_t RESERVED13[6];
    __IO uint32_t PDSLEEPCFG;               // (offset: 0x230) Power-down states in Deep-sleep mode (R/W)
    __IO uint32_t PDAWAKECFG;               // (offset: 0x234) Power-down states after wake-up (R/W)
    __IO uint32_t PDRUNCFG;                 // (offset: 0x238) Power-down configuration Register (R/W)
         uint32_t RESERVED14[111];
    __I  uint32_t DEVICEID;                 // (offset: 0x3F8) Device ID (R/ )
} LPC_SYSCON_T;




// ------------ I/O Configuration (IOCON) -------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PIO[19];
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
    __IO uint32_t PCON;                     // (offset: 0x000) Power control Register (R/W)
    __IO uint32_t GPREG0;                   // (offset: 0x004) General purpose Register 0 (R/W)
    __IO uint32_t GPREG1;                   // (offset: 0x008) General purpose Register 1 (R/W)
    __IO uint32_t GPREG2;                   // (offset: 0x00C) General purpose Register 2 (R/W)
    __IO uint32_t GPREG3;                   // (offset: 0x010) General purpose Register 3 (R/W)
    __IO uint32_t DPDCTRL;                  // (offset: 0x014) Deep power-down control register (R/W)
} LPC_PMU_T;




// ------------ Switch Matrix (SWM) -------------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PINASSIGN[9];         // Pin Assign register array
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
        };
    };
         uint32_t RESERVED0[103];           // RESERVED
    __IO uint32_t PINENABLE0;               // (offset: 0x1C0) Pin Enable register
} LPC_SWM_T;




// ------------ General Purpose I/O (GPIO) ------------------------------------
typedef struct
{
    __IO uint8_t  B[128][32];               // (offset: 0x0000) Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31
    __IO uint32_t W[32][32];                // (offset: 0x1000) Word pin registers port 0 to n
    __IO uint32_t DIR[32];                  // (offset: 0x2000) Direction registers port n
    __IO uint32_t MASK[32];                 // (offset: 0x2080) Mask register port n
    __IO uint32_t PIN[32];                  // (offset: 0x2100) Port pin register port n
    __IO uint32_t MPIN[32];                 // (offset: 0x2180) Masked port register port n
    __IO uint32_t SET[32];                  // (offset: 0x2200) Write: Set register for port n Read: output bits for port n
    __O  uint32_t CLR[32];                  // (offset: 0x2280) Clear port n
    __O  uint32_t NOT[32];                  // (offset: 0x2300) Toggle port n
    __O  uint32_t DIRSET[32];               // (offset: 0x2380) Set Direction
    __O  uint32_t DIRCLR[32];               // (offset: 0x2400) Clear Direction
    __O  uint32_t DIRNOT[32];               // (offset: 0x2480) Toggle Direction
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
    __IO uint32_t MODE;                     // CRC Mode Register
    __IO uint32_t SEED;                     // CRC SEED Register
    union
    {
        __I uint32_t SUM;                   // CRC Checksum Register.
        __O uint32_t WRDATA32;              // CRC Data Register: write size 32-bit
        __O uint16_t WRDATA16;              // CRC Data Register: write size 16-bit
        __O uint8_t  WRDATA8;               // CRC Data Register: write size 8-bit
    };
} LPC_CRC_T;




// ------------ Analog Comparator (CMP) ---------------------------------------
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
    __IO uint32_t INTVAL;                   // Timer interval register
    __O  uint32_t TIMER;                    // Timer register
    __IO uint32_t CTRL;                     // Timer control register
    __IO uint32_t STAT;                     // Timer status register
} LPC_MRT_CHANNEL_T;

typedef struct
{
    LPC_MRT_CHANNEL_T CHANNEL[4];           // (offset: 0x00-0x3C)

         uint32_t     RESERVED[45];         // (offset: 0x40-0xF0) RESERVED
    __O  uint32_t     IDLE_CH;              // (offset: 0xF4)
    __IO uint32_t     IRQ_FLAG;             // (offset: 0xF8)
} LPC_MRT_T;




// ------ Universal Synchronous Asynchronous Receiver Transmitter (USART) -----
typedef struct
{
    __IO uint32_t  CFG;                     // Configuration register
    __IO uint32_t  CTRL;                    // Control register
    __IO uint32_t  STAT;                    // Status register
    __IO uint32_t  INTENSET;                // Interrupt Enable read and set register
    __O  uint32_t  INTENCLR;                // Interrupt Enable clear register
    __I  uint32_t  RXDAT;                   // Receive Data register
    __I  uint32_t  RXDATSTAT;               // Receive Data with status register
    __IO uint32_t  TXDAT;                   // Transmit data register
    __IO uint32_t  BRG;                     // Baud Rate Generator register
    __IO uint32_t  INTSTAT;                 // Interrupt status register
    __IO uint32_t  OSR;                     // Oversampling Selection register
    __IO uint32_t  ADDR;                    // Address register for automatic address matching
} LPC_USART_T;




// ------------ Serial Peripheral Interface (SPI) -----------------------------
typedef struct
{
    __IO uint32_t  CFG;                     // Configuration register
    __IO uint32_t  DLY;                     // Delay register
    __IO uint32_t  STAT;                    // Status register
    __IO uint32_t  INTENSET;                // Interrupt Enable Set register
    __O  uint32_t  INTENCLR;                // Interrupt Enable Clear register
    __I  uint32_t  RXDAT;                   // Receive Data register
    __IO uint32_t  TXDATCTL;                // Transmit Data with Control register
    __IO uint32_t  TXDAT;                   // Transmit Data register
    __IO uint32_t  TXCTL;                   // Transmit Control register
    __IO uint32_t  DIV;                     // clock Divider register
    __I  uint32_t  INTSTAT;                 // Interrupt Status register
} LPC_SPI_T;




// ------------ Inter-Integrated Circuit (I2C) --------------------------------
typedef struct
{
    __IO uint32_t CFG;                      // Configuration Register common for Master, Slave and Monitor
    __IO uint32_t STAT;                     // Status Register common for Master, Slave and Monitor
    __IO uint32_t INTENSET;                 // Interrupt Enable Set Register common for Master, Slave and Monitor
    __O  uint32_t INTENCLR;                 // Interrupt Enable Clear Register common for Master, Slave and Monitor
    __IO uint32_t TIMEOUT;                  // Timeout value Register
    __IO uint32_t CLKDIV;                   // Clock Divider Register
    __I  uint32_t INTSTAT;                  // Interrupt Status Register
    __I  uint32_t RESERVED0;
    __IO uint32_t MSTCTL;                   // Master Control Register
    __IO uint32_t MSTTIME;                  // Master Time Register for SCL
    __IO uint32_t MSTDAT;                   // Master Data Register
    __I  uint32_t RESERVED1[5];
    __IO uint32_t SLVCTL;                   // Slave Control Register
    __IO uint32_t SLVDAT;                   // Slave Data Register
    __IO uint32_t SLVADR[4];                // Slave Address Registers
    __IO uint32_t SLVQUAL0;                 // Slave Address Qualifier 0 Register
    __I  uint32_t RESERVED2[9];
    __I  uint32_t MONRXDAT;                 // Monitor Data Register
} LPC_I2C_T;




// ------------ State Configurable Timer (SCT) --------------------------------
#define SCT_NUM_EVENTS      (8)             // Number of events
#define SCT_NUM_REGISTERS   (8)             // Number of match/compare registers
#define SCT_NUM_OUTPUTS     (6)             // Number of outputs

typedef struct
{
    __IO  uint32_t CONFIG;                  // configuration Register (offset (0x000)
    union
    {
        __IO uint32_t CTRL;                 // control Register
        struct
        {
            __IO uint16_t CTRL_L;           // low control register
            __IO uint16_t CTRL_H;           // high control register
        };
    };
    union
    {
        __IO uint32_t LIMIT;                // limit Register
        struct
        {
            __IO uint16_t LIMIT_L;          // limit register for counter L
            __IO uint16_t LIMIT_H;          // limit register for counter H
        };
    };

    union
    {
        __IO uint32_t HALT;                 // halt Register
        struct
        {
            __IO uint16_t HALT_L;           // halt register for counter L
            __IO uint16_t HALT_H;           // halt register for counter H
        };
    };

    union
    {
        __IO uint32_t STOP;                 // stop Register
        struct
        {
            __IO uint16_t STOP_L;           // stop register for counter L
            __IO uint16_t STOP_H;           // stop register for counter H
        };
    };

    union
    {
        __IO uint32_t START;                // start Register
        struct
        {
            __IO uint16_t START_L;          // start register for counter L
            __IO uint16_t START_H;          // start register for counter H
        };
    };

    uint32_t RESERVED0[10];                 // 0x018 - 0x03C reserved

    union
    {
        __IO uint32_t COUNT;                // counter register (offset 0x040)
        struct
        {
            __IO uint16_t COUNT_L;          // counter register for counter L
            __IO uint16_t COUNT_H;          // counter register for counter H
        };
    };

    union
    {
        __IO uint32_t STATE;                // State register
        struct
        {
            __IO uint16_t STATE_L;          // state register for counter L
            __IO uint16_t STATE_H;          // state register for counter H
        };
    };

    __I  uint32_t INPUT;                    // input register
    union
    {
        __IO uint32_t REGMODE;              // RegMode register
        struct
        {
            __IO uint16_t REGMODE_L;        // match - capture registers mode register L
            __IO uint16_t REGMODE_H;        // match - capture registers mode register H
        };
    };

    __IO uint32_t OUTPUT;                   // output register
    __IO uint32_t OUTPUTDIRCTRL;            // output counter direction Control Register
    __IO uint32_t RES;                      // conflict resolution register
    __IO uint32_t DMAREQ0;                  // DMA0 Request Register
    __IO uint32_t DMAREQ1;                  // DMA1 Request Register

    uint32_t RESERVED1[35];                 // 0x064 - 0x0EC reserved

    __IO uint32_t EVEN;                     // event enable register (offset 0x0F0)*/
    __IO uint32_t EVFLAG;                   // event flag register
    __IO uint32_t CONEN;                    // conflict enable register
    __IO uint32_t CONFLAG;                  // conflict flag register
    union
    {
        __IO union
        {
            uint32_t U;                     //  MATCH[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 //  MATCH[i].L  Access to L value
                uint16_t H;                 //  MATCH[i].H  Access to H value
            };
        } MATCH[SCT_NUM_REGISTERS];

        __I union
        {
            uint32_t U;                     //  CAP[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 //  CAP[i].L  Access to L value
                uint16_t H;                 //  CAP[i].H  Access to H value
            };
        } CAP[SCT_NUM_REGISTERS];
    };

    uint32_t RESERVED2[56];                 // 0x120 - 0x1FC reserved

    union
    {
        __IO union
        {
            uint32_t U;                     //  MATCHREL[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 //  MATCHREL[i].L  Access to L value
                uint16_t H;                 //  MATCHREL[i].H  Access to H value
            };
        } MATCHREL[SCT_NUM_REGISTERS];

        __IO union
        {
            uint32_t U;                     //  CAPCTRL[i].U  Unified 32-bit register
            struct
            {
                uint16_t L;                 //  CAPCTRL[i].L  Access to L value
                uint16_t H;                 //  CAPCTRL[i].H  Access to H value
            };
        } CAPCTRL[SCT_NUM_REGISTERS];
    };

    uint32_t RESERVED3[56];                 // 0x220 - 0x2FC reserved

    __IO struct
    {
        uint32_t STATE;                     // Event State Register
        uint32_t CTRL;                      // Event Control Register
    } EV[SCT_NUM_EVENTS];

    uint32_t RESERVED4[112];                // 0x340 - 0x4FC reserved

    __IO struct
    {
        uint32_t SET;                       // Output n Set Register
        uint32_t CLR;                       // Output n Clear Register
    } OUT[SCT_NUM_OUTPUTS];
} LPC_SCT_T;




// ------------ Windowed Watchdog Timer (WWDT) --------------------------------
typedef struct
{
    __IO uint32_t MOD;                      // (offset: 0x000) Watchdog mode register
    __IO uint32_t TC;                       // (offset: 0x004) Watchdog timer constant register
    __O  uint32_t FEED;                     // (offset: 0x008) Watchdog feed sequence register
    __I  uint32_t TV;                       // (offset: 0x00C) Watchdog timer value register
         uint32_t RESERVED;                 // (offset: 0x010) RESERVED
    __IO uint32_t WARNINT;                  // (offset: 0x014) Watchdog timer warning interrupt register
    __IO uint32_t WINDOW;                   // (offset: 0x018) Watchdog timer window value register
} LPC_WWDT_T;




// ------------ ROM API -------------------------------------------------------

// Error code returned by Boot ROM drivers/library functions enumeration
// NOTE: Error codes are a 32-bit value with:
//       - The 16 MSB contains the peripheral code number
//       - The 16 LSB contains an error code number associated to that peripheral
typedef enum
{
  /*0x00000000*/ LPC_OK = 0,                            // enum value returned on Successful completion
  /*0x00000001*/ LPC_ERROR,                             // enum value returned on general error (I2C)

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

// ROM Driver table
#define LPC_ROM_DRIVER_BASE     (0x1FFF1FF8UL)
#define LPC_ROM_IAP_BASE        (0X1FFF1FF1UL)

// APB peripherals
#define LPC_WWDT_BASE           (0x40000000UL)
#define LPC_MRT_BASE            (0x40004000UL)
#define LPC_WKT_BASE            (0x40008000UL)
#define LPC_SWM_BASE            (0x4000C000UL)
#define LPC_PMU_BASE            (0x40020000UL)
#define LPC_CMP_BASE            (0x40024000UL)

#define LPC_FMC_BASE            (0x40040000UL)
#define LPC_IOCON_BASE          (0x40044000UL)
#define LPC_SYSCON_BASE         (0x40048000UL)
#define LPC_I2C_BASE            (0x40050000UL)
#define LPC_SPI0_BASE           (0x40058000UL)
#define LPC_SPI1_BASE           (0x4005C000UL)
#define LPC_USART0_BASE         (0x40064000UL)
#define LPC_USART1_BASE         (0x40068000UL)
#define LPC_USART2_BASE         (0x4006C000UL)

// AHB peripherals
#define LPC_CRC_BASE            (0x50000000UL)
#define LPC_SCT_BASE            (0x50004000UL)

#define LPC_GPIO_BASE           (0xA0000000UL)
#define LPC_PIN_INT_BASE        (0xA0004000UL)




//-------------------------------------------------------------------------
// Peripheral declarations
//-------------------------------------------------------------------------
// ROM API
#define LPC_ROM_API             (*(LPC_ROM_API_T    * *) LPC_ROM_DRIVER_BASE)
#define LPC_ROM_PWR_API          ((LPC_ROM_PWR_API_T  *)(LPC_ROM_API->PWR_BASE_PTR))
#define LPC_ROM_I2C_API          ((LPC_ROM_I2C_API_T  *)(LPC_ROM_API->I2C_BASE_PTR))
#define LPC_ROM_UART_API         ((LPC_ROM_UART_API_T *)(LPC_ROM_API->UART_BASE_PTR))

// IAP entry function pointer
const LPC_ROM_IAP_ENTRY_T iap_entry = (LPC_ROM_IAP_ENTRY_T)(LPC_ROM_IAP_BASE);

#define LPC_WWDT                ((LPC_WWDT_T   *) LPC_WWDT_BASE)
#define LPC_SPI0                ((LPC_SPI_T    *) LPC_SPI0_BASE)
#define LPC_SPI1                ((LPC_SPI_T    *) LPC_SPI1_BASE)
#define LPC_USART0              ((LPC_USART_T  *) LPC_USART0_BASE)
#define LPC_USART1              ((LPC_USART_T  *) LPC_USART1_BASE)
#define LPC_USART2              ((LPC_USART_T  *) LPC_USART2_BASE)
#define LPC_WKT                 ((LPC_WKT_T    *) LPC_WKT_BASE)
#define LPC_PMU                 ((LPC_PMU_T    *) LPC_PMU_BASE)
#define LPC_CRC                 ((LPC_CRC_T    *) LPC_CRC_BASE)
#define LPC_SCT                 ((LPC_SCT_T    *) LPC_SCT_BASE)
#define LPC_GPIO                ((LPC_GPIO_T   *) LPC_GPIO_BASE)
#define LPC_PIN_INT             ((LPC_PIN_INT_T*) LPC_PIN_INT_BASE)
#define LPC_IOCON               ((LPC_IOCON_T  *) LPC_IOCON_BASE)
#define LPC_SWM                 ((LPC_SWM_T    *) LPC_SWM_BASE)
#define LPC_SYSCON              ((LPC_SYSCON_T *) LPC_SYSCON_BASE)
#define LPC_CMP                 ((LPC_CMP_T    *) LPC_CMP_BASE)
#define LPC_FMC                 ((LPC_FMC_T    *) LPC_FMC_BASE)
#define LPC_MRT                 ((LPC_MRT_T    *) LPC_MRT_BASE)
#define LPC_I2C                 ((LPC_I2C_T    *) LPC_I2C_BASE)




#ifdef __cplusplus
}
#endif

#endif // __XARMLIB_TARGETS_LPC81X_CMSIS_HPP
