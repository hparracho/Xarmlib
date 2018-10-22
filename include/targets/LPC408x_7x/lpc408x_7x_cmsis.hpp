// ----------------------------------------------------------------------------
// @file    lpc408x_7x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for NXP LPC408x_7x MCUs.
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

// ----------------------------------------------------------------------------
// This file is based on 2 sources:
// - on the code from Keil MDK Version 5
//   (Microcontroller Development Kit) for NXP LPC4088FET208
//   https://www.keil.com/dd2/nxp/lpc4088fet208/#/eula-container
//
// - on the code from LPCOpen v2.10 for LPCXpresso v7.0.2_102
//   (Embedded Artists LPC4088 board) provided by NXP.
//   https://www.nxp.com/downloads/en/libraries/lpcopen_2_10_lpcxpresso_ea_devkit_4088.zip
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_TARGETS_LPC408X_7X_CMSIS_HPP
#define __XARMLIB_TARGETS_LPC408X_7X_CMSIS_HPP

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
    // Cortex-M4 Processor Exceptions Numbers
    Reset_IRQn                  = -15,      // 1 Reset Vector, invoked on power up and warm reset
    NonMaskableInt_IRQn         = -14,      // 2 Non Maskable Interrupt, cannot be stopped or preempted
    HardFault_IRQn              = -13,      // 3 Hard Fault, all classes of fault
    MemoryManagement_IRQn       = -12,      // 4 Memory Management, MPU mismatch, including Access Violation and No Match
    BusFault_IRQn               = -11,      // 5 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault
    UsageFault_IRQn             = -10,      // 6 Usage Fault, i.e. Undef Instruction, Illegal State Transition
    SVCall_IRQn                 = -5,       // 11 System Service Call via SVC instruction
    DebugMonitor_IRQn           = -4,       // 12 CDebug Monitor
    PendSV_IRQn                 = -2,       // 14 Pendable request for system service
    SysTick_IRQn                = -1,       // 15 System Tick Interrupt

    // LPC408x_7x Specific Interrupt Numbers
    WDT_IRQn                    = 0,        // Watchdog Interrupt
    TIMER0_IRQn                 = 1,        // Timer 0 Interrupt
    TIMER1_IRQn                 = 2,        // Timer 1 Interrupt
    TIMER2_IRQn                 = 3,        // Timer 2 Interrupt
    TIMER3_IRQn                 = 4,        // Timer 3 Interrupt
    UART0_IRQn                  = 5,        // UART0 Interrupt
    UART1_IRQn                  = 6,        // UART1 Interrupt
    UART2_IRQn                  = 7,        // UART2 Interrupt
    UART3_IRQn                  = 8,        // UART3 Interrupt
    PWM1_IRQn                   = 9,        // PWM1 Interrupt
    I2C0_IRQn                   = 10,       // I2C0 Interrupt
    I2C1_IRQn                   = 11,       // I2C1 Interrupt
    I2C2_IRQn                   = 12,       // I2C2 Interrupt
    Reserved_IRQn               = 13,       // Reserved interrupt
    SSP0_IRQn                   = 14,       // SSP0 Interrupt
    SSP1_IRQn                   = 15,       // SSP1 Interrupt
    PLL0_IRQn                   = 16,       // PLL0 (Main PLL) Interrupt
    RTC_IRQn                    = 17,       // RTC and Event Monitor/Recorder Interrupt
    EINT0_IRQn                  = 18,       // External Interrupt 0 Interrupt
    EINT1_IRQn                  = 19,       // External Interrupt 1 Interrupt
    EINT2_IRQn                  = 20,       // External Interrupt 2 Interrupt
    EINT3_IRQn                  = 21,       // External Interrupt 3 Interrupt
    ADC_IRQn                    = 22,       // ADC Interrupt
    BOD_IRQn                    = 23,       // BOD Interrupt
    USB_IRQn                    = 24,       // USB Interrupt
    CAN_IRQn                    = 25,       // CAN Interrupt
    DMA_IRQn                    = 26,       // DMA Controller Interrupt
    I2S_IRQn                    = 27,       // I2S Interrupt
    ENET_IRQn                   = 28,       // Ethernet Interrupt
    SDC_IRQn                    = 29,       // SD Card Interface Interrupt
    MCPWM_IRQn                  = 30,       // Motor Control PWM Interrupt
    QEI_IRQn                    = 31,       // Quadrature Encoder Interrupt
    PLL1_IRQn                   = 32,       // PLL1 (USB and/or SPIFI PLL) Interrupt
    USBACT_IRQn                 = 33,       // USB Activity interrupt
    CANACT_IRQn                 = 34,       // CAN Activity interrupt
    UART4_IRQn                  = 35,       // UART4 Interrupt
    SSP2_IRQn                   = 36,       // SSP2 Interrupt
    LCD_IRQn                    = 37,       // LCD Controller Interrupt
    GPIO_IRQn                   = 38,       // GPIO Interrupt
    PWM0_IRQn                   = 39,       // PWM0 Interrupt
    EEPROM_IRQn                 = 40,       // EEPROM Interrupt
} IRQn_Type;




// ----------------------------------------------------------------------------
// Processor and Core Peripheral Section
// ----------------------------------------------------------------------------

// Configuration of the Cortex-M4 Processor and Core Peripherals
#define __CM4_REV               0x0001      // Cortex-M4 Core Revision
#define __MPU_PRESENT           1           // MPU present or not
#define __NVIC_PRIO_BITS        5           // Number of Bits used for Priority Levels
#define __Vendor_SysTickConfig  0           // Set to 1 if different SysTick Config is used
#define __FPU_PRESENT           1           // FPU present or not




extern uint32_t SystemCoreClock;            // CMSIS system core clock variable definition

// Update system core clock frequency
// NOTE: This function is called in startup functions but should be
//       called every time the system has a clock frequency change.
void SystemCoreClockUpdate(void);




#include "core_cm4.h"                       // Cortex-M4 processor and core peripherals




// ----------------------------------------------------------------------------
// Device Specific Peripheral Registers structures
// ----------------------------------------------------------------------------

// ------------ General Purpose DMA controller (GPDMA) ------------------------
#define GPDMA_NUM_CHANNELS      8

typedef struct
{
    __IO uint32_t SRCADDR;                  // DMA Channel Source Address Register
    __IO uint32_t DESTADDR;                 // DMA Channel Destination Address Register
    __IO uint32_t LLI;                      // DMA Channel Linked List Item Register
    __IO uint32_t CONTROL;                  // DMA Channel Control Register
    __IO uint32_t CONFIG;                   // DMA Channel Configuration Register
         uint32_t RESERVED[3];
} LPC_GPDMA_CHANNEL_T;

typedef struct
{
    __I  uint32_t INTSTAT;                  // DMA Interrupt Status Register
    __I  uint32_t INTTCSTAT;                // DMA Interrupt Terminal Count Request Status Register
    __O  uint32_t INTTCCLEAR;               // DMA Interrupt Terminal Count Request Clear Register
    __I  uint32_t INTERRSTAT;               // DMA Interrupt Error Status Register
    __O  uint32_t INTERRCLR;                // DMA Interrupt Error Clear Register
    __I  uint32_t RAWINTTCSTAT;             // DMA Raw Interrupt Terminal Count Status Register
    __I  uint32_t RAWINTERRSTAT;            // DMA Raw Error Interrupt Status Register
    __I  uint32_t ENBLDCHNS;                // DMA Enabled Channel Register
    __IO uint32_t SOFTBREQ;                 // DMA Software Burst Request Register
    __IO uint32_t SOFTSREQ;                 // DMA Software Single Request Register
    __IO uint32_t SOFTLBREQ;                // DMA Software Last Burst Request Register
    __IO uint32_t SOFTLSREQ;                // DMA Software Last Single Request Register
    __IO uint32_t CONFIG;                   // DMA Configuration Register
    __IO uint32_t SYNC;                     // DMA Synchronization Register
         uint32_t RESERVED[50];

    LPC_GPDMA_CHANNEL_T CHANNEL[GPDMA_NUM_CHANNELS];
} LPC_GPDMA_T;




// ------------ System and clock control (SYSCON) -----------------------------
typedef struct
{
    __IO uint32_t FLASHCFG;                 // (offset: 0x000) Flash Accelerator Configuration Register
         uint32_t RESERVED0[31];
    __IO uint32_t PLL0CON;                  // (offset: 0x080) PLL0 Control Register
    __IO uint32_t PLL0CFG;                  // (offset: 0x084) PLL0 Configuration Register
    __I  uint32_t PLL0STAT;                 // (offset: 0x088) PLL0 Status Register
    __O  uint32_t PLL0FEED;                 // (offset: 0x08C) PLL0 Feed Register
         uint32_t RESERVED1[4];
    __IO uint32_t PLL1CON;                  // (offset: 0x0A0) PLL1 Control Register
    __IO uint32_t PLL1CFG;                  // (offset: 0x0A4) PLL1 Configuration Register
    __I  uint32_t PLL1STAT;                 // (offset: 0x0A8) PLL1 Status Register
    __O  uint32_t PLL1FEED;                 // (offset: 0x0AC) PLL1 Feed Register
         uint32_t RESERVED2[4];
    __IO uint32_t PCON;                     // (offset: 0x0C0) Power Control Register
    __IO uint32_t PCONP;                    // (offset: 0x0C4) Power Control for Peripherals Register
    __IO uint32_t PCONP1;                   // (offset: 0x0C8) Power Control for Peripherals Register
         uint32_t RESERVED3[13];
    __IO uint32_t EMCCLKSEL;                // (offset: 0x100) External Memory Controller Clock Selection Register
    __IO uint32_t CCLKSEL;                  // (offset: 0x104) CPU Clock Selection Register
    __IO uint32_t USBCLKSEL;                // (offset: 0x108) USB Clock Selection Register
    __IO uint32_t CLKSRCSEL;                // (offset: 0x10C) Clock Source Select Register
    __IO uint32_t CANSLEEPCLR;              // (offset: 0x110) CAN Sleep Clear Register
    __IO uint32_t CANWAKEFLAGS;             // (offset: 0x114) CAN Wake-up Flags Register
         uint32_t RESERVED4[10];
    __IO uint32_t EXTINT;                   // (offset: 0x140) External Interrupt Flag Register
         uint32_t RESERVED5[1];
    __IO uint32_t EXTMODE;                  // (offset: 0x148) External Interrupt Mode Register
    __IO uint32_t EXTPOLAR;                 // (offset: 0x14C) External Interrupt Polarity Register
         uint32_t RESERVED6[12];
    __IO uint32_t RSID;                     // (offset: 0x180) Reset Source Identification Register
         uint32_t RESERVED7[7];
    __IO uint32_t SCS;                      // (offset: 0x1A0) System Controls and Status Register
    __IO uint32_t IRCTRIM;                  // (offset: 0x1A4) Clock Dividers
    __IO uint32_t PCLKSEL;                  // (offset: 0x1A8) Peripheral Clock Selection Register
         uint32_t RESERVED8;
    __IO uint32_t PBOOST;                   // (offset: 0x1B0) Power Boost control register
    __IO uint32_t SPIFICLKSEL;
    __IO uint32_t LCD_CFG;                  // (offset: 0x1B8) LCD Configuration and clocking control Register
         uint32_t RESERVED9[1];
    __IO uint32_t USBINTST;                 // (offset: 0x1C0) USB Interrupt Status Register
    __IO uint32_t DMAREQSEL;                // (offset: 0x1C4) DMA Request Select Register
    __IO uint32_t CLKOUTCFG;                // (offset: 0x1C8) Clock Output Configuration Register
    __IO uint32_t RSTCON0;                  // (offset: 0x1CC) RESET Control0 Register
    __IO uint32_t RSTCON1;                  // (offset: 0x1D0) RESET Control1 Register
         uint32_t RESERVED10[2];
    __IO uint32_t EMCDLYCTL;                // (offset: 0x1DC) SDRAM programmable delays
    __IO uint32_t EMCCAL;                   // (offset: 0x1E0) Calibration of programmable delays
} LPC_SYSCON_T;




// ------------ Ethernet (ENET) -----------------------------------------------
typedef struct
{
    __IO uint32_t MAC1;                     // MAC Configuration register 1
    __IO uint32_t MAC2;                     // MAC Configuration register 2
    __IO uint32_t IPGT;                     // Back-to-Back Inter-Packet-Gap register
    __IO uint32_t IPGR;                     // Non Back-to-Back Inter-Packet-Gap register
    __IO uint32_t CLRT;                     // Collision window / Retry register
    __IO uint32_t MAXF;                     // Maximum Frame register
    __IO uint32_t SUPP;                     // PHY Support register
    __IO uint32_t TEST;                     // Test register
    __IO uint32_t MCFG;                     // MII Mgmt Configuration register
    __IO uint32_t MCMD;                     // MII Mgmt Command register
    __IO uint32_t MADR;                     // MII Mgmt Address register
    __O  uint32_t MWTD;                     // MII Mgmt Write Data register
    __I  uint32_t MRDD;                     // MII Mgmt Read Data register
    __I  uint32_t MIND;                     // MII Mgmt Indicators register
         uint32_t RESERVED[2];
    __IO uint32_t SA[3];                    // Station Address registers
} LPC_ENET_MAC_T;

typedef struct
{
    __IO uint32_t DESCRIPTOR;               // Descriptor base address register
    __IO uint32_t STATUS;                   // Status base address register
    __IO uint32_t DESCRIPTORNUMBER;         // Number of descriptors register
    __IO uint32_t PRODUCEINDEX;             // Produce index register
    __IO uint32_t CONSUMEINDEX;             // Consume index register
} LPC_ENET_TRANSFER_INFO_T;

typedef struct
{
    __IO uint32_t COMMAND;                  // Command register
    __I  uint32_t STATUS;                   // Status register
    LPC_ENET_TRANSFER_INFO_T RX;            // Receive block registers
    LPC_ENET_TRANSFER_INFO_T TX;            // Transmit block registers
         uint32_t RESERVED0[10];
    __I  uint32_t TSV0;                     // Transmit status vector 0 register
    __I  uint32_t TSV1;                     // Transmit status vector 1 register
    __I  uint32_t RSV;                      // Receive status vector register
         uint32_t RESERVED1[3];
    __IO uint32_t FLOWCONTROLCOUNTER;       // Flow control counter register
    __I  uint32_t FLOWCONTROLSTATUS;        // Flow control status register
} LPC_ENET_CONTROL_T;

typedef struct
{
    __IO uint32_t CONTROL;                  // Receive filter control register
    __I  uint32_t WOLSTATUS;                // Receive filter WoL status register
    __O  uint32_t WOLCLEAR;                 // Receive filter WoL clear register
         uint32_t RESERVED;
    __IO uint32_t HASHFILTERL;              // Hash filter table LSBs register
    __IO uint32_t HASHFILTERH;              // Hash filter table MSBs register
} LPC_ENET_RXFILTER_T;

typedef struct{
    __I  uint32_t INTSTATUS;                // Interrupt status register
    __IO uint32_t INTENABLE;                // Interrupt enable register
    __O  uint32_t INTCLEAR;                 // Interrupt clear register
    __O  uint32_t INTSET;                   // Interrupt set register
         uint32_t RESERVED;
    __IO uint32_t POWERDOWN;                // Power-down register
} LPC_ENET_MODULE_CONTROL_T;

typedef struct
{
    LPC_ENET_MAC_T MAC;                     // MAC registers
         uint32_t RESERVED0[45];
    LPC_ENET_CONTROL_T CONTROL;             // Control registers
         uint32_t RESERVED1[34];
    LPC_ENET_RXFILTER_T RXFILTER;           // RxFilter registers
         uint32_t RESERVED2[882];
    LPC_ENET_MODULE_CONTROL_T MODULE_CONTROL;   // Module Control registers
} LPC_ENET_T;




// ------------ LCD controller (LCD) ------------------------------------------
typedef struct
{
    __IO uint32_t TIMH;                     // Horizontal Timing Control register
    __IO uint32_t TIMV;                     // Vertical Timing Control register
    __IO uint32_t POL;                      // Clock and Signal Polarity Control register
    __IO uint32_t LE;                       // Line End Control register
    __IO uint32_t UPBASE;                   // Upper Panel Frame Base Address register
    __IO uint32_t LPBASE;                   // Lower Panel Frame Base Address register
    __IO uint32_t CTRL;                     // LCD Control register
    __IO uint32_t INTMSK;                   // Interrupt Mask register
    __I  uint32_t INTRAW;                   // Raw Interrupt Status register
    __I  uint32_t INTSTAT;                  // Masked Interrupt Status register
    __O  uint32_t INTCLR;                   // Interrupt Clear register
    __I  uint32_t UPCURR;                   // Upper Panel Current Address Value register
    __I  uint32_t LPCURR;                   // Lower Panel Current Address Value register
         uint32_t RESERVED0[115];
    __IO uint16_t PAL[256];                 // 256x16-bit Color Palette registers
         uint32_t RESERVED1[256];
    __IO uint32_t CRSR_IMG[256];            // Cursor Image registers
    __IO uint32_t CRSR_CTRL;                // Cursor Control register
    __IO uint32_t CRSR_CFG;                 // Cursor Configuration register
    __IO uint32_t CRSR_PAL0;                // Cursor Palette register 0
    __IO uint32_t CRSR_PAL1;                // Cursor Palette register 1
    __IO uint32_t CRSR_XY;                  // Cursor XY Position register
    __IO uint32_t CRSR_CLIP;                // Cursor Clip Position register
         uint32_t RESERVED2[2];
    __IO uint32_t CRSR_INTMSK;              // Cursor Interrupt Mask register
    __O  uint32_t CRSR_INTCLR;              // Cursor Interrupt Clear register
    __I  uint32_t CRSR_INTRAW;              // Cursor Raw Interrupt Status register
    __I  uint32_t CRSR_INTSTAT;             // Cursor Masked Interrupt Status register
} LPC_LCD_T;




// ------------ USB controller (USB) ------------------------------------------
typedef struct
{
    // USB Host Registers
    __I  uint32_t Revision;
    __IO uint32_t Control;
    __IO uint32_t CommandStatus;
    __IO uint32_t InterruptStatus;
    __IO uint32_t InterruptEnable;
    __IO uint32_t InterruptDisable;
    __IO uint32_t HCCA;
    __I  uint32_t PeriodCurrentED;
    __IO uint32_t ControlHeadED;
    __IO uint32_t ControlCurrentED;
    __IO uint32_t BulkHeadED;
    __IO uint32_t BulkCurrentED;
    __I  uint32_t DoneHead;
    __IO uint32_t FmInterval;
    __I  uint32_t FmRemaining;
    __I  uint32_t FmNumber;
    __IO uint32_t PeriodicStart;
    __IO uint32_t LSTreshold;
    __IO uint32_t RhDescriptorA;
    __IO uint32_t RhDescriptorB;
    __IO uint32_t RhStatus;
    __IO uint32_t RhPortStatus1;
    __IO uint32_t RhPortStatus2;
         uint32_t RESERVED0[40];
    __I  uint32_t Module_ID;

    // USB On-The-Go Registers
    __I  uint32_t INTST;
    __IO uint32_t INTEN;
    __O  uint32_t INTSET;
    __O  uint32_t INTCLR;
    __IO uint32_t PORTSEL;
    __IO uint32_t TMR;
         uint32_t RESERVED1[58];

    // USB Device Interrupt Registers (the last one is below)
    __I  uint32_t DEVINTST;
    __IO uint32_t DEVINTEN;
    __O  uint32_t DEVINTCLR;
    __O  uint32_t DEVINTSET;

    // USB Device SIE Command Registers
    __O  uint32_t CMDCODE;
    __I  uint32_t CMDDATA;

    // USB Device Transfer Registers
    __I  uint32_t RXDATA;
    __O  uint32_t TXDATA;
    __I  uint32_t RXPLEN;
    __O  uint32_t TXPLEN;
    __IO uint32_t CTRL;

    // USB Device Interrupt Register (last one)
    __O  uint32_t DEVINTPRI;

    // USB Device Endpoint Interrupt Registers
    __I  uint32_t EPINTST;
    __IO uint32_t EPINTEN;
    __O  uint32_t EPINTCLR;
    __O  uint32_t EPINTSET;
    __O  uint32_t EPINTPRI;

    // USB Device Endpoint Realization Registers
    __IO uint32_t REEP;
    __O  uint32_t EPIN;
    __IO uint32_t MAXPSIZE;

    // USB Device DMA Registers
    __I  uint32_t DMARST;
    __O  uint32_t DMARCLR;
    __O  uint32_t DMARSET;
         uint32_t RESERVED2[9];
    __IO uint32_t UDCAH;
    __I  uint32_t EPDMAST;
    __O  uint32_t EPDMAEN;
    __O  uint32_t EPDMADIS;
    __I  uint32_t DMAINTST;
    __IO uint32_t DMAINTEN;
         uint32_t RESERVED3[2];
    __I  uint32_t EOTINTST;
    __O  uint32_t EOTINTCLR;
    __O  uint32_t EOTINTSET;
    __I  uint32_t NDDRINTST;
    __O  uint32_t NDDRINTCLR;
    __O  uint32_t NDDRINTSET;
    __I  uint32_t SYSERRINTST;
    __O  uint32_t SYSERRINTCLR;
    __O  uint32_t SYSERRINTSET;
         uint32_t RESERVED4[15];

    // USB OTG I2C Registers
    union
    {
        __I  uint32_t I2C_RX;
        __O  uint32_t I2C_TX;
    };
    __I  uint32_t I2C_STS;
    __IO uint32_t I2C_CTL;
    __IO uint32_t I2C_CLKHI;
    __IO uint32_t I2C_CLKLO;
         uint32_t RESERVED5[824];

    // USB Clock Control Registers
    union
    {
        __IO uint32_t USBCLKCTRL;
        __IO uint32_t OTGCLKCTRL;
    };
    union
    {
        __I  uint32_t USBCLKST;
        __I  uint32_t OTGCLKST;
    };
} LPC_USB_T;




// ------------ CRC Engine (CRC) ----------------------------------------------
typedef struct
{
    __IO uint32_t MODE;                     // CRC Mode Register
    __IO uint32_t SEED;                     // CRC SEED Register
    union
    {
        __I  uint32_t SUM;                  // CRC Checksum Register
        __O  uint32_t WRDATA32;             // CRC Data Register: write size 32-bit
        __O  uint16_t WRDATA16;             // CRC Data Register: write size 16-bit
        __O  uint8_t  WRDATA8;              // CRC Data Register: write size 8-bit
    };
} LPC_CRC_T;




// ------------ General Purpose Input/Output (GPIO) ---------------------------
typedef struct
{
    __IO uint32_t DIR;                      // (offset: 0x0000) GPIO Port Direction control register
         uint32_t RESERVED[3];
    __IO uint32_t MASK;                     // (offset: 0x0010) GPIO Mask register
    __IO uint32_t PIN;                      // (offset: 0x0014) Pin value register using FIOMASK
    __IO uint32_t SET;                      // (offset: 0x0018) Output Set register using FIOMASK
    __O  uint32_t CLR;                      // (offset: 0x001C) Output Clear register using FIOMASK
} LPC_GPIO_T;

typedef struct
{
    __I  uint32_t STATR;                    // GPIO Interrupt Status Register for Rising edge
    __I  uint32_t STATF;                    // GPIO Interrupt Status Register for Falling edge
    __O  uint32_t CLR;                      // GPIO Interrupt Clear  Register
    __IO uint32_t ENR;                      // GPIO Interrupt Enable Register 0 for Rising edge
    __IO uint32_t ENF;                      // GPIO Interrupt Enable Register 0 for Falling edge
} LPC_GPIOINT_PORT_T;

typedef struct
{
    __I  uint32_t STATUS;                   // GPIO overall Interrupt Status Register
    LPC_GPIOINT_PORT_T PORT0;               // GPIO Interrupt Registers for Port 0
         uint32_t RESERVED[3];
    LPC_GPIOINT_PORT_T PORT2;               // GPIO Interrupt Registers for Port 2
} LPC_GPIOINT_T;




// ------------ External Memory Controller (EMC) ------------------------------
typedef struct
{
    __IO uint32_t CONTROL;                  // Controls operation of the memory controller
    __I  uint32_t STATUS;                   // Provides EMC status information
    __IO uint32_t CONFIG;                   // Configures operation of the memory controller
         uint32_t RESERVED0[5];
    __IO uint32_t DYNAMICCONTROL;           // Controls dynamic memory operation
    __IO uint32_t DYNAMICREFRESH;           // Configures dynamic memory refresh operation
    __IO uint32_t DYNAMICREADCONFIG;        // Configures the dynamic memory read strategy
         uint32_t RESERVED1;
    __IO uint32_t DYNAMICRP;                // Selects the precharge command period
    __IO uint32_t DYNAMICRAS;               // Selects the active to precharge command period
    __IO uint32_t DYNAMICSREX;              // Selects the self-refresh exit time
    __IO uint32_t DYNAMICAPR;               // Selects the last-data-out to active command time
    __IO uint32_t DYNAMICDAL;               // Selects the data-in to active command time
    __IO uint32_t DYNAMICWR;                // Selects the write recovery time
    __IO uint32_t DYNAMICRC;                // Selects the active to active command period
    __IO uint32_t DYNAMICRFC;               // Selects the auto-refresh period
    __IO uint32_t DYNAMICXSR;               // Selects the exit self-refresh to active command time
    __IO uint32_t DYNAMICRRD;               // Selects the active bank A to active bank B latency
    __IO uint32_t DYNAMICMRD;               // Selects the load mode register to active command time
         uint32_t RESERVED2[9];
    __IO uint32_t STATICEXTENDEDWAIT;       // Selects time for long static memory read and write transfers
         uint32_t RESERVED3[31];
    __IO uint32_t DYNAMICCONFIG0;           // Selects the configuration information for dynamic memory chip select n
    __IO uint32_t DYNAMICRASCAS0;           // Selects the RAS and CAS latencies for dynamic memory chip select n
         uint32_t RESERVED4[6];
    __IO uint32_t DYNAMICCONFIG1;           // Selects the configuration information for dynamic memory chip select n
    __IO uint32_t DYNAMICRASCAS1;           // Selects the RAS and CAS latencies for dynamic memory chip select n
         uint32_t RESERVED5[6];
    __IO uint32_t DYNAMICCONFIG2;           // Selects the configuration information for dynamic memory chip select n
    __IO uint32_t DYNAMICRASCAS2;           // Selects the RAS and CAS latencies for dynamic memory chip select n
         uint32_t RESERVED6[6];
    __IO uint32_t DYNAMICCONFIG3;           // Selects the configuration information for dynamic memory chip select n
    __IO uint32_t DYNAMICRASCAS3;           // Selects the RAS and CAS latencies for dynamic memory chip select n
         uint32_t RESERVED7[38];
    __IO uint32_t STATICCONFIG0;            // Selects the memory configuration for static chip select n
    __IO uint32_t STATICWAITWEN0;           // Selects the delay from chip select n to write enable
    __IO uint32_t STATICWAITOEN0;           // Selects the delay from chip select n or address change, whichever is later, to output enable
    __IO uint32_t STATICWAITRD0;            // Selects the delay from chip select n to a read access
    __IO uint32_t STATICWAITPAG0;           // Selects the delay for asynchronous page mode sequential accesses for chip select n
    __IO uint32_t STATICWAITWR0;            // Selects the delay from chip select n to a write access
    __IO uint32_t STATICWAITTURN0;          // Selects bus turnaround cycles
         uint32_t RESERVED8;
    __IO uint32_t STATICCONFIG1;            // Selects the memory configuration for static chip select n
    __IO uint32_t STATICWAITWEN1;           // Selects the delay from chip select n to write enable
    __IO uint32_t STATICWAITOEN1;           // Selects the delay from chip select n or address change, whichever is later, to output enable
    __IO uint32_t STATICWAITRD1;            // Selects the delay from chip select n to a read access
    __IO uint32_t STATICWAITPAG1;           // Selects the delay for asynchronous page mode sequential accesses for chip select n
    __IO uint32_t STATICWAITWR1;            // Selects the delay from chip select n to a write access
    __IO uint32_t STATICWAITTURN1;          // Selects bus turnaround cycles
         uint32_t RESERVED9;
    __IO uint32_t STATICCONFIG2;            // Selects the memory configuration for static chip select n
    __IO uint32_t STATICWAITWEN2;           // Selects the delay from chip select n to write enable
    __IO uint32_t STATICWAITOEN2;           // Selects the delay from chip select n or address change, whichever is later, to output enable
    __IO uint32_t STATICWAITRD2;            // Selects the delay from chip select n to a read access
    __IO uint32_t STATICWAITPAG2;           // Selects the delay for asynchronous page mode sequential accesses for chip select n
    __IO uint32_t STATICWAITWR2;            // Selects the delay from chip select n to a write access
    __IO uint32_t STATICWAITTURN2;          // Selects bus turnaround cycles
         uint32_t RESERVED10;
    __IO uint32_t STATICCONFIG3;            // Selects the memory configuration for static chip select n
    __IO uint32_t STATICWAITWEN3;           // Selects the delay from chip select n to write enable
    __IO uint32_t STATICWAITOEN3;           // Selects the delay from chip select n or address change, whichever is later, to output enable
    __IO uint32_t STATICWAITRD3;            // Selects the delay from chip select n to a read access
    __IO uint32_t STATICWAITPAG3;           // Selects the delay for asynchronous page mode sequential accesses for chip select n
    __IO uint32_t STATICWAITWR3;            // Selects the delay from chip select n to a write access
    __IO uint32_t STATICWAITTURN3;          // Selects bus turnaround cycles
} LPC_EMC_T;




// ------------ Windowed Watchdog Timer (WWDT) --------------------------------
typedef struct
{
    __IO uint32_t MOD;                      // Watchdog mode register
    __IO uint32_t TC;                       // Watchdog timer constant register
    __O  uint32_t FEED;                     // Watchdog feed sequence register
    __I  uint32_t TV;                       // Watchdog timer value register
         uint32_t RESERVED;
    __IO uint32_t WARNINT;                  // Watchdog warning interrupt register
    __IO uint32_t WINDOW;                   // Watchdog timer window register
} LPC_WWDT_T;




// ------------ Timer (TIMER) -------------------------------------------------
typedef struct
{
    __IO uint32_t IR;                       // Interrupt Register
    __IO uint32_t TCR;                      // Timer Control Register
    __IO uint32_t TC;                       // Timer Counter
    __IO uint32_t PR;                       // Prescale Register
    __IO uint32_t PC;                       // Prescale Counter
    __IO uint32_t MCR;                      // Match Control Register
    __IO uint32_t MR[4];                    // Match Register
    __IO uint32_t CCR;                      // Capture Control Register
    __IO uint32_t CR[4];                    // Capture Register
    __IO uint32_t EMR;                      // External Match Register
         uint32_t RESERVED[12];
    __IO uint32_t CTCR;                     // Count Control Register
} LPC_TIMER_T;




// ------------ Pulse Width Modulator (PWM) -----------------------------------
typedef struct
{
  __IO uint32_t IR;                         // (offset: 0x000) Interrupt Register
  __IO uint32_t TCR;                        // (offset: 0x004) Timer Control Register
  __IO uint32_t TC;                         // (offset: 0x008) Timer Counter Register
  __IO uint32_t PR;                         // (offset: 0x00C) Prescale Register
  __IO uint32_t PC;                         // (offset: 0x010) Prescale Counter Register
  __IO uint32_t MCR;                        // (offset: 0x014) Match Control Register
  __IO uint32_t MR0;                        // (offset: 0x018) Match Register 0
  __IO uint32_t MR1;                        // (offset: 0x01C) Match Register 1
  __IO uint32_t MR2;                        // (offset: 0x020) Match Register 2
  __IO uint32_t MR3;                        // (offset: 0x024) Match Register 3
  __IO uint32_t CCR;                        // (offset: 0x028) Capture Control Register
  __I  uint32_t CR0;                        // (offset: 0x02C) Capture Register 0
  __I  uint32_t CR1;                        // (offset: 0x030) Capture Register 1
  __I  uint32_t CR2;                        // (offset: 0x034) Capture Register 2
  __I  uint32_t CR3;                        // (offset: 0x038) Capture Register 3
       uint32_t RESERVED0;
  __IO uint32_t MR4;                        // (offset: 0x040) Match Register 4
  __IO uint32_t MR5;                        // (offset: 0x044) Match Register 5
  __IO uint32_t MR6;                        // (offset: 0x048) Match Register 6
  __IO uint32_t PCR;                        // (offset: 0x04C) PWM Control Register
  __IO uint32_t LER;                        // (offset: 0x050) Load Enable Register
       uint32_t RESERVED1[7];
  __IO uint32_t CTCR;                       // (offset: 0x070) Counter Control Register
} LPC_PWM_T;




// ------------ UART (UARTx) --------------------------------------------------
// NOTE:
// There are three types of UARTs on the chip:
// (1) UART0, UART2 and UART3 are the standard UART.
// (2) UART1 is the standard with modem capability.
// (3) UART4 (USART) is the sync/async UART with smart card capability.
// More details can be found on the Users Manual.

#if 0   // ALL UART REGISTERS
typedef struct
{
    union
    {
        __I  uint32_t RBR;                  // Receiver Buffer Register
        __O  uint32_t THR;                  // Transmit Holding Register
        __IO uint32_t DLL;                  // Divisor Latch LSB
    };
    union
    {
        __IO uint32_t DLM;                  // Divisor Latch MSB
        __IO uint32_t IER;                  // Interrupt Enable Register
    };
    union
    {
        __I  uint32_t IIR;                  // Interrupt ID Register
        __O  uint32_t FCR;                  // FIFO Control Register
    };
    __IO uint32_t LCR;                      // Line Control Register
    __IO uint32_t MCR;                      // Modem Control Register
    __I  uint32_t LSR;                      // Line Status Register
    __I  uint32_t MSR;                      // Modem Status Register
    __IO uint32_t SCR;                      // Scratch Pad Register
    __IO uint32_t ACR;                      // Auto-baud Control Register
    __IO uint32_t ICR;                      // IrDA control register
    __IO uint32_t FDR;                      // Fractional Divider Register
    __IO uint32_t OSR;                      // Oversampling Register
    __IO uint32_t TER;                      // Transmit Enable Register
         uint32_t RESERVED[5];
    __IO uint32_t SCICTRL;                  // Smart card interface control register
    __IO uint32_t RS485CTRL;                // RS-485/EIA-485 Control
    __IO uint32_t RS485ADRMATCH;            // RS-485/EIA-485 address match
    __IO uint32_t RS485DLY;                 // RS-485/EIA-485 direction control delay
    __IO uint32_t SYNCCTRL;                 // Synchronous mode control register
} LPC_UART_T;
#endif

typedef struct
{
    union
    {
        __I  uint32_t RBR;                  // Receiver Buffer Register
        __O  uint32_t THR;                  // Transmit Holding Register
        __IO uint32_t DLL;                  // Divisor Latch LSB
    };
    union
    {
        __IO uint32_t DLM;                  // Divisor Latch MSB
        __IO uint32_t IER;                  // Interrupt Enable Register
    };
    union
    {
        __I  uint32_t IIR;                  // Interrupt ID Register
        __O  uint32_t FCR;                  // FIFO Control Register
    };
    __IO uint32_t LCR;                      // Line Control Register
         uint32_t RESERVED0;
    __I  uint32_t LSR;                      // Line Status Register
         uint32_t RESERVED1;
    __IO uint32_t SCR;                      // Scratch Pad Register
    __IO uint32_t ACR;                      // Auto-baud Control Register
         uint32_t RESERVED2;
    __IO uint32_t FDR;                      // Fractional Divider Register
         uint32_t RESERVED3;
    __IO uint32_t TER;                      // Transmit Enable Register
         uint32_t RESERVED4[6];
    __IO uint32_t RS485CTRL;                // RS-485/EIA-485 Control
    __IO uint32_t RS485ADRMATCH;            // RS-485/EIA-485 address match
    __IO uint32_t RS485DLY;                 // RS-485/EIA-485 direction control delay
         uint32_t RESERVED5;
} LPC_UART_T;

typedef struct
{
    union
    {
        __I  uint32_t RBR;                  // Receiver Buffer Register
        __O  uint32_t THR;                  // Transmit Holding Register
        __IO uint32_t DLL;                  // Divisor Latch LSB
    };
    union
    {
        __IO uint32_t DLM;                  // Divisor Latch MSB
        __IO uint32_t IER;                  // Interrupt Enable Register
    };
    union
    {
        __I  uint32_t IIR;                  // Interrupt ID Register
        __O  uint32_t FCR;                  // FIFO Control Register
    };
    __IO uint32_t LCR;                      // Line Control Register
    __IO uint32_t MCR;                      // Modem Control Register
    __I  uint32_t LSR;                      // Line Status Register
    __I  uint32_t MSR;                      // Modem Status Register
    __IO uint32_t SCR;                      // Scratch Pad Register
    __IO uint32_t ACR;                      // Auto-baud Control Register
         uint32_t RESERVED0;
    __IO uint32_t FDR;                      // Fractional Divider Register
         uint32_t RESERVED1;
    __IO uint32_t TER;                      // Transmit Enable Register
         uint32_t RESERVED2[6];
    __IO uint32_t RS485CTRL;                // RS-485/EIA-485 Control
    __IO uint32_t RS485ADRMATCH;            // RS-485/EIA-485 address match
    __IO uint32_t RS485DLY;                 // RS-485/EIA-485 direction control delay
         uint32_t RESERVED3;
} LPC_UART1_T;

typedef struct
{
    union
    {
        __I  uint32_t RBR;                  // Receiver Buffer Register
        __O  uint32_t THR;                  // Transmit Holding Register
        __IO uint32_t DLL;                  // Divisor Latch LSB
    };
    union
    {
        __IO uint32_t DLM;                  // Divisor Latch MSB
        __IO uint32_t IER;                  // Interrupt Enable Register
    };
    union
    {
        __I  uint32_t IIR;                  // Interrupt ID Register
        __O  uint32_t FCR;                  // FIFO Control Register
    };
    __IO uint32_t LCR;                      // Line Control Register
         uint32_t RESERVED0;
    __I  uint32_t LSR;                      // Line Status Register
         uint32_t RESERVED1;
    __IO uint32_t SCR;                      // Scratch Pad Register
    __IO uint32_t ACR;                      // Auto-baud Control Register
    __IO uint32_t ICR;                      // IrDA control register
    __IO uint32_t FDR;                      // Fractional Divider Register
    __IO uint32_t OSR;                      // Oversampling Register
         uint32_t RESERVED2[6];
    __IO uint32_t SCICTRL;                  // Smart card interface control register
    __IO uint32_t RS485CTRL;                // RS-485/EIA-485 Control
    __IO uint32_t RS485ADRMATCH;            // RS-485/EIA-485 address match
    __IO uint32_t RS485DLY;                 // RS-485/EIA-485 direction control delay
    __IO uint32_t SYNCCTRL;                 // Synchronous mode control register
} LPC_UART4_T;




// ------------ I2C-bus interface (I2C) ---------------------------------------
typedef struct
{
    __IO uint32_t CONSET;                   // (offset: 0x000) I2C Control Set Register
    __I  uint32_t STAT;                     // (offset: 0x004) I2C Status Register
    __IO uint32_t DAT;                      // (offset: 0x008) I2C Data Register
    __IO uint32_t ADR0;                     // (offset: 0x00C) I2C Slave Address Register 0
    __IO uint32_t SCLH;                     // (offset: 0x010) SCH Duty Cycle Register High Half Word
    __IO uint32_t SCLL;                     // (offset: 0x014) SCL Duty Cycle Register Low Half Word
    __O  uint32_t CONCLR;                   // (offset: 0x018) I2C Control Clear Register
    __IO uint32_t MMCTRL;                   // (offset: 0x01C) Monitor mode control register
    __IO uint32_t ADR1;                     // (offset: 0x020) I2C Slave Address Register 1
    __IO uint32_t ADR2;                     // (offset: 0x024) I2C Slave Address Register 2
    __IO uint32_t ADR3;                     // (offset: 0x028) I2C Slave Address Register 3
    __I  uint32_t DATA_BUFFER;              // (offset: 0x02C) Data buffer register
    union
    {
        __IO uint32_t MASK[4];
        struct
        {
            __IO uint32_t MASK0;            // (offset: 0x030) I2C Slave address mask register 0
            __IO uint32_t MASK1;            // (offset: 0x034) I2C Slave address mask register 1
            __IO uint32_t MASK2;            // (offset: 0x038) I2C Slave address mask register 2
            __IO uint32_t MASK3;            // (offset: 0x03C) I2C Slave address mask register 3
        };
    };
} LPC_I2C_T;




// ------------ Real Time Clock (RTC) -----------------------------------------
#define RTC_TIMETYPE_NUM        8

typedef enum
{
    RTC_TIMETYPE_SECOND,                    // Seconds
    RTC_TIMETYPE_MINUTE,                    // Minutes
    RTC_TIMETYPE_HOUR,                      // Hours
    RTC_TIMETYPE_DAYOFMONTH,                // Day of month
    RTC_TIMETYPE_DAYOFWEEK,                 // Day of week
    RTC_TIMETYPE_DAYOFYEAR,                 // Day of year
    RTC_TIMETYPE_MONTH,                     // Months
    RTC_TIMETYPE_YEAR                       // Years
} RTC_TIMETYPE_T;

#define RTC_EVENT_CHANNEL_NUM   3

typedef enum
{
    RTC_EVENT_CHANNEL_0 = 0,
    RTC_EVENT_CHANNEL_1,
    RTC_EVENT_CHANNEL_2
} RTC_EVENT_CHANNEL_T;

typedef struct
{
    __IO uint32_t ILR;                      // Interrupt Location Register
         uint32_t RESERVED0;
    __IO uint32_t CCR;                      // Clock Control Register
    __IO uint32_t CIIR;                     // Counter Increment Interrupt Register
    __IO uint32_t AMR;                      // Alarm Mask Register
    __I  uint32_t CTIME[3];                 // Consolidated Time Register 0, 1, 2
    __IO uint32_t TIME[RTC_TIMETYPE_NUM];   // Timer field registers
    __IO uint32_t CALIBRATION;              // Calibration Value Register
    __IO uint32_t GPREG[5];                 // General Purpose Storage Registers
    __IO uint32_t RTC_AUXEN;                // RTC Auxiliary Enable register
    __IO uint32_t RTC_AUX;                  // RTC Auxiliary control register
    __IO uint32_t ALARM[RTC_TIMETYPE_NUM];  // Alarm field registers
    __IO uint32_t ERSTATUS;                 // Event Monitor/Recorder Status register
    __IO uint32_t ERCONTROL;                // Event Monitor/Recorder Control register
    __I  uint32_t ERCOUNTERS;               // Event Monitor/Recorder Counters register
         uint32_t RESERVED1;
    __I  uint32_t ERFIRSTSTAMP[RTC_EVENT_CHANNEL_NUM];  // Event Monitor/Recorder First Stamp registers
         uint32_t RESERVED2;
    __I  uint32_t ERLASTSTAMP[RTC_EVENT_CHANNEL_NUM];   // Event Monitor/Recorder Last Stamp registers
} LPC_RTC_T;




// ------------ I/O configuration (IOCON) -------------------------------------
typedef struct
{
    union
    {
        __IO uint32_t PIO[165];             // Digital I/O control for ports
        struct
        {
            __IO uint32_t PIO0_0;           // (offset: 0x000)
            __IO uint32_t PIO0_1;
            __IO uint32_t PIO0_2;
            __IO uint32_t PIO0_3;
            __IO uint32_t PIO0_4;
            __IO uint32_t PIO0_5;
            __IO uint32_t PIO0_6;
            __IO uint32_t PIO0_7;
            __IO uint32_t PIO0_8;           // (offset: 0x020)
            __IO uint32_t PIO0_9;
            __IO uint32_t PIO0_10;
            __IO uint32_t PIO0_11;
            __IO uint32_t PIO0_12;
            __IO uint32_t PIO0_13;
            __IO uint32_t PIO0_14;
            __IO uint32_t PIO0_15;
            __IO uint32_t PIO0_16;          // (offset: 0x040)
            __IO uint32_t PIO0_17;
            __IO uint32_t PIO0_18;
            __IO uint32_t PIO0_19;
            __IO uint32_t PIO0_20;
            __IO uint32_t PIO0_21;
            __IO uint32_t PIO0_22;
            __IO uint32_t PIO0_23;
            __IO uint32_t PIO0_24;          // (offset: 0x060)
            __IO uint32_t PIO0_25;
            __IO uint32_t PIO0_26;
            __IO uint32_t PIO0_27;
            __IO uint32_t PIO0_28;
            __IO uint32_t PIO0_29;
            __IO uint32_t PIO0_30;
            __IO uint32_t PIO0_31;

            __IO uint32_t PIO1_0;           // (offset: 0x080)
            __IO uint32_t PIO1_1;
            __IO uint32_t PIO1_2;
            __IO uint32_t PIO1_3;
            __IO uint32_t PIO1_4;
            __IO uint32_t PIO1_5;
            __IO uint32_t PIO1_6;
            __IO uint32_t PIO1_7;
            __IO uint32_t PIO1_8;           // (offset: 0x0A0)
            __IO uint32_t PIO1_9;
            __IO uint32_t PIO1_10;
            __IO uint32_t PIO1_11;
            __IO uint32_t PIO1_12;
            __IO uint32_t PIO1_13;
            __IO uint32_t PIO1_14;
            __IO uint32_t PIO1_15;
            __IO uint32_t PIO1_16;          // (offset: 0x0C0)
            __IO uint32_t PIO1_17;
            __IO uint32_t PIO1_18;
            __IO uint32_t PIO1_19;
            __IO uint32_t PIO1_20;
            __IO uint32_t PIO1_21;
            __IO uint32_t PIO1_22;
            __IO uint32_t PIO1_23;
            __IO uint32_t PIO1_24;          // (offset: 0x0E0)
            __IO uint32_t PIO1_25;
            __IO uint32_t PIO1_26;
            __IO uint32_t PIO1_27;
            __IO uint32_t PIO1_28;
            __IO uint32_t PIO1_29;
            __IO uint32_t PIO1_30;
            __IO uint32_t PIO1_31;

            __IO uint32_t PIO2_0;           // (offset: 0x100)
            __IO uint32_t PIO2_1;
            __IO uint32_t PIO2_2;
            __IO uint32_t PIO2_3;
            __IO uint32_t PIO2_4;
            __IO uint32_t PIO2_5;
            __IO uint32_t PIO2_6;
            __IO uint32_t PIO2_7;
            __IO uint32_t PIO2_8;           // (offset: 0x120)
            __IO uint32_t PIO2_9;
            __IO uint32_t PIO2_10;
            __IO uint32_t PIO2_11;
            __IO uint32_t PIO2_12;
            __IO uint32_t PIO2_13;
            __IO uint32_t PIO2_14;
            __IO uint32_t PIO2_15;
            __IO uint32_t PIO2_16;          // (offset: 0x140)
            __IO uint32_t PIO2_17;
            __IO uint32_t PIO2_18;
            __IO uint32_t PIO2_19;
            __IO uint32_t PIO2_20;
            __IO uint32_t PIO2_21;
            __IO uint32_t PIO2_22;
            __IO uint32_t PIO2_23;
            __IO uint32_t PIO2_24;          // (offset: 0x160)
            __IO uint32_t PIO2_25;
            __IO uint32_t PIO2_26;
            __IO uint32_t PIO2_27;
            __IO uint32_t PIO2_28;
            __IO uint32_t PIO2_29;
            __IO uint32_t PIO2_30;
            __IO uint32_t PIO2_31;

            __IO uint32_t PIO3_0;           // (offset: 0x180)
            __IO uint32_t PIO3_1;
            __IO uint32_t PIO3_2;
            __IO uint32_t PIO3_3;
            __IO uint32_t PIO3_4;
            __IO uint32_t PIO3_5;
            __IO uint32_t PIO3_6;
            __IO uint32_t PIO3_7;
            __IO uint32_t PIO3_8;           // (offset: 0x1A0)
            __IO uint32_t PIO3_9;
            __IO uint32_t PIO3_10;
            __IO uint32_t PIO3_11;
            __IO uint32_t PIO3_12;
            __IO uint32_t PIO3_13;
            __IO uint32_t PIO3_14;
            __IO uint32_t PIO3_15;
            __IO uint32_t PIO3_16;          // (offset: 0x1C0)
            __IO uint32_t PIO3_17;
            __IO uint32_t PIO3_18;
            __IO uint32_t PIO3_19;
            __IO uint32_t PIO3_20;
            __IO uint32_t PIO3_21;
            __IO uint32_t PIO3_22;
            __IO uint32_t PIO3_23;
            __IO uint32_t PIO3_24;          // (offset: 0x1E0)
            __IO uint32_t PIO3_25;
            __IO uint32_t PIO3_26;
            __IO uint32_t PIO3_27;
            __IO uint32_t PIO3_28;
            __IO uint32_t PIO3_29;
            __IO uint32_t PIO3_30;
            __IO uint32_t PIO3_31;

            __IO uint32_t PIO4_0;           // (offset: 0x200)
            __IO uint32_t PIO4_1;
            __IO uint32_t PIO4_2;
            __IO uint32_t PIO4_3;
            __IO uint32_t PIO4_4;
            __IO uint32_t PIO4_5;
            __IO uint32_t PIO4_6;
            __IO uint32_t PIO4_7;
            __IO uint32_t PIO4_8;           // (offset: 0x220)
            __IO uint32_t PIO4_9;
            __IO uint32_t PIO4_10;
            __IO uint32_t PIO4_11;
            __IO uint32_t PIO4_12;
            __IO uint32_t PIO4_13;
            __IO uint32_t PIO4_14;
            __IO uint32_t PIO4_15;
            __IO uint32_t PIO4_16;          // (offset: 0x240)
            __IO uint32_t PIO4_17;
            __IO uint32_t PIO4_18;
            __IO uint32_t PIO4_19;
            __IO uint32_t PIO4_20;
            __IO uint32_t PIO4_21;
            __IO uint32_t PIO4_22;
            __IO uint32_t PIO4_23;
            __IO uint32_t PIO4_24;          // (offset: 0x260)
            __IO uint32_t PIO4_25;
            __IO uint32_t PIO4_26;
            __IO uint32_t PIO4_27;
            __IO uint32_t PIO4_28;
            __IO uint32_t PIO4_29;
            __IO uint32_t PIO4_30;
            __IO uint32_t PIO4_31;

            __IO uint32_t PIO5_0;           // (offset: 0x280)
            __IO uint32_t PIO5_1;
            __IO uint32_t PIO5_2;
            __IO uint32_t PIO5_3;
            __IO uint32_t PIO5_4;           // (offset: 0x290)
        };
    };
} LPC_IOCON_T;




// ------------ SSP interface (SSP) -------------------------------------------
typedef struct
{
  __IO uint32_t CR0;                        // (offset: 0x000) Control Register 0
  __IO uint32_t CR1;                        // (offset: 0x004) Control Register 1
  __IO uint32_t DR;                         // (offset: 0x008) Data Register
  __I  uint32_t SR;                         // (offset: 0x00C) Status Register
  __IO uint32_t CPSR;                       // (offset: 0x010) Clock Prescale Register
  __IO uint32_t IMSC;                       // (offset: 0x014) Interrupt Mask Set and Clear Register
  __IO uint32_t RIS;                        // (offset: 0x018) Raw Interrupt Status Register
  __IO uint32_t MIS;                        // (offset: 0x01C) Masked Interrupt Status Register
  __IO uint32_t ICR;                        // (offset: 0x020) SSPICR Interrupt Clear Register
  __IO uint32_t DMACR;                      // (offset: 0x024) DMA Control Register
} LPC_SSP_T;




// ------------ Analog-to-Digital Converter (ADC) -----------------------------
typedef struct
{
  __IO uint32_t CR;                         // (offset: 0x000)       A/D Control Register
  __IO uint32_t GDR;                        // (offset: 0x004)       A/D Global Data Register
       uint32_t RESERVED;
  __IO uint32_t INTEN;                      // (offset: 0x00C)       A/D Interrupt Enable Register
  __I  uint32_t DR[8];                      // (offset: 0x010-0x02C) A/D Channel 0..7 Data Register
  __I  uint32_t STAT;                       // (offset: 0x030)       A/D Status Register
  __IO uint32_t TRM;                        // (offset: 0x034)       ADC trim register
} LPC_ADC_T;




// ------------ CAN controller (CAN) ------------------------------------------
#define CAN_AF_RAM_ENTRY_NUM    512

typedef struct
{
    __IO uint32_t MASK[CAN_AF_RAM_ENTRY_NUM];   // Acceptance Filter RAM ID mask register
} LPC_CAN_AF_RAM_T;

typedef struct
{
    __IO uint32_t AFMR;                     // (offset: 0x000) Acceptance Filter Register
    __IO uint32_t SFF_SA;                   // (offset: 0x004) Standard Frame Individual Start Address Register
    __IO uint32_t SFF_GRP_SA;               // (offset: 0x008) Standard Frame Group Start Address Register
    __IO uint32_t EFF_SA;                   // (offset: 0x00C) Extended Frame Start Address Register
    __IO uint32_t EFF_GRP_SA;               // (offset: 0x010) Extended Frame Group Start Address Register
    __IO uint32_t ENDOFTABLE;               // (offset: 0x014) End of AF Tables register
    __I  uint32_t LUTERRAD;                 // (offset: 0x018) LUT Error Address register
    __I  uint32_t LUTERR;                   // (offset: 0x01C) LUT Error Register
    __IO uint32_t FCANIE;                   // (offset: 0x020) FullCAN Interrupt Enable register
    union
    {
        __IO uint32_t FCANIC[2];
        struct
        {
            __IO uint32_t FCANIC0;          // (offset: 0x024) FullCAN Interrupt and Capture register 0
            __IO uint32_t FCANIC1;          // (offset: 0x028) FullCAN Interrupt and Capture register 1
        };
    };
} LPC_CAN_AF_T;

typedef struct
{
  __I  uint32_t TXSR;                       // CAN Central Transmit Status Register
  __I  uint32_t RXSR;                       // CAN Central Receive Status Register
  __I  uint32_t MSR;                        // CAN Central Miscellaneous Register
} LPC_CAN_CENTRAL_T;

// ************* UNFINISHED ***************************************************

typedef struct
{
    ///Offset: 0x00000000 - Controls the operating mode of the CAN Controller
    __IO uint32_t MOD;

    ///Offset: 0x00000004 - Command bits that affect the state
    __O  uint32_t CMR;

    ///Offset: 0x00000008 - Global Controller Status and Error Counters
    __IO uint32_t GSR;

    ///Offset: 0x0000000C - Interrupt status, Arbitration Lost Capture, Error Code Capture
    __I  uint32_t ICR;

    ///Offset: 0x00000010 - Interrupt Enable Register
    __IO uint32_t IER;

    ///Offset: 0x00000014 - Bus Timing Register
    __IO uint32_t BTR;

    ///Offset: 0x00000018 - Error Warning Limit
    __IO uint32_t EWL;

    ///Offset: 0x0000001C - Status Register
    __I  uint32_t SR;

    ///Offset: 0x00000020 - Receive frame status
    __IO uint32_t RFS;

    ///Offset: 0x00000024 - Received Identifier
    __IO uint32_t RID;

    ///Offset: 0x00000028 - Received data bytes 1-4
    __IO uint32_t RDA;

    ///Offset: 0x0000002C - Received data bytes 5-8
    __IO uint32_t RDB;

    ///Offset: 0x00000030 - Transmit frame info (Tx Buffer 1)
    __IO uint32_t TFI1;

    ///Offset: 0x00000034 - Transmit Identifier (Tx Buffer 1)
    __IO uint32_t TID1;

    ///Offset: 0x00000038 - Transmit data bytes 1-4 (Tx Buffer 1)
    __IO uint32_t TDA1;

    ///Offset: 0x0000003C - Transmit data bytes 5-8 (Tx Buffer 1)
    __IO uint32_t TDB1;

    ///Offset: 0x00000040 - Transmit frame info (Tx Buffer 2)
    __IO uint32_t TFI2;

    ///Offset: 0x00000044 - Transmit Identifier (Tx Buffer 2)
    __IO uint32_t TID2;

    ///Offset: 0x00000048 - Transmit data bytes 1-4 (Tx Buffer 2)
    __IO uint32_t TDA2;

    ///Offset: 0x0000004C - Transmit data bytes 5-8 (Tx Buffer 2)
    __IO uint32_t TDB2;

    ///Offset: 0x00000050 - Transmit frame info (Tx Buffer 3)
    __IO uint32_t TFI3;

    ///Offset: 0x00000054 - Transmit Identifier (Tx Buffer 3)
    __IO uint32_t TID3;

    ///Offset: 0x00000058 - Transmit data bytes 1-4 (Tx Buffer 3)
    __IO uint32_t TDA3;

    ///Offset: 0x0000005C - Transmit data bytes 5-8 (Tx Buffer 3)
    __IO uint32_t TDB3;
} LPC_CAN_TypeDef;




/*------------- Digital-to-Analog Converter (DAC) ----------------------------*/
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CTRL;
  __IO uint32_t CNTVAL;
} LPC_DAC_TypeDef;


/*------------- Inter IC Sound (I2S) -----------------------------------------*/
typedef struct
{
  __IO uint32_t DAO;
  __IO uint32_t DAI;
  __O  uint32_t TXFIFO;
  __I  uint32_t RXFIFO;
  __I  uint32_t STATE;
  __IO uint32_t DMA1;
  __IO uint32_t DMA2;
  __IO uint32_t IRQ;
  __IO uint32_t TXRATE;
  __IO uint32_t RXRATE;
  __IO uint32_t TXBITRATE;
  __IO uint32_t RXBITRATE;
  __IO uint32_t TXMODE;
  __IO uint32_t RXMODE;
} LPC_I2S_TypeDef;






/*------------- Motor Control Pulse-Width Modulation (MCPWM) -----------------*/
typedef struct
{
  __I  uint32_t CON;
  __O  uint32_t CON_SET;
  __O  uint32_t CON_CLR;
  __I  uint32_t CAPCON;
  __O  uint32_t CAPCON_SET;
  __O  uint32_t CAPCON_CLR;
  __IO uint32_t TC0;
  __IO uint32_t TC1;
  __IO uint32_t TC2;
  __IO uint32_t LIM0;
  __IO uint32_t LIM1;
  __IO uint32_t LIM2;
  __IO uint32_t MAT0;
  __IO uint32_t MAT1;
  __IO uint32_t MAT2;
  __IO uint32_t DT;
  __IO uint32_t CP;
  __IO uint32_t CAP0;
  __IO uint32_t CAP1;
  __IO uint32_t CAP2;
  __I  uint32_t INTEN;
  __O  uint32_t INTEN_SET;
  __O  uint32_t INTEN_CLR;
  __I  uint32_t CNTCON;
  __O  uint32_t CNTCON_SET;
  __O  uint32_t CNTCON_CLR;
  __I  uint32_t INTF;
  __O  uint32_t INTF_SET;
  __O  uint32_t INTF_CLR;
  __O  uint32_t CAP_CLR;
} LPC_MCPWM_TypeDef;

/*------------- Quadrature Encoder Interface (QEI) ---------------------------*/
typedef struct
{
  __O  uint32_t CON;
  __I  uint32_t STAT;
  __IO uint32_t CONF;
  __I  uint32_t POS;
  __IO uint32_t MAXPOS;
  __IO uint32_t CMPOS0;
  __IO uint32_t CMPOS1;
  __IO uint32_t CMPOS2;
  __I  uint32_t INXCNT;
  __IO uint32_t INXCMP0;
  __IO uint32_t LOAD;
  __I  uint32_t TIME;
  __I  uint32_t VEL;
  __I  uint32_t CAP;
  __IO uint32_t VELCOMP;
  __IO uint32_t FILTERPHA;
  __IO uint32_t FILTERPHB;
  __IO uint32_t FILTERINX;
  __IO uint32_t WINDOW;
  __IO uint32_t INXCMP1;
  __IO uint32_t INXCMP2;
       uint32_t RESERVED0[993];
  __O  uint32_t IEC;
  __O  uint32_t IES;
  __I  uint32_t INTSTAT;
  __I  uint32_t IE;
  __O  uint32_t CLR;
  __O  uint32_t SET;
} LPC_QEI_TypeDef;

/*------------- SD/MMC card Interface (MCI)-----------------------------------*/
typedef struct
{
  __IO uint32_t POWER;
  __IO uint32_t CLOCK;
  __IO uint32_t ARGUMENT;
  __IO uint32_t COMMAND;
  __I  uint32_t RESP_CMD;
  __I  uint32_t RESP0;
  __I  uint32_t RESP1;
  __I  uint32_t RESP2;
  __I  uint32_t RESP3;
  __IO uint32_t DATATMR;
  __IO uint32_t DATALEN;
  __IO uint32_t DATACTRL;
  __I  uint32_t DATACNT;
  __I  uint32_t STATUS;
  __O  uint32_t CLEAR;
  __IO uint32_t MASK0;
       uint32_t RESERVED0[2];
  __I  uint32_t FIFOCNT;
       uint32_t RESERVED1[13];
  __IO uint32_t FIFO[16];
} LPC_MCI_TypeDef;










/*------------- EEPROM Controller (EEPROM) -----------------------------------*/
typedef struct
{
  __IO uint32_t CMD;            /* 0x0080 */
  __IO uint32_t ADDR;
  __IO uint32_t WDATA;
  __IO uint32_t RDATA;
  __IO uint32_t WSTATE;         /* 0x0090 */
  __IO uint32_t CLKDIV;
  __IO uint32_t PWRDWN;         /* 0x0098 */
       uint32_t RESERVED0[975];
  __IO uint32_t INT_CLR_ENABLE; /* 0x0FD8 */
  __IO uint32_t INT_SET_ENABLE;
  __IO uint32_t INT_STATUS;     /* 0x0FE0 */
  __IO uint32_t INT_ENABLE;
  __IO uint32_t INT_CLR_STATUS;
  __IO uint32_t INT_SET_STATUS;
} LPC_EEPROM_TypeDef;


/*------------- COMPARATOR ----------------------------------------------------*/

typedef struct {                                    /*!< (@ 0x40020000) COMPARATOR Structure                                   */
  __IO uint32_t  CTRL;                              /*!< (@ 0x40020000) Comparator block control register                      */
  __IO uint32_t  CTRL0;                             /*!< (@ 0x40020004) Comparator 0 control register                          */
  __IO uint32_t  CTRL1;                             /*!< (@ 0x40020008) Comparator 1 control register                          */
} LPC_COMPARATOR_Type;




// ISO C++ prohibits anonymous structs [-Wpedantic]
// #pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop




//-------------------------------------------------------------------------
// Peripheral memory map
//-------------------------------------------------------------------------

// Base addresses
#define LPC_FLASH_BASE          (0x00000000UL)
#define LPC_RAM_BASE            (0x10000000UL)
//#define LPC_PERI_RAM_BASE       (0x20000000UL)
//#define LPC_AHBRAM1_BASE        (0x20004000UL)
#define LPC_AHB_BASE            (0x20080000UL)
#define LPC_APB0_BASE           (0x40000000UL)
#define LPC_APB1_BASE           (0x40080000UL)

// EEPROM
#define LPC_EEPROM_BASE         (LPC_FLASH_BASE + 0x200080)

// AHB peripherals
#define LPC_GPDMA_BASE          (LPC_AHB_BASE + 0x00000)
#define LPC_ENET_BASE           (LPC_AHB_BASE + 0x04000)
#define LPC_LCD_BASE            (LPC_AHB_BASE + 0x08000)
#define LPC_USB_BASE            (LPC_AHB_BASE + 0x0C000)
#define LPC_CRC_BASE            (LPC_AHB_BASE + 0x10000)
#define LPC_SPIFI_BASE          (LPC_AHB_BASE + 0x14000)
#define LPC_GPIO0_BASE          (LPC_AHB_BASE + 0x18000)
#define LPC_GPIO1_BASE          (LPC_AHB_BASE + 0x18020)
#define LPC_GPIO2_BASE          (LPC_AHB_BASE + 0x18040)
#define LPC_GPIO3_BASE          (LPC_AHB_BASE + 0x18060)
#define LPC_GPIO4_BASE          (LPC_AHB_BASE + 0x18080)
#define LPC_GPIO5_BASE          (LPC_AHB_BASE + 0x180A0)
#define LPC_EMC_BASE            (LPC_AHB_BASE + 0x1C000)

// APB0 peripherals
#define LPC_WWDT_BASE           (LPC_APB0_BASE + 0x00000)
#define LPC_TIMER0_BASE         (LPC_APB0_BASE + 0x04000)
#define LPC_TIMER1_BASE         (LPC_APB0_BASE + 0x08000)
#define LPC_UART0_BASE          (LPC_APB0_BASE + 0x0C000)
#define LPC_UART1_BASE          (LPC_APB0_BASE + 0x10000)
#define LPC_PWM0_BASE           (LPC_APB0_BASE + 0x14000)
#define LPC_PWM1_BASE           (LPC_APB0_BASE + 0x18000)
#define LPC_I2C0_BASE           (LPC_APB0_BASE + 0x1C000)
#define LPC_CMP_BASE            (LPC_APB0_BASE + 0x20000)
#define LPC_RTC_BASE            (LPC_APB0_BASE + 0x24000)
#define LPC_GPIOINT_BASE        (LPC_APB0_BASE + 0x28080)
#define LPC_IOCON_BASE          (LPC_APB0_BASE + 0x2C000)
#define LPC_SSP1_BASE           (LPC_APB0_BASE + 0x30000)
#define LPC_ADC_BASE            (LPC_APB0_BASE + 0x34000)
#define LPC_CAN_AF_RAM_BASE     (LPC_APB0_BASE + 0x38000)
#define LPC_CAN_AF_BASE         (LPC_APB0_BASE + 0x3C000)
#define LPC_CAN_CENTRAL_BASE    (LPC_APB0_BASE + 0x40000)
#define LPC_CAN1_BASE           (LPC_APB0_BASE + 0x44000)
#define LPC_CAN2_BASE           (LPC_APB0_BASE + 0x48000)
#define LPC_I2C1_BASE           (LPC_APB0_BASE + 0x5C000)

// APB1 peripherals
#define LPC_SSP0_BASE           (LPC_APB1_BASE + 0x08000)
#define LPC_DAC_BASE            (LPC_APB1_BASE + 0x0C000)
#define LPC_TIMER2_BASE         (LPC_APB1_BASE + 0x10000)
#define LPC_TIMER3_BASE         (LPC_APB1_BASE + 0x14000)
#define LPC_UART2_BASE          (LPC_APB1_BASE + 0x18000)
#define LPC_UART3_BASE          (LPC_APB1_BASE + 0x1C000)
#define LPC_I2C2_BASE           (LPC_APB1_BASE + 0x20000)
#define LPC_UART4_BASE          (LPC_APB1_BASE + 0x24000)
#define LPC_I2S_BASE            (LPC_APB1_BASE + 0x28000)
#define LPC_SSP2_BASE           (LPC_APB1_BASE + 0x2C000)
#define LPC_MCPWM_BASE          (LPC_APB1_BASE + 0x38000)
#define LPC_QEI_BASE            (LPC_APB1_BASE + 0x3C000)
#define LPC_SDC_BASE            (LPC_APB1_BASE + 0x40000)
#define LPC_SYSCON_BASE         (LPC_APB1_BASE + 0x7C000)




//-------------------------------------------------------------------------
// Peripheral declarations
//-------------------------------------------------------------------------
// IAP entry function pointer
//static const LPC_ROM_IAP_ENTRY_T iap_entry = (LPC_ROM_IAP_ENTRY_T)(LPC_ROM_IAP_BASE);

#define LPC_SYSCON              ((LPC_SYSCON_T     *) LPC_SYSCON_BASE)
#define LPC_WWDT                ((LPC_WWDT_T       *) LPC_WDWT_BASE)
#define LPC_TIMER0              ((LPC_TIMER_T      *) LPC_TIMER0_BASE)
#define LPC_TIMER1              ((LPC_TIMER_T      *) LPC_TIMER1_BASE)
#define LPC_TIMER2              ((LPC_TIMER_T      *) LPC_TIMER2_BASE)
#define LPC_TIMER3              ((LPC_TIMER_T      *) LPC_TIMER3_BASE)
#define LPC_UART0               ((LPC_UART_T       *) LPC_UART0_BASE)
#define LPC_UART1               ((LPC_UART1_T      *) LPC_UART1_BASE)
#define LPC_UART2               ((LPC_UART_T       *) LPC_UART2_BASE)
#define LPC_UART3               ((LPC_UART_T       *) LPC_UART3_BASE)
#define LPC_UART4               ((LPC_UART4_T      *) LPC_UART4_BASE)
#define LPC_PWM0                ((LPC_PWM_T        *) LPC_PWM0_BASE)
#define LPC_PWM1                ((LPC_PWM_T        *) LPC_PWM1_BASE)
#define LPC_I2C0                ((LPC_I2C_T        *) LPC_I2C0_BASE)
#define LPC_I2C1                ((LPC_I2C_T        *) LPC_I2C1_BASE)
#define LPC_I2C2                ((LPC_I2C_T        *) LPC_I2C2_BASE)
#define LPC_I2S                 ((LPC_I2S_T        *) LPC_I2S_BASE)
#define LPC_CMP                 ((LPC_CMP_T        *) LPC_CMP_BASE)
#define LPC_RTC                 ((LPC_RTC_T        *) LPC_RTC_BASE)
#define LPC_GPIOINT             ((LPC_GPIOINT_T    *) LPC_GPIOINT_BASE)
#define LPC_IOCON               ((LPC_IOCON_T      *) LPC_IOCON_BASE)
#define LPC_SSP0                ((LPC_SSP_T        *) LPC_SSP0_BASE)
#define LPC_SSP1                ((LPC_SSP_T        *) LPC_SSP1_BASE)
#define LPC_SSP2                ((LPC_SSP_T        *) LPC_SSP2_BASE)
#define LPC_ADC                 ((LPC_ADC_T        *) LPC_ADC_BASE)
#define LPC_DAC                 ((LPC_DAC_T        *) LPC_DAC_BASE)
#define LPC_CAN_AF_RAM          ((LPC_CAN_AF_RAM_T *) LPC_CAN_AF_RAM_BASE)
#define LPC_CAN_AF              ((LPC_CAN_AF_T     *) LPC_CAN_AF_BASE)
#define LPC_CAN_CENTRAL         ((LPC_CAN_CENTRAL_T*) LPC_CAN_CENTRAL_BASE)
#define LPC_CAN1                ((LPC_CAN_T        *) LPC_CAN1_BASE)
#define LPC_CAN2                ((LPC_CAN_T        *) LPC_CAN2_BASE)
#define LPC_MCPWM               ((LPC_MCPWM_T      *) LPC_MCPWM_BASE)
#define LPC_QEI                 ((LPC_QEI_T        *) LPC_QEI_BASE)
#define LPC_SDC                 ((LPC_SDC_T        *) LPC_SDC_BASE)
#define LPC_GPDMA               ((LPC_GPDMA_T      *) LPC_GPDMA_BASE)
#define LPC_ENET                ((LPC_ENET_T       *) LPC_ENET_BASE)
#define LPC_LCD                 ((LPC_LCD_T        *) LPC_LCD_BASE)
#define LPC_USB                 ((LPC_USB_T        *) LPC_USB_BASE)
#define LPC_GPIO0               ((LPC_GPIO_T       *) LPC_GPIO0_BASE)
#define LPC_GPIO1               ((LPC_GPIO_T       *) LPC_GPIO1_BASE)
#define LPC_GPIO2               ((LPC_GPIO_T       *) LPC_GPIO2_BASE)
#define LPC_GPIO3               ((LPC_GPIO_T       *) LPC_GPIO3_BASE)
#define LPC_GPIO4               ((LPC_GPIO_T       *) LPC_GPIO4_BASE)
#define LPC_GPIO5               ((LPC_GPIO_T       *) LPC_GPIO5_BASE)
#define LPC_EMC                 ((LPC_EMC_T        *) LPC_EMC_BASE)
#define LPC_CRC                 ((LPC_CRC_T        *) LPC_CRC_BASE)
#define LPC_EEPROM              ((LPC_EEPROM_T     *) LPC_EEPROM_BASE)




#ifdef __cplusplus
} // extern "C"
#endif

#endif // __XARMLIB_TARGETS_LPC408X_7X_CMSIS_HPP
