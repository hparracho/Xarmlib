// ----------------------------------------------------------------------------
// @file    xarmlib_config.hpp
// @brief   Xarmlib configuration file for LPCXpresso845-Max-blinky example.
// @date    2 September 2020
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

#ifndef __XARMLIB_CONFIG_HPP
#define __XARMLIB_CONFIG_HPP

#include "hal/hal_faim.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_system.hpp"

namespace xarmlib
{




// ----------------------------------------------------------------------------
// SYSTEM AND MCU SPECIFIC DEFINITIONS
// ----------------------------------------------------------------------------

// CRP (Code Read Protect) word definition
#define XARMLIB_CONFIG_CRP_SETTING                              (CRP_NO_CRP)

// Buffer for Micro Trace Buffer (MTB) instruction trace on Cortex-M0+ parts
#ifndef NDEBUG
#define XARMLIB_ENABLE_MTB                                      (1)
#define XARMLIB_CONFIG_MTB_BUFFER_SIZE                          (256)
#endif

// -------- SYSTEM CLOCK ------------------------------------------------------
constexpr System::Clock XARMLIB_CONFIG_SYSTEM_CLOCK { System::Clock::osc_30mhz };

// -------- FAIM --------------------------------------------------------------
constexpr System::Swd             XARMLIB_CONFIG_FAIM_SWD              { System::Swd::enabled }; // Enabled by default (!!!CAUTION WHEN DISABLING!!!)
constexpr Pin::Name               XARMLIB_CONFIG_FAIM_ISP_UART0_TX_PIN { Pin::Name::nc        }; // Use default pin (PIO0_25)
constexpr Pin::Name               XARMLIB_CONFIG_FAIM_ISP_UART0_RX_PIN { Pin::Name::nc        }; // Use default pin (PIO0_24)
constexpr Faim::PinConfigArray<0> XARMLIB_CONFIG_FAIM_GPIO_PINS;                                 // Setup all IOs with pull-up by default




// ----------------------------------------------------------------------------
//XARMLIB LIBRARY DEFINITIONS
// ----------------------------------------------------------------------------

// -------- FATFS -------------------------------------------------------------
#define XARMLIB_ENABLE_FATFS                                     (0)    // Enable / disable FatFs functionality

// -------- PIN SCANNER -------------------------------------------------------
#define XARMLIB_ENABLE_PIN_SCANNER                               (0)    // Enable / disable pin scanner functionality

constexpr std::size_t XARMLIB_CONFIG_PIN_SCANNER_SOURCE_COUNT    { 0 }; // Maximum possible number of pin scanner sources
constexpr std::size_t XARMLIB_CONFIG_PIN_SCANNER_DEBOUNCER_COUNT { 0 }; // Maximum possible number of pin scanner debouncers




} // namespace xarmlib

#endif // __XARMLIB_CONFIG_HPP
