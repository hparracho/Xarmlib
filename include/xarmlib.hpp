// ----------------------------------------------------------------------------
// @file    xarmlib.hpp
// @brief   Xarmlib main header file.
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

#ifndef __XARMLIB_HPP
#define __XARMLIB_HPP

// HAL interface to peripherals
#include "hal/hal_faim.hpp"
#include "hal/hal_gpio.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_port.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_system.hpp"
#include "hal/hal_timer.hpp"
#include "hal/hal_us_ticker.hpp"
//#include "hal/hal_usart.hpp"
#include "hal/hal_watchdog.hpp"

// API interface
#include "api/api_crc.hpp"
#include "api/api_debouncer.hpp"
#include "api/api_digital_in.hpp"
#include "api/api_digital_out.hpp"




#endif // __XARMLIB_HPP
