// ----------------------------------------------------------------------------
// @file    api_pin_scanner.cpp
// @brief   API pin scanner class (takes control of one available Timer).
// @date    13 July 2018
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

#include "xarmlib_config.hpp"

#if (XARMLIB_ENABLE_PIN_SCANNER == 1)

namespace xarmlib
{




// Static initialization
Timer                                       PinScanner::m_timer;
std::dynarray<PinScanner::PinSourceHandler> PinScanner::m_pin_source_handlers(XARMLIB_CONFIG_PIN_SCANNER_SOURCE_COUNT);
std::dynarray<PinScanner::DebouncerHandler> PinScanner::m_debouncer_handlers(XARMLIB_CONFIG_PIN_SCANNER_DEBOUNCER_COUNT);
PinScanner::PinChangeHandler                PinScanner::m_pin_change_handler;
bool                                        PinScanner::m_is_starting { true };




} // namespace xarmlib

#endif // (XARMLIB_ENABLE_PIN_SCANNER == 1)
