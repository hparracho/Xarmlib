// ----------------------------------------------------------------------------
// @file    api_pin_debouncer.cpp
// @brief   API pin debouncer class.
// @date    14 July 2018
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

#if (XARMLIB_ENABLE_INPUT_DEBOUNCER == 1)

namespace xarmlib
{




std::array<PinDebouncer::PortMask, TARGET_PORT_COUNT> PinDebouncer::m_ports {};
std::dynarray<PinDebouncer::Input>                    PinDebouncer::m_pins(XARMLIB_CONFIG_PIN_DEBOUNCER_PIN_COUNT);
std::size_t                                           PinDebouncer::m_assigned_pin_count { 0 };
bool                                                  PinDebouncer::m_is_first_debounce { true };




} // namespace xarmlib

#endif // (XARMLIB_ENABLE_INPUT_DEBOUNCER == 1)
