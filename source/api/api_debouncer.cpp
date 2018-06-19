// ----------------------------------------------------------------------------
// @file    api_debouncer.cpp
// @brief   API debouncer class (takes control of one available Timer).
// @date    19 June 2018
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

#include "api/api_debouncer.hpp"
#include "xarmlib_config.hpp"

namespace xarmlib
{




// Static initializations
Timer Debouncer::m_timer;

//@TODO: APAGAR !!
Gpio Debouncer::temp_m_led_red(Pin::Name::P0_12, Gpio::OutputMode::PUSH_PULL_HIGH);

std::array<private_debouncer::Input, XARMLIB_CONFIG_DEBOUNCER_INPUT_COUNT_MAX> g_input { /*{ Pin::Name::NC, 0, 0, 0 }*/ };
gsl::span<private_debouncer::Input> Debouncer::m_input { g_input };

std::ptrdiff_t Debouncer::m_input_count { 0 };

std::array<uint32_t, Port::COUNT> Debouncer::m_pins_mask { 0 };

std::array<uint32_t, Port::COUNT> Debouncer::m_last_read_pins { 0 };

std::array<uint32_t, Port::COUNT> Debouncer::m_current_debounced { 0 };

std::array<uint32_t, Port::COUNT> Debouncer::m_last_debounced { 0 };

std::array<uint32_t, Port::COUNT> Debouncer::m_sampling { 0 };

Debouncer::NewDebouncedHandler Debouncer::m_new_debounced_handler;




} // namespace xarmlib
