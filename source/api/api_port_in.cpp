// ----------------------------------------------------------------------------
// @file    api_port_in.cpp
// @brief   API port input class.
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

#include "xarmlib_config.hpp"

namespace xarmlib
{




// Static initialization
dynarray<InputDebouncer::Input>                    PortIn::m_inputs(XARMLIB_CONFIG_PORT_IN_INPUTS_COUNT, { 0, 0, 0, 0, 0 });
std::size_t                                        PortIn::m_inputs_count { 0 };

std::array<InputDebouncer::InputPort, Port::COUNT> PortIn::m_input_ports { 0, 0, 0, 0, 0 };

std::array<uint32_t, Port::COUNT>                  PortIn::m_input_ports_mask { 0 };




} // namespace xarmlib
