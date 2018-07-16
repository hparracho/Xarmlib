// ----------------------------------------------------------------------------
// @file    api_pin_source.hpp
// @brief   API pin source base class.
// @date    16 July 2018
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

#ifndef __XARMLIB_API_PIN_SOURCE_HPP
#define __XARMLIB_API_PIN_SOURCE_HPP

#include "api/api_pin_scanner.hpp"

namespace xarmlib
{




class PinSource
{
    public:

        virtual ~PinSource()
        {}

        // Get the handler that is intended to be used as a pin source reader handler of the PinScanner class
        virtual PinScanner::PinSourceHandler get_pin_source_handler() = 0;

        virtual std::size_t get_port_count() const = 0;

        virtual uint32_t get_current_read(const std::size_t port_index) const = 0;

        virtual uint32_t get_current_read_bit(const std::size_t port_index, const std::size_t pin_bit) const = 0;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_SOURCE_HPP
