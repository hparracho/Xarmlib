// ----------------------------------------------------------------------------
// @file    api_pin_bus_debounced.hpp
// @brief   API pin bus debounced class.
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

#ifndef __XARMLIB_API_PIN_BUS_DEBOUNCED_HPP
#define __XARMLIB_API_PIN_BUS_DEBOUNCED_HPP

#include "api/api_pin_bus.hpp"

#include <type_traits>

namespace xarmlib
{




template <Pin::Name... pins>
class PinBusDebounced : private PinBus<pins...>
{
        using Type = typename std::conditional<sizeof...(pins) <= 32, uint32_t, uint64_t>::type;

    public:

        Type read() const
        {
            return m_value;
        }

        operator Type () const
        {
            return read();
        }

        Type operator ! () const
        {
            return !read();
        }

        constexpr std::size_t get_width() const
        {
            return sizeof...(pins);
        }

        constexpr Type get_mask() const
        {
            Type mask = 0;

            for(std::size_t bit = 0; bit < get_width(); bit++)
            {
                mask |= static_cast<Type>(1) << bit;
            }

            return mask;
        }

    private:

        friend class PortIn;

        Type m_value { 0 };
};




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_BUS_DEBOUNCED_HPP
