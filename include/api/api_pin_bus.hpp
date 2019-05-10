// ----------------------------------------------------------------------------
// @file    api_pin_bus.hpp
// @brief   API pin bus class.
// @date    10 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_API_PIN_BUS_HPP
#define __XARMLIB_API_PIN_BUS_HPP

#include "hal/hal_pin.hpp"

#include <type_traits>

namespace xarmlib
{




template <class Type, class Enable = void>
class PinBus;

template <class Type>
class PinBus<Type, typename std::enable_if<std::is_same<Type, hal::Pin::Name>::value == true
                                        || std::is_same<Type, int8_t        >::value == true>::type>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        PinBus() = delete;

        constexpr PinBus(const std::initializer_list<Type> pin_list) : m_pin_list (pin_list)
        {
#ifndef NDEBUG
            assert(pin_list.size() <= 32);

            for(const auto pin : pin_list)
            {
                if constexpr(std::is_same<Type, hal::Pin::Name>::value == true)
                {
                    assert(pin != hal::Pin::Name::nc);
                }

                if constexpr(std::is_same<Type, int8_t>::value == true)
                {
                    assert(pin >= 0);
                }
            }
#endif
        }

        constexpr std::size_t get_size() const
        {
            return m_pin_list.size();
        }

        constexpr uint32_t get_mask() const
        {
            return static_cast<uint32_t>((1UL << get_size()) - 1);
        }

        constexpr const Type* begin() const
        {
            return m_pin_list.begin();
        }

        constexpr const Type* end() const
        {
            return m_pin_list.end();
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const std::initializer_list<Type> m_pin_list;
};




using PinNameBus  = PinBus<hal::Pin::Name>;
using PinIndexBus = PinBus<int8_t>;




#if 0 // DEPRECATED 20180703
template <Pin::Name... pins>
class PinBus
{
    public:

        constexpr PinBus() : m_pin_names { pins... }
        {}

        constexpr std::size_t get_count() const
        {
            return sizeof...(pins);
        }

        constexpr const std::array<Pin::Name, sizeof...(pins)>& get_array() const
        {
            return m_pin_names;
        }

        constexpr Pin::Name get_pin_name(const std::size_t index) const
        {
            assert(index < get_count());

            return m_pin_names[index];
        }

    private:

        const std::array<Pin::Name, sizeof...(pins)> m_pin_names;
};
#endif // DEPRECATED 20180703




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_BUS_HPP
