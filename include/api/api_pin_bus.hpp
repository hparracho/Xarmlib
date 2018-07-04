// ----------------------------------------------------------------------------
// @file    api_pin_bus.hpp
// @brief   API pin bus class.
// @date    21 June 2018
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

#ifndef __XARMLIB_API_PIN_BUS_HPP
#define __XARMLIB_API_PIN_BUS_HPP

#include "system/array"
#include "hal/hal_pin.hpp"

namespace xarmlib
{



/*
class PinBus
{
    public:

        constexpr PinBus(std::initializer_list<Pin::Name> pin_names) : m_pin_names { pin_names }
        {}

        constexpr std::size_t get_width() const
        {
            return m_pin_names.size();
        }

        constexpr const Pin::Name* begin() const
        {
            return m_pin_names.begin();
        }

        constexpr const Pin::Name* end() const
        {
            return m_pin_names.end();
        }

        constexpr Pin::Name get_pin_name(const std::size_t index) const
        {
            assert(index < get_width());

            return m_pin_names[index];
        }

    private:

        const std::initializer_list<Pin::Name> m_pin_names;
};
*/




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




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_BUS_HPP
