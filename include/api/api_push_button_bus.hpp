// ----------------------------------------------------------------------------
// @file    api_push_button_bus.hpp
// @brief   API push-button bus class.
// @date    4 October 2018
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

#ifndef __XARMLIB_API_PUSH_BUTTON_BUS_HPP
#define __XARMLIB_API_PUSH_BUTTON_BUS_HPP

#include "hal/hal_pin.hpp"

#include <type_traits>

namespace xarmlib
{




template <class Type, class Enable = void>
class PushButtonBus;

template <class Type>
class PushButtonBus<Type, typename std::enable_if<std::is_same<Type, Pin::Name>::value == true
                                               || std::is_same<Type, int8_t   >::value == true>::type>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        struct Data
        {
            Type     pin;
            uint8_t  binary1 { 0 };
            uint32_t code    { 0 }; // NOTES: - 0 means no code
                                    //        - code is based on 4 binary1 digits (4 bytes): digit4 | digit3 | digit2 | digit1
            bool     enabled { false };
            bool     stage2_selected { false };
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        PushButtonBus() = delete;

        constexpr PushButtonBus(const std::size_t count) : m_push_button_list(count),
                                                           m_push_button_list_size { 0 }
        {
            assert(count <= 32);
        }

        // -------- APPEND PUSH-BUTTON FUNCTION -------------------------------

        constexpr void append(const Data data)
        {
#ifndef NDEBUG
            assert(m_push_button_list_size + 1 <= m_push_button_list.size());

            if constexpr(std::is_same<Type, Pin::Name>::value == true)
            {
                assert(data.pin != Pin::Name::NC);
            }

            if constexpr(std::is_same<Type, int8_t>::value == true)
            {
                assert(data.pin >= 0);
            }

            assert(data.binary1 > 0);
#endif

            m_push_button_list[m_push_button_list_size] = data;

            m_push_button_list_size++;
        }

        // -------- GET/SET DATA MEMBERS FUNCTIONS ----------------------------

        constexpr uint8_t get_binary1(const std::size_t index) const
        {
            assert(index < m_push_button_list_size);

            return m_push_button_list[index].binary1;
        }

        constexpr void set_binary1(const std::size_t index, const uint8_t binary1)
        {
            assert(index < m_push_button_list_size);
            assert(binary1 > 0);

            m_push_button_list[index].binary1 = binary1;
        }

        constexpr uint32_t get_code(const std::size_t index) const
        {
            assert(index < m_push_button_list_size);

            return m_push_button_list[index].code;
        }

        constexpr void set_code(const std::size_t index, const uint32_t code)
        {
            assert(index < m_push_button_list_size);

            m_push_button_list[index].code = code;
        }

        constexpr bool get_enabled(const std::size_t index) const
        {
            assert(index < m_push_button_list_size);

            return m_push_button_list[index].enabled;
        }

        constexpr void set_enabled(const std::size_t index, const bool enabled)
        {
            assert(index < m_push_button_list_size);

            m_push_button_list[index].enabled = enabled;
        }

        constexpr bool get_stage2_selected(const std::size_t index) const
        {
            assert(index < m_push_button_list_size);

            return m_push_button_list[index].stage2_selected;
        }

        constexpr void set_stage2_selected(const std::size_t index, const bool stage2_selected)
        {
            assert(index < m_push_button_list_size);

            m_push_button_list[index].stage2_selected = stage2_selected;
        }

        // -------- CAPACITY FUNCTION -----------------------------------------

        constexpr bool is_empty() const
        {
            return (m_push_button_list_size == 0);
        }

        constexpr std::size_t get_size() const
        {
            return m_push_button_list_size;
        }

        // -------- MASK FUNCTION ---------------------------------------------

        constexpr uint32_t get_mask() const
        {
            return static_cast<uint32_t>((1UL << get_size()) - 1);
        }

        // -------- ITERATORS -------------------------------------------------

        constexpr const Data* begin() const
        {
            return m_push_button_list.begin();
        }

        constexpr const Data* end() const
        {
            return m_push_button_list.begin() + get_size();
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::dynarray<Data> m_push_button_list;
        std::size_t         m_push_button_list_size;
};




using PushButtonNameBus  = PushButtonBus<Pin::Name>;
using PushButtonIndexBus = PushButtonBus<int8_t>;




} // namespace xarmlib

#endif // __XARMLIB_API_PUSH_BUTTON_BUS_HPP
