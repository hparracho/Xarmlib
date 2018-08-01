// ----------------------------------------------------------------------------
// @file    api_push_button_bus.hpp
// @brief   API push-button bus class.
// @date    1 August 2018
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




struct PushButtonData
{
    int16_t index   { -1 };
    bool    coded   { false };
    bool    enabled { false };
};




template <class Type, class Enable = void>
class PushButtonBus;

template <class Type>
class PushButtonBus<Type, typename std::enable_if<std::is_same<Type, Pin::Name>::value == true
                                               || std::is_same<Type, int8_t   >::value == true>::type>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        PushButtonBus() = delete;

        constexpr PushButtonBus(const std::initializer_list<Type>           pin_list,
                                const std::initializer_list<PushButtonData> data_list) : m_pin_list(pin_list),
                                                                                         m_data_list(data_list),
                                                                                         m_list_count { pin_list.size() }
        {
#ifndef NDEBUG
            assert(pin_list.size() == data_list.size());
            assert(pin_list.size() <= 32);

            for(const auto pin : pin_list)
            {
                if constexpr(std::is_same<Type, Pin::Name>::value == true)
                {
                    assert(pin != Pin::Name::NC);
                }

                if constexpr(std::is_same<Type, int8_t>::value == true)
                {
                    assert(pin >= 0);
                }
            }

            for(const auto data : data_list)
            {
                assert(data.index > 0);
            }
#endif
        }

        constexpr PushButtonBus(const std::size_t count) : m_pin_list(count),
                                                           m_data_list(count),
                                                           m_list_count { 0 }
        {
            assert(count <= 32);
        }

        // -------- ASSIGNMENT FUNCTION ---------------------------------------

        constexpr void append(const Type*           pin_list_begin,  const Type*           pin_list_end,
                              const PushButtonData* data_list_begin, const PushButtonData* data_list_end)
        {
            const auto pin_list_size = std::distance(pin_list_begin, pin_list_end);

#ifndef NDEBUG
            assert(pin_list_size == std::distance(data_list_begin, data_list_end));
            assert(m_list_count + pin_list_size <= m_pin_list.size());

            for(auto i = 0; i < pin_list_size; ++i)
            {
                if constexpr(std::is_same<Type, Pin::Name>::value == true)
                {
                    assert(pin_list_begin[i] != Pin::Name::NC);
                }

                if constexpr(std::is_same<Type, int8_t>::value == true)
                {
                    assert(pin_list_begin[i] >= 0);
                }

                assert(data_list_begin[i].index > 0);
            }
#endif

            std::copy(pin_list_begin,  pin_list_end,  &m_pin_list[m_list_count]);
            std::copy(data_list_begin, data_list_end, &m_data_list[m_list_count]);

            m_list_count += pin_list_size;
        }

        // -------- DATA ACCESS FUNCTIONS / OPERATORS -------------------------

        constexpr const PushButtonData* get_data_list() const
        {
            return m_data_list.data();
        }

        PushButtonData& operator [] (const std::size_t pos)
        {
            assert(pos < m_list_count);

            return m_data_list[pos];
        }

        constexpr const PushButtonData& operator [] (const std::size_t pos) const
        {
            assert(pos < m_list_count);

            return m_data_list[pos];
        }

        // -------- CAPACITY FUNCTION -----------------------------------------

        constexpr std::size_t get_size() const
        {
            return m_list_count;
        }

        // -------- MASK FUNCTION ---------------------------------------------

        constexpr uint32_t get_mask() const
        {
            assert(m_list_count > 0);

            return static_cast<uint32_t>((1UL << get_size()) - 1);
        }

        // -------- PIN ITERATORS ---------------------------------------------

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

        std::dynarray<Type>           m_pin_list;
        std::dynarray<PushButtonData> m_data_list;
        std::size_t                   m_list_count;
};




using PushButtonNameBus  = PushButtonBus<Pin::Name>;
using PushButtonIndexBus = PushButtonBus<int8_t>;




} // namespace xarmlib

#endif // __XARMLIB_API_PUSH_BUTTON_BUS_HPP
