// ---------------------------------------------------------------------------
// @file    peripheral_ref_counter.hpp
// @brief   Peripheral reference counter class (helper class to keep a record
//          of the peripherals objects that are created and destructed).
// @date    16 April 2019
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

#ifndef __XARMLIB_CORE_PERIPHERAL_REF_COUNTER_HPP
#define __XARMLIB_CORE_PERIPHERAL_REF_COUNTER_HPP

#include "core/non_copyable.hpp"

#include <array>
#include <cassert>

namespace xarmlib
{




template<class Peripheral, std::size_t PERIPHERAL_COUNT, uint32_t FIXED_MASK = 0>
class PeripheralRefCounter : private NonCopyable<PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT, FIXED_MASK>>
{
    protected:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        explicit PeripheralRefCounter(Peripheral& peripheral, int32_t index = -1)
        {
            index = get_available_index(m_used_mask, index);
            assert(index >= 0);

            m_index = static_cast<std::size_t>(index);

            m_used_mask |= (1 << m_index);
            m_peripherals[m_index] = &peripheral;
        }

        ~PeripheralRefCounter()
        {
            m_used_mask &= ~(1 << m_index);
            m_peripherals[m_index] = nullptr;
        }

        std::size_t get_index() const
        {
            return m_index;
        }

        static Peripheral& get_reference(const std::size_t index)
        {
            assert(index < PERIPHERAL_COUNT && m_peripherals[index] != nullptr);

            return *m_peripherals[index];
        }

        static Peripheral* get_pointer(const std::size_t index)
        {
            assert(index < PERIPHERAL_COUNT);

            return m_peripherals[index];
        }

        static int32_t get_used()
        {
            // Count the number of bits set
            return __builtin_popcount(m_used_mask);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr int32_t get_available_index(const uint32_t used_mask, const int32_t index)
        {
            if(index <= -1)
            {
                for(std::size_t p = 0; p < PERIPHERAL_COUNT; ++p)
                {
                    if((used_mask & (1 << p)) == 0 && (FIXED_MASK & (1 << p)) == 0)
                    {
                        return p;
                    }
                }
            }
            else
            {
                if(static_cast<std::size_t>(index) < PERIPHERAL_COUNT && (used_mask & (1 << index)) == 0 && (FIXED_MASK & (1 << index)) != 0)
                {
                    return index;
                }
            }

            return -1;
        }

        // --------------------------------------------------------------------
        // PRIVATE VARIABLES
        // --------------------------------------------------------------------

        std::size_t m_index;

        static std::array<Peripheral*, PERIPHERAL_COUNT> m_peripherals;
        static uint32_t                                  m_used_mask;
};




// Static initialization
template <class Peripheral, std::size_t PERIPHERAL_COUNT, uint32_t FIXED_MASK>
std::array<Peripheral*, PERIPHERAL_COUNT> PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT, FIXED_MASK>::m_peripherals { nullptr };

template <class Peripheral, std::size_t PERIPHERAL_COUNT, uint32_t FIXED_MASK>
uint32_t PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT, FIXED_MASK>::m_used_mask { 0 };




} // namespace xarmlib

#endif // __XARMLIB_CORE_PERIPHERAL_REF_COUNTER_HPP
