// ---------------------------------------------------------------------------
// @file    peripheral_ref_counter.hpp
// @brief   Peripheral reference counter class (helper class to keep a record
//          of the peripherals objects that are created and destructed).
// @date    27 June 2018
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

#ifndef __XARMLIB_TARGETS_PERIPHERAL_REF_COUNTER_HPP
#define __XARMLIB_TARGETS_PERIPHERAL_REF_COUNTER_HPP

#include "system/array"
#include "system/cassert"
#include "system/non_copyable"

namespace xarmlib
{
namespace targets
{




template<class Peripheral, std::size_t PERIPHERAL_COUNT>
class PeripheralRefCounter : private NonCopyable<PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT>>
{
    protected:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        explicit PeripheralRefCounter(Peripheral& peripheral)
        {
            const int32_t index = get_available_index(m_used_mask);
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

        static constexpr int32_t get_available_index(const uint32_t used_mask)
        {
            for(std::size_t p = 0; p < PERIPHERAL_COUNT; ++p)
            {
                if((used_mask & (1 << p)) == 0)
                {
                    return p;
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
template <class Peripheral, std::size_t PERIPHERAL_COUNT>
std::array<Peripheral*, PERIPHERAL_COUNT> PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT>::m_peripherals { nullptr };

template <class Peripheral, std::size_t PERIPHERAL_COUNT>
uint32_t PeripheralRefCounter<Peripheral, PERIPHERAL_COUNT>::m_used_mask { 0 };




} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_PERIPHERAL_REF_COUNTER_HPP
