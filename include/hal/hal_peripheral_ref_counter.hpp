// ---------------------------------------------------------------------------
// @file    peripheral_ref_counter.hpp
// @brief   Peripheral reference counter class (helper class to keep a record
//          of the peripherals objects that are created and destructed).
// @date    8 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_HAL_PERIPHERAL_REF_COUNTER_HPP
#define XARMLIB_HAL_PERIPHERAL_REF_COUNTER_HPP

#include "xarmlib_config.hpp"

#include "core/non_copyable.hpp"

#include <array>
#include <cassert>




namespace xarmlib::hal
{

template<class Peripheral, std::size_t PeripheralCount, uint32_t FixedMask = 0>
class PeripheralRefCounter : NonCopyable<PeripheralRefCounter<Peripheral, PeripheralCount, FixedMask>>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    explicit PeripheralRefCounter(Peripheral& peripheral, int32_t index = -1)
    {
        index = get_available_index(s_used_mask, index);
        assert(index >= 0);

        m_index = static_cast<std::size_t>(index);

        s_used_mask |= (1 << m_index);
        s_peripherals[m_index] = &peripheral;
    }

#if (XARMLIB_DISABLE_EXPENSIVE_PERIPHERAL_DESTRUCTORS != 1)
    ~PeripheralRefCounter()
    {
        s_used_mask &= ~(1 << m_index);
        s_peripherals[m_index] = nullptr;
    }
#endif

    std::size_t get_this_index() const
    {
        return m_index;
    }

    static Peripheral& get_reference(const std::size_t index)
    {
        assert(index < PeripheralCount && s_peripherals[index] != nullptr);

        return *s_peripherals[index];
    }

    static Peripheral* get_pointer(const std::size_t index)
    {
        assert(index < PeripheralCount);

        return s_peripherals[index];
    }

    static int32_t get_use_count()
    {
        // Count the number of bits set
        return __builtin_popcount(s_used_mask);
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static constexpr int32_t get_available_index(const uint32_t used_mask, const int32_t index)
    {
        if(index <= -1)
        {
            for(std::size_t p = 0; p < PeripheralCount; ++p)
            {
                if((used_mask & (1 << p)) == 0 && (FixedMask & (1 << p)) == 0)
                {
                    return p;
                }
            }
        }
        else
        {
            if(static_cast<std::size_t>(index) < PeripheralCount
            && (used_mask & (1 << index)) == 0 && (FixedMask & (1 << index)) != 0)
            {
                return index;
            }
        }

        return -1;
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    std::size_t m_index;

    inline static std::array<Peripheral*, PeripheralCount> s_peripherals {nullptr};
    inline static uint32_t                                 s_used_mask   {0};
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PERIPHERAL_REF_COUNTER_HPP
