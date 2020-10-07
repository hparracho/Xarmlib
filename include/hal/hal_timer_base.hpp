// ----------------------------------------------------------------------------
// @file    hal_timer_base.hpp
// @brief   32-bit Timer HAL interface class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_HAL_TIMER_BASE_HPP
#define XARMLIB_HAL_TIMER_BASE_HPP

#include "core/chrono.hpp"
#include "core/target_specs.hpp"

#if (TARGET_HAS_MRT_TIMER == 1)
#include "hal/hal_peripheral_shared_irq.hpp"
namespace xarmlib::hal
{
template <typename Driver>
using TimerPeripheral = PeripheralSharedIrq<Driver, TARGET_TIMER_COUNT>;
}
#else
#include "hal/hal_peripheral_irq.hpp"
namespace xarmlib::hal
{
template <typename Driver>
using TimerPeripheral = PeripheralIrq<Driver, TARGET_TIMER_COUNT>;
}
#endif




namespace xarmlib::hal
{

template <typename Driver>
class TimerBase : public TimerPeripheral<Driver>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Timer running mode selection
    enum class Mode : uint32_t
    {
        free_running = (1UL << 0),  // Re-start at the end of counter
        single_shot  = (1UL << 1)   // Stop at the end of counter
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    void start(const chrono::microseconds_u32& rate_us, const Mode mode = Mode::free_running)
    {
        static_cast<Driver*>(this)->start(rate_us, mode);
    }

    void stop()
    {
        static_cast<Driver*>(this)->stop();
    }

    void reload()
    {
        static_cast<Driver*>(this)->reload();
    }

    bool is_running() const
    {
        return static_cast<const Driver*>(this)->is_running();
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    TimerBase(Driver& driver_ref) : TimerPeripheral<Driver>(driver_ref)
    {}
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_TIMER_BASE_HPP
