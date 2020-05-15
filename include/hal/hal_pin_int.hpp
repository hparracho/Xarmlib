// ----------------------------------------------------------------------------
// @file    hal_pin_int.hpp
// @brief   Pin Interrupt HAL interface class.
// @date    15 May 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_HAL_PIN_INT_HPP
#define __XARMLIB_HAL_PIN_INT_HPP

#include "hal/hal_pin.hpp"
#include "hal/hal_port.hpp"

namespace xarmlib
{
namespace hal
{




template <typename PinIntDriver>
class PinIntBase : protected PinIntDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using InputMode                    = typename PinIntDriver::InputMode;

        using InputModeConfig              = typename PinIntDriver::InputModeConfig;
#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        using InputModeTrueOpenDrainConfig = typename PinIntDriver::InputModeTrueOpenDrainConfig;
#endif

        using IntMode                      = typename PinIntDriver::IntMode;

        using IrqHandler                   = typename PinIntDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Normal input pin constructor
        PinIntBase(const hal::Pin::Name   pin_name,
                   const InputModeConfig& config,
                   const IntMode          int_mode) : PinIntDriver(pin_name, config, int_mode)
        {}

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        // True open-drain input pin constructor
        PinIntBase(const hal::Pin::Name                pin_name,
                   const InputModeTrueOpenDrainConfig& config,
                   const IntMode                       int_mode) : PinIntDriver(pin_name, config, int_mode)
        {}
#endif

        // -------- CONFIGURATION ---------------------------------------------

        void set_mode(const InputModeConfig& config)              { PinIntDriver::set_mode(config); }
#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        void set_mode(const InputModeTrueOpenDrainConfig& config) { PinIntDriver::set_mode(config); }
#endif

        hal::Pin::Name get_pin_name() const { return PinIntDriver::get_pin_name(); }

        // -------- READ ------------------------------------------------------

        uint32_t read() const { return PinIntDriver::read(); }

        // -------- INTERRUPT -------------------------------------------------

        void enable_interrupt()           { PinIntDriver::enable_interrupt(); }
        void disable_interrupt()          { PinIntDriver::disable_interrupt(); }
        bool is_interrupt_enabled() const { return PinIntDriver::is_interrupt_enabled(); }

        bool is_interrupt_pending() const { return PinIntDriver::is_interrupt_pending(); }
        void clear_interrupt_pending()    { PinIntDriver::clear_interrupt_pending(); }

        // -------- IRQ HANDLER -----------------------------------------------

        void assign_irq_handler(const IrqHandler& irq_handler) { PinIntDriver::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { PinIntDriver::remove_irq_handler(); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_pin_int.hpp"

namespace xarmlib
{
namespace hal
{

using PinInt = PinIntBase<targets::kv5x::PinIntDriver>;

} // namespace hal

class PinInt : public hal::PinInt
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::PinInt;

        using PassiveFilter = typename Hal::PassiveFilter;
        using LockRegister  = typename Hal::LockRegister;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- PORT IRQ --------------------------------------------------

        static void enable_port_irq(const hal::Port::Name port_name)
        {
            Hal::enable_port_irq(port_name);
        }

        static void disable_port_irq(const hal::Port::Name port_name)
        {
            Hal::disable_port_irq(port_name);
        }

        static bool is_port_irq_enabled(const hal::Port::Name port_name)
        {
            return Hal::is_port_irq_enabled(port_name);
        }

        static void set_port_irq_priority(const hal::Port::Name port_name, const int32_t irq_priority)
        {
            Hal::set_port_irq_priority(port_name, irq_priority);
        }
};

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_pin_int.hpp"

namespace xarmlib
{
namespace hal
{

using PinInt = PinIntBase<targets::kv4x::PinIntDriver>;

} // namespace hal

class PinInt : public hal::PinInt
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::PinInt;

        using PassiveFilter = typename Hal::PassiveFilter;
        using LockRegister  = typename Hal::LockRegister;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- PORT IRQ --------------------------------------------------

        static void enable_port_irq(const hal::Port::Name port_name)
        {
            Hal::enable_port_irq(port_name);
        }

        static void disable_port_irq(const hal::Port::Name port_name)
        {
            Hal::disable_port_irq(port_name);
        }

        static bool is_port_irq_enabled(const hal::Port::Name port_name)
        {
            return Hal::is_port_irq_enabled(port_name);
        }

        static void set_port_irq_priority(const hal::Port::Name port_name, const int32_t irq_priority)
        {
            Hal::set_port_irq_priority(port_name, irq_priority);
        }
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using PinInt = PinIntBase<targets::other_target::PinIntDriver>;

} // namespace hal

using PinInt = hal::PinInt;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_PIN_INT_HPP
