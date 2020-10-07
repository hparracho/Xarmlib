// ----------------------------------------------------------------------------
// @file    hal_enc.hpp
// @brief   Quadrature Encoder/Decoder (ENC) HAL interface class.
// @date    20 September 2020
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

#ifndef __XARMLIB_HAL_ENC_HPP
#define __XARMLIB_HAL_ENC_HPP

#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <typename EncDriver>
class EncBase : protected EncDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Config = typename EncDriver::Config;

        using Status        = typename EncDriver::Status;
        using StatusBitmask = typename EncDriver::StatusBitmask;

        using SignalStatus        = typename EncDriver::SignalStatus;
        using SignalStatusBitmask = typename EncDriver::SignalStatusBitmask;

        using HomeInterrupt        = typename EncDriver::HomeInterrupt;
        using HomeInterruptBitmask = typename EncDriver::HomeInterruptBitmask;

        using IndexInterrupt        = typename EncDriver::IndexInterrupt;
        using IndexInterruptBitmask = typename EncDriver::IndexInterruptBitmask;

        using WatchdogAndSabInterrupt        = typename EncDriver::WatchdogAndSabInterrupt;
        using WatchdogAndSabInterruptBitmask = typename EncDriver::WatchdogAndSabInterruptBitmask;

        using CompareInterrupt        = typename EncDriver::CompareInterrupt;
        using CompareInterruptBitmask = typename EncDriver::CompareInterruptBitmask;

        using IrqHandler = typename EncDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        EncBase(const hal::Pin::Name input_phase_a,
                const hal::Pin::Name input_phase_b,
                const Config&        config,
                const hal::Pin::Name input_index           = hal::Pin::Name::nc,
                const hal::Pin::Name input_home            = hal::Pin::Name::nc,
                const hal::Pin::Name input_trigger         = hal::Pin::Name::nc,
                const hal::Pin::Name output_position_match = hal::Pin::Name::nc) : EncDriver(input_phase_a,
                                                                                             input_phase_b,
                                                                                             config,
                                                                                             input_index,
                                                                                             input_home,
                                                                                             input_trigger,
                                                                                             output_position_match)
        {}

        // -------- ENABLE / DISABLE WATCHDOG ---------------------------------

        void enable_watchdog()          { EncDriver::enable_watchdog(); }
        void disable_watchdog()         { EncDriver::disable_watchdog(); }
        bool is_enable_watchdog() const { return EncDriver::is_enable_watchdog(); }

        // -------- SOFTWARE INITIALIZE POSITION COUNTER ----------------------

        uint32_t get_initial_position() const               { return EncDriver::get_initial_position(); }
        void     set_initial_position(const uint32_t value) { EncDriver::set_initial_position(value); }

        void software_initialize_position_counter() { EncDriver::software_initialize_position_counter(); }

        // -------- POSITION / REVOLUTION -------------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        uint32_t get_position() const               { return EncDriver::get_position(); }
        void     set_position(const uint32_t value) { EncDriver::set_position(value); }

        uint16_t get_position_difference() const               { return EncDriver::get_position_difference(); }
        void     set_position_difference(const uint16_t value) { EncDriver::set_position_difference(value); }

        uint16_t get_revolution() const               { return EncDriver::get_revolution(); }
        void     set_revolution(const uint16_t value) { EncDriver::set_revolution(value); }

        // -------- HOLD POSITION / REVOLUTION --------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        uint32_t get_hold_position() const            { return EncDriver::get_hold_position(); }
        uint16_t get_hold_position_difference() const { return EncDriver::get_hold_position_difference(); }
        uint16_t get_hold_revolution() const          { return EncDriver::get_hold_revolution(); }

        // -------- COMPARE POSITION ------------------------------------------

        uint32_t get_compare_position() const               { return EncDriver::get_compare_position(); }
        void     set_compare_position(const uint32_t value) { EncDriver::set_compare_position(value); }

        // -------- STATUS FLAGS ----------------------------------------------

        StatusBitmask get_status() const                        { return EncDriver::get_status(); }
        void          clear_status(const StatusBitmask bitmask) { EncDriver::clear_status(bitmask); }

        // -------- SIGNAL STATUS FLAGS ---------------------------------------

        SignalStatusBitmask get_signal_status() const { return EncDriver::get_signal_status(); }

        // -------- HOME INTERRUPTS -------------------------------------------

        void                 enable_home_interrupts (const HomeInterruptBitmask bitmask) { EncDriver::enable_home_interrupts(bitmask); }
        void                 disable_home_interrupts(const HomeInterruptBitmask bitmask) { EncDriver::disable_home_interrupts(bitmask); }
        HomeInterruptBitmask get_home_interrupts_enabled() const                         { return EncDriver::get_home_interrupts_enabled(); }

        // -------- INDEX INTERRUPTS ------------------------------------------

        void                  enable_index_interrupts (const IndexInterruptBitmask bitmask) { EncDriver::enable_index_interrupts(bitmask); }
        void                  disable_index_interrupts(const IndexInterruptBitmask bitmask) { EncDriver::disable_index_interrupts(bitmask); }
        IndexInterruptBitmask get_index_interrupts_enabled() const                          { return EncDriver::get_index_interrupts_enabled(); }

        // -------- WATCHDOG AND SAB INTERRUPTS -------------------------------

        void                           enable_watchdog_and_sab_interrupts (const WatchdogAndSabInterruptBitmask bitmask) { EncDriver::enable_watchdog_and_sab_interrupts(bitmask); }
        void                           disable_watchdog_and_sab_interrupts(const WatchdogAndSabInterruptBitmask bitmask) { EncDriver::disable_watchdog_and_sab_interrupts(bitmask); }
        WatchdogAndSabInterruptBitmask get_watchdog_and_sab_interrupts_enabled() const                                   { return EncDriver::get_watchdog_and_sab_interrupts_enabled(); }

        // -------- COMPARE INTERRUPTS ----------------------------------------

        void                    enable_compare_interrupts (const CompareInterruptBitmask bitmask) { EncDriver::enable_compare_interrupts(bitmask); }
        void                    disable_compare_interrupts(const CompareInterruptBitmask bitmask) { EncDriver::disable_compare_interrupts(bitmask); }
        CompareInterruptBitmask get_compare_interrupts_enabled() const                            { return EncDriver::get_compare_interrupts_enabled(); }

        // -------- HOME IRQ / IRQ HANDLER ------------------------------------

        void enable_home_irq()           { EncDriver::enable_home_irq(); }
        void disable_home_irq()          { EncDriver::disable_home_irq(); }
        bool is_home_irq_enabled() const { return EncDriver::is_home_irq_enabled(); }

        void set_home_irq_priority(const int32_t irq_priority) { EncDriver::set_home_irq_priority(irq_priority); }

        void assign_home_irq_handler(const IrqHandler& irq_handler) { EncDriver::assign_home_irq_handler(irq_handler); }
        void remove_home_irq_handler()                              { EncDriver::remove_home_irq_handler(); }

        // -------- INDEX IRQ / IRQ HANDLER -----------------------------------

        void enable_index_irq()           { EncDriver::enable_index_irq(); }
        void disable_index_irq()          { EncDriver::disable_index_irq(); }
        bool is_index_irq_enabled() const { return EncDriver::is_index_irq_enabled(); }

        void set_index_irq_priority(const int32_t irq_priority) { EncDriver::set_index_irq_priority(irq_priority); }

        void assign_index_irq_handler(const IrqHandler& irq_handler) { EncDriver::assign_index_irq_handler(irq_handler); }
        void remove_index_irq_handler()                              { EncDriver::remove_index_irq_handler(); }

        // -------- WATCHDOG AND SAB IRQ / IRQ HANDLER ------------------------

        void enable_watchdog_and_sab_irq()           { EncDriver::enable_watchdog_and_sab_irq(); }
        void disable_watchdog_and_sab_irq()          { EncDriver::disable_watchdog_and_sab_irq(); }
        bool is_watchdog_and_sab_irq_enabled() const { return EncDriver::is_watchdog_and_sab_irq_enabled(); }

        void set_watchdog_and_sab_irq_priority(const int32_t irq_priority) { EncDriver::set_watchdog_and_sab_irq_priority(irq_priority); }

        void assign_watchdog_and_sab_irq_handler(const IrqHandler& irq_handler) { EncDriver::assign_watchdog_and_sab_irq_handler(irq_handler); }
        void remove_watchdog_and_sab_irq_handler()                              { EncDriver::remove_watchdog_and_sab_irq_handler(); }

        // -------- COMPARE IRQ / IRQ HANDLER ---------------------------------

        void enable_compare_irq()           { EncDriver::enable_compare_irq(); }
        void disable_compare_irq()          { EncDriver::disable_compare_irq(); }
        bool is_compare_irq_enabled() const { return EncDriver::is_compare_irq_enabled(); }

        void set_compare_irq_priority(const int32_t irq_priority) { EncDriver::set_compare_irq_priority(irq_priority); }

        void assign_compare_irq_handler(const IrqHandler& irq_handler) { EncDriver::assign_compare_irq_handler(irq_handler); }
        void remove_compare_irq_handler()                              { EncDriver::remove_compare_irq_handler(); }
};



} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_enc.hpp"

namespace xarmlib
{
namespace hal
{

using Enc = EncBase<targets::kv5x::EncDriver>;

} // namespace hal

class Enc : public hal::Enc
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Enc;

        using HomeTriggerMode          = typename Hal::HomeTriggerMode;
        using DirectionCounting        = typename Hal::DirectionCounting;
        using PhaseCountMode           = typename Hal::PhaseCountMode;
        using IndexTriggerMode         = typename Hal::IndexTriggerMode;
        using Watchdog                 = typename Hal::Watchdog;
        using InputFilterSampleCount   = typename Hal::InputFilterSampleCount;
        using OutputControl            = typename Hal::OutputControl;
        using RevolutionCounterModulus = typename Hal::RevolutionCounterModulus;
        using ModulusCounting          = typename Hal::ModulusCounting;
        using UpdatePositionRegisters  = typename Hal::UpdatePositionRegisters;
        using UpdateHoldRegisters      = typename Hal::UpdateHoldRegisters;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_enc.hpp"

namespace xarmlib
{
namespace hal
{

using Enc = EncBase<targets::kv4x::EncDriver>;

} // namespace hal

class Enc : public hal::Enc
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Enc;

        using HomeTriggerMode          = typename Hal::HomeTriggerMode;
        using DirectionCounting        = typename Hal::DirectionCounting;
        using PhaseCountMode           = typename Hal::PhaseCountMode;
        using IndexTriggerMode         = typename Hal::IndexTriggerMode;
        using Watchdog                 = typename Hal::Watchdog;
        using InputFilterSampleCount   = typename Hal::InputFilterSampleCount;
        using OutputControl            = typename Hal::OutputControl;
        using RevolutionCounterModulus = typename Hal::RevolutionCounterModulus;
        using ModulusCounting          = typename Hal::ModulusCounting;
        using UpdatePositionRegisters  = typename Hal::UpdatePositionRegisters;
        using UpdateHoldRegisters      = typename Hal::UpdateHoldRegisters;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __OTHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Enc = EncBase<targets::other_target::EncDriver>;

} // namespace hal

using Enc = hal::Enc;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_ENC_HPP
