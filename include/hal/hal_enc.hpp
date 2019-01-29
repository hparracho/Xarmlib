// ----------------------------------------------------------------------------
// @file    hal_enc.hpp
// @brief   Quadrature Encoder/Decoder (ENC) HAL interface class.
// @date    28 January 2019
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

#ifndef __XARMLIB_HAL_ENC_HPP
#define __XARMLIB_HAL_ENC_HPP

#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <typename TargetEncDriver>
class EncHal : protected TargetEncDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Config = typename TargetEncDriver::Config;

        using Status        = typename TargetEncDriver::Status;
        using StatusBitmask = typename TargetEncDriver::StatusBitmask;

        using SignalStatus        = typename TargetEncDriver::SignalStatus;
        using SignalStatusBitmask = typename TargetEncDriver::SignalStatusBitmask;

        using HomeInterrupt        = typename TargetEncDriver::HomeInterrupt;
        using HomeInterruptBitmask = typename TargetEncDriver::HomeInterruptBitmask;

        using IndexInterrupt        = typename TargetEncDriver::IndexInterrupt;
        using IndexInterruptBitmask = typename TargetEncDriver::IndexInterruptBitmask;

        using WatchdogAndSabInterrupt        = typename TargetEncDriver::WatchdogAndSabInterrupt;
        using WatchdogAndSabInterruptBitmask = typename TargetEncDriver::WatchdogAndSabInterruptBitmask;

        using CompareInterrupt        = typename TargetEncDriver::CompareInterrupt;
        using CompareInterruptBitmask = typename TargetEncDriver::CompareInterruptBitmask;

        using IrqHandler = typename TargetEncDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        EncHal(const xarmlib::PinHal::Name input_phase_a,
               const xarmlib::PinHal::Name input_phase_b,
               const Config&               config,
               const xarmlib::PinHal::Name input_index           = xarmlib::PinHal::Name::nc,
               const xarmlib::PinHal::Name input_home            = xarmlib::PinHal::Name::nc,
               const xarmlib::PinHal::Name input_trigger         = xarmlib::PinHal::Name::nc,
               const xarmlib::PinHal::Name output_position_match = xarmlib::PinHal::Name::nc) : TargetEncDriver(input_phase_a,
                                                                                                                input_phase_b,
                                                                                                                config,
                                                                                                                input_index,
                                                                                                                input_home,
                                                                                                                input_trigger,
                                                                                                                output_position_match)
        {}

        // -------- ENABLE / DISABLE WATCHDOG ---------------------------------

        inline void enable_watchdog()          { TargetEncDriver::enable_watchdog(); }
        inline void disable_watchdog()         { TargetEncDriver::disable_watchdog(); }
        inline bool is_enable_watchdog() const { return TargetEncDriver::is_enable_watchdog(); }

        // -------- SOFTWARE INITIALIZE POSITION COUNTER ----------------------

        inline uint32_t get_initial_position() const               { return TargetEncDriver::get_initial_position(); }
        inline void     set_initial_position(const uint32_t value) { TargetEncDriver::set_initial_position(value); }

        inline void software_initialize_position_counter() { TargetEncDriver::software_initialize_position_counter(); }

        // -------- POSITION / REVOLUTION -------------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        inline uint32_t get_position() const               { return TargetEncDriver::get_position(); }
        inline void     set_position(const uint32_t value) { TargetEncDriver::set_position(value); }

        inline uint16_t get_position_difference() const               { return TargetEncDriver::get_position_difference(); }
        inline void     set_position_difference(const uint16_t value) { TargetEncDriver::set_position_difference(value); }

        inline uint16_t get_revolution() const               { return TargetEncDriver::get_revolution(); }
        inline void     set_revolution(const uint16_t value) { TargetEncDriver::set_revolution(value); }

        // -------- HOLD POSITION / REVOLUTION --------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        inline uint32_t get_hold_position() const            { return TargetEncDriver::get_hold_position(); }
        inline uint16_t get_hold_position_difference() const { return TargetEncDriver::get_hold_position_difference(); }
        inline uint16_t get_hold_revolution() const          { return TargetEncDriver::get_hold_revolution(); }

        // -------- COMPARE POSITION ------------------------------------------

        inline uint32_t get_compare_position() const               { return TargetEncDriver::get_compare_position(); }
        inline void     set_compare_position(const uint32_t value) { TargetEncDriver::set_compare_position(value); }

        // -------- STATUS FLAGS ----------------------------------------------

        inline StatusBitmask get_status() const                        { return TargetEncDriver::get_status(); }
        inline void          clear_status(const StatusBitmask bitmask) { TargetEncDriver::clear_status(bitmask); }

        // -------- SIGNAL STATUS FLAGS ---------------------------------------

        inline SignalStatusBitmask get_signal_status() const { return TargetEncDriver::get_signal_status(); }

        // -------- HOME INTERRUPTS -------------------------------------------

        inline void                 enable_home_interrupts (const HomeInterruptBitmask bitmask) { TargetEncDriver::enable_home_interrupts(bitmask); }
        inline void                 disable_home_interrupts(const HomeInterruptBitmask bitmask) { TargetEncDriver::disable_home_interrupts(bitmask); }
        inline HomeInterruptBitmask get_home_interrupts_enabled() const                         { return TargetEncDriver::get_home_interrupts_enabled(); }

        // -------- INDEX INTERRUPTS ------------------------------------------

        inline void                  enable_index_interrupts (const IndexInterruptBitmask bitmask) { TargetEncDriver::enable_index_interrupts(bitmask); }
        inline void                  disable_index_interrupts(const IndexInterruptBitmask bitmask) { TargetEncDriver::disable_index_interrupts(bitmask); }
        inline IndexInterruptBitmask get_index_interrupts_enabled() const                          { return TargetEncDriver::get_index_interrupts_enabled(); }

        // -------- WATCHDOG AND SAB INTERRUPTS -------------------------------

        inline void                           enable_watchdog_and_sab_interrupts (const WatchdogAndSabInterruptBitmask bitmask) { TargetEncDriver::enable_watchdog_and_sab_interrupts(bitmask); }
        inline void                           disable_watchdog_and_sab_interrupts(const WatchdogAndSabInterruptBitmask bitmask) { TargetEncDriver::disable_watchdog_and_sab_interrupts(bitmask); }
        inline WatchdogAndSabInterruptBitmask get_watchdog_and_sab_interrupts_enabled() const                                   { return TargetEncDriver::get_watchdog_and_sab_interrupts_enabled(); }

        // -------- COMPARE INTERRUPTS ----------------------------------------

        inline void                    enable_compare_interrupts (const CompareInterruptBitmask bitmask) { TargetEncDriver::enable_compare_interrupts(bitmask); }
        inline void                    disable_compare_interrupts(const CompareInterruptBitmask bitmask) { TargetEncDriver::disable_compare_interrupts(bitmask); }
        inline CompareInterruptBitmask get_compare_interrupts_enabled() const                            { return TargetEncDriver::get_compare_interrupts_enabled(); }

        // -------- HOME IRQ / IRQ HANDLER ------------------------------------

        inline void enable_home_irq()           { TargetEncDriver::enable_home_irq(); }
        inline void disable_home_irq()          { TargetEncDriver::disable_home_irq(); }
        inline bool is_home_irq_enabled() const { return TargetEncDriver::is_home_irq_enabled(); }

        inline void set_home_irq_priority(const int32_t irq_priority) { TargetEncDriver::set_home_irq_priority(irq_priority); }

        inline void assign_home_irq_handler(const IrqHandler& irq_handler) { TargetEncDriver::assign_home_irq_handler(irq_handler); }
        inline void remove_home_irq_handler()                              { TargetEncDriver::remove_home_irq_handler(); }

        // -------- INDEX IRQ / IRQ HANDLER -----------------------------------

        inline void enable_index_irq()           { TargetEncDriver::enable_index_irq(); }
        inline void disable_index_irq()          { TargetEncDriver::disable_index_irq(); }
        inline bool is_index_irq_enabled() const { return TargetEncDriver::is_index_irq_enabled(); }

        inline void set_index_irq_priority(const int32_t irq_priority) { TargetEncDriver::set_index_irq_priority(irq_priority); }

        inline void assign_index_irq_handler(const IrqHandler& irq_handler) { TargetEncDriver::assign_index_irq_handler(irq_handler); }
        inline void remove_index_irq_handler()                              { TargetEncDriver::remove_index_irq_handler(); }

        // -------- WATCHDOG AND SAB IRQ / IRQ HANDLER ------------------------

        inline void enable_watchdog_and_sab_irq()           { TargetEncDriver::enable_watchdog_and_sab_irq(); }
        inline void disable_watchdog_and_sab_irq()          { TargetEncDriver::disable_watchdog_and_sab_irq(); }
        inline bool is_watchdog_and_sab_irq_enabled() const { return TargetEncDriver::is_watchdog_and_sab_irq_enabled(); }

        inline void set_watchdog_and_sab_irq_priority(const int32_t irq_priority) { TargetEncDriver::set_watchdog_and_sab_irq_priority(irq_priority); }

        inline void assign_watchdog_and_sab_irq_handler(const IrqHandler& irq_handler) { TargetEncDriver::assign_watchdog_and_sab_irq_handler(irq_handler); }
        inline void remove_watchdog_and_sab_irq_handler()                              { TargetEncDriver::remove_watchdog_and_sab_irq_handler(); }

        // -------- COMPARE IRQ / IRQ HANDLER ---------------------------------

        inline void enable_compare_irq()           { TargetEncDriver::enable_compare_irq(); }
        inline void disable_compare_irq()          { TargetEncDriver::disable_compare_irq(); }
        inline bool is_compare_irq_enabled() const { return TargetEncDriver::is_compare_irq_enabled(); }

        inline void set_compare_irq_priority(const int32_t irq_priority) { TargetEncDriver::set_compare_irq_priority(irq_priority); }

        inline void assign_compare_irq_handler(const IrqHandler& irq_handler) { TargetEncDriver::assign_compare_irq_handler(irq_handler); }
        inline void remove_compare_irq_handler()                              { TargetEncDriver::remove_compare_irq_handler(); }
};



} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_enc.hpp"

namespace xarmlib
{
using EncHal = hal::EncHal<targets::kv4x::EncDriver>;

class Enc : public EncHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using HomeTriggerMode          = typename EncHal::HomeTriggerMode;
        using DirectionCounting        = typename EncHal::DirectionCounting;
        using PhaseCountMode           = typename EncHal::PhaseCountMode;
        using IndexTriggerMode         = typename EncHal::IndexTriggerMode;
        using Watchdog                 = typename EncHal::Watchdog;
        using InputFilterSampleCount   = typename EncHal::InputFilterSampleCount;
        using OutputControl            = typename EncHal::OutputControl;
        using RevolutionCounterModulus = typename EncHal::RevolutionCounterModulus;
        using ModulusCounting          = typename EncHal::ModulusCounting;
        using UpdatePositionRegisters  = typename EncHal::UpdatePositionRegisters;
        using UpdateHoldRegisters      = typename EncHal::UpdateHoldRegisters;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Enc(const PinHal::Name input_phase_a,
            const PinHal::Name input_phase_b,
            const Config&      config,
            const PinHal::Name input_index           = PinHal::Name::nc,
            const PinHal::Name input_home            = PinHal::Name::nc,
            const PinHal::Name input_trigger         = PinHal::Name::nc,
            const PinHal::Name output_position_match = PinHal::Name::nc) : EncHal(input_phase_a,
                                                                                  input_phase_b,
                                                                                  config,
                                                                                  input_index,
                                                                                  input_home,
                                                                                  input_trigger,
                                                                                  output_position_match)
        {}
};
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using EncHal = hal::EncHal<targets::other_target::EncDriver>;
using Enc = EncHal;
}

#endif




#endif // __XARMLIB_HAL_ENC_HPP
