// ----------------------------------------------------------------------------
// @file    kv4x_enc.hpp
// @brief   Kinetis KV4x Quadrature Encoder/Decoder (ENC) class.
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

#ifndef __XARMLIB_TARGETS_KV4X_ENC_HPP
#define __XARMLIB_TARGETS_KV4X_ENC_HPP

#include "xarmlib_config.hpp"
#include "external/bitmask.hpp"
#include "fsl_enc.h"
#include "targets/KV4x/kv4x_xbara.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <chrono>




// Forward declaration of IRQ handlers for all KV4x packages
extern "C" void ENC_HOME_IRQHandler    (void);
extern "C" void ENC_INDEX_IRQHandler   (void);
extern "C" void ENC_WDOG_SAB_IRQHandler(void);
extern "C" void ENC_COMPARE_IRQHandler (void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




namespace private_enc
{

// Quadrature Encoder/Decoder status flags
enum class Status
{
    home_signal_transition         = (1 << 0),  // HOME signal transition interrupt request (CTRL[HIRQ])
    index_pulse                    = (1 << 1),  // INDEX pulse interrupt request (CTRL[XIRQ])
    watchdog_timeout               = (1 << 2),  // Watchdog timeout interrupt request (CTRL[DIRQ])
    compare_match                  = (1 << 3),  // Compare match interrupt request (CTRL[CMPIRQ])
    simultaneous_both_phase_change = (1 << 4),  // Simultaneous PHASEA and PHASEB change interrupt request (CTRL2[SABIRQ])
    position_roll_over             = (1 << 5),  // Roll-over interrupt request (CTRL2[ROIRQ])
    position_roll_under            = (1 << 6),  // Roll-under interrupt request (CTRL2[RUIRQ])
    last_count_direction           = (1 << 7),  // 0: Last count was in the down direction (CTRL2[DIR])
                                                // 1: Last count was in the up direction
    clear_all_bitmask              = home_signal_transition
                                   | index_pulse
                                   | watchdog_timeout
                                   | compare_match
                                   | simultaneous_both_phase_change
                                   | position_roll_over
                                   | position_roll_under,
    bitmask                        = home_signal_transition
                                   | index_pulse
                                   | watchdog_timeout
                                   | compare_match
                                   | simultaneous_both_phase_change
                                   | position_roll_over
                                   | position_roll_under
                                   | last_count_direction
};

// Quadrature Encoder/Decoder signal status flags read-only
enum class SignalStatus
{
    raw_home_input        = ENC_IMR_HOME_MASK,  // Raw HOME input
    raw_index_input       = ENC_IMR_INDEX_MASK, // Raw INDEX input
    raw_phaseb_input      = ENC_IMR_PHB_MASK,   // Raw PHASEB input
    raw_phasea_input      = ENC_IMR_PHA_MASK,   // Raw PHASEA input
    filtered_home_input   = ENC_IMR_FHOM_MASK,  // The filtered version of HOME input
    filtered_index_input  = ENC_IMR_FIND_MASK,  // The filtered version of INDEX input
    filtered_phaseb_input = ENC_IMR_FPHB_MASK,  // The filtered version of PHASEB input
    filtered_phasea_input = ENC_IMR_FPHA_MASK,  // The filtered version of PHASEA input
    bitmask               = raw_home_input
                          | raw_index_input
                          | raw_phaseb_input
                          | raw_phasea_input
                          | filtered_home_input
                          | filtered_index_input
                          | filtered_phaseb_input
                          | filtered_phasea_input
};




// Quadrature Encoder/Decoder HOME interrupt source
enum class HomeInterrupt
{
    home_signal_transition = (1 << 0),  // HOME signal transition interrupt (CTRL[HIE])
    bitmask                = home_signal_transition
};

// Quadrature Encoder/Decoder INDEX interrupt sources
enum class IndexInterrupt
{
    index_pulse         = (1 << 1),  // INDEX pulse interrupt (CTRL[XIE])
    position_roll_over  = (1 << 5),  // Roll-over interrupt (CTRL2[ROIE])
    position_roll_under = (1 << 6),  // Roll-under interrupt (CTRL2[RUIE])
    bitmask             = index_pulse
                        | position_roll_over
                        | position_roll_under
};

// Quadrature Encoder/Decoder watchdog timeout and simultaneous PHASEA and PHASEB change interrupt sources
enum class WatchdogAndSabInterrupt
{
    watchdog_timeout               = (1 << 2),  // Watchdog timeout interrupt (CTRL[DIE])
    simultaneous_both_phase_change = (1 << 4),  // Simultaneous PHASEA and PHASEB change interrupt (CTRL2[SABIE])
    bitmask                        = watchdog_timeout
                                   | simultaneous_both_phase_change
};

// Quadrature Encoder/Decoder compare interrupt source
enum class CompareInterrupt
{
    compare_match = (1 << 3),  // Compare match interrupt (CTRL[CMPIE])
    bitmask       = compare_match
};

// Quadrature Encoder/Decoder interrupt sources
/*enum class Interrupt
{
    home_signal_transition         = (1 << 0),  // HOME signal transition interrupt (CTRL[HIE])
    index_pulse                    = (1 << 1),  // INDEX pulse interrupt (CTRL[XIE])
    watchdog_timeout               = (1 << 2),  // Watchdog timeout interrupt (CTRL[DIE])
    compare_match                  = (1 << 3),  // Compare match interrupt (CTRL[CMPIE])
    simultaneous_both_phase_change = (1 << 4),  // Simultaneous PHASEA and PHASEB change interrupt (CTRL2[SABIE])
    position_roll_over             = (1 << 5),  // Roll-over interrupt (CTRL2[ROIE])
    position_roll_under            = (1 << 6),  // Roll-under interrupt (CTRL2[RUIE])
    bitmask                        = home_signal_transition
                                   | index_pulse
                                   | watchdog_timeout
                                   | compare_match
                                   | simultaneous_both_phase_change
                                   | position_roll_over
                                   | position_roll_under
};*/

BITMASK_DEFINE_VALUE_MASK(Status,                  static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(SignalStatus,            static_cast<uint32_t>(SignalStatus::bitmask))
BITMASK_DEFINE_VALUE_MASK(HomeInterrupt,           static_cast<uint32_t>(HomeInterrupt::bitmask))
BITMASK_DEFINE_VALUE_MASK(IndexInterrupt,          static_cast<uint32_t>(IndexInterrupt::bitmask))
BITMASK_DEFINE_VALUE_MASK(WatchdogAndSabInterrupt, static_cast<uint32_t>(WatchdogAndSabInterrupt::bitmask))
BITMASK_DEFINE_VALUE_MASK(CompareInterrupt,        static_cast<uint32_t>(CompareInterrupt::bitmask))

} // namespace private_enc




class EncDriver : private PeripheralRefCounter<EncDriver, TARGET_ENC_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::ENC_HOME_IRQHandler    (void);
        friend void ::ENC_INDEX_IRQHandler   (void);
        friend void ::ENC_WDOG_SAB_IRQHandler(void);
        friend void ::ENC_COMPARE_IRQHandler (void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralEnc = PeripheralRefCounter<EncDriver, TARGET_ENC_COUNT>;

        // HOME signal's trigger mode selection
        enum class HomeTriggerMode
        {
            disabled = 0,   // HOME signal's trigger is disabled
            on_rising_edge, // Use positive going edge-to-trigger initialization of position counters
            on_falling_edge // Use negative going edge-to-trigger initialization of position counters
        };

        // Direction of count (interpretation of the quadrature signal) mode selection
        enum class DirectionCounting
        {
            normal = 0,     // Count normally
            reverse         // Count in the reverse direction
        };

        // Phase count (quadrature decoder) mode selection
        enum class PhaseCountMode
        {
            standard = 0,   // Use standard quadrature decoder with PHASEA and PHASEB
            bypass          // PHASEA input generates a count signal
                            // while PHASEB input and the REV bit (DirectionCounting) control the direction
        };

        // INDEX signal's trigger mode selection
        enum class IndexTriggerMode
        {
            disabled = 0,   // INDEX signal's trigger is disabled
            on_rising_edge, // Use positive going edge-to-trigger initialization of position counters
            on_falling_edge // Use negative going edge-to-trigger initialization of position counters
        };

        // Watchdog enable selection
        enum class Watchdog
        {
            disabled = 0,   // Disable watchdog timer
            enabled         // Enable the watchdog timer to detect if the target is moving
        };

        // Input filter sample count for PHASEA, PHASEB, INDEX and HOME
        // NOTE: this value should be chosen to reduce the probability of noisy samples causing an
        //       incorrect transition to be recognized. The value represent the number of consecutive
        //       samples that must agree prior to the input filter accepting an input transition.
        enum class InputFilterSampleCount
        {
            count_3_samples = 0,
            count_4_samples,
            count_5_samples,
            count_6_samples,
            count_7_samples,
            count_8_samples,
            count_9_samples,
            count_10_samples
        };

        // Output control [the behavior of the POSMATCH output signal] mode selection
        enum class OutputControl
        {
            on_position_counter_equal_to_compare_value = 0, // POSMATCH pulses when a match occurs between the
                                                            // position counters (POS) and the compare value (COMP)
            on_reading_any_position_counter                 // POSMATCH pulses when the UPOS, LPOS, REV or POSD
                                                            // registers are read
        };

        // Revolution counter modulus enable selection
        enum class RevolutionCounterModulus
        {
            on_index_pulse = 0,                             // Use INDEX pulse to increment/decrement revolution counter (REV)
            on_roll_over_or_under_modulus                   // Use modulus counting roll-over/under to increment/decrement
                                                            // revolution counter (REV)
        };

        // Enable modulus counting selection
        // NOTE: if enabled, it allows the position counters to count in a modulo fashion using
        //       position_modulus_value and position_initial_value as the upper and lower bounds
        //       of the counting range. During modulo counting when a count up is indicated and
        //       the position counter is equal to position_modulus_value, then the position
        //       counter will be reloaded with position_initial_value. When a count down is
        //       indicated and the position counter is equal to position_initial_value, then the
        //       position counter will be reloaded with position_modulus_value.
        //       If disabled, then the position_modulus_value and position_initial_value are
        //       ignored and the position counter wraps to zero when counting up from 0xffffffff
        //       and wraps to 0xffffffff when counting down from 0.
        enum class ModulusCounting
        {
            disabled = 0,   // Disable modulus counting
            enabled         // Enable modulus counting
        };

        // Update position registers mode selection
        enum class UpdatePositionRegisters
        {
            disabled = 0,   // No action for POSD, REV, UPOS and LPOS on rising edge of TRIGGER
            enabled         // Clear POSD, REV, UPOS and LPOS on rising edge of TRIGGER
        };

        // Update hold registers mode selection
        enum class UpdateHoldRegisters
        {
            disabled = 0,   // Disable updates of hold registers on rising edge of TRIGGER
            enabled         // Enable updates of hold registers on rising edge of TRIGGER
        };

        struct Config
        {
            HomeTriggerMode   home_trigger_mode             = HomeTriggerMode::disabled;
            DirectionCounting direction_counting            = DirectionCounting::normal;
            PhaseCountMode    phase_count_mode              = PhaseCountMode::standard;
            IndexTriggerMode  index_trigger_mode            = IndexTriggerMode::disabled;
            Watchdog          watchdog                      = Watchdog::disabled;
            std::chrono::microseconds watchdog_timeout_rate = std::chrono::microseconds(0);
            //uint16_t          watchdog_timeout   = 0;

            InputFilterSampleCount input_filter_sample_count = InputFilterSampleCount::count_3_samples;
            // Input filter sample period for PHASEA, PHASEB, INDEX and HOME
            // - This value should be set such that the sampling period is larger than the period of
            //   the expected noise. This value represents the sampling period (in IPBus clock cycles)
            //   of the decoder input signals. If 0x00 (default), then the input filter is bypassed.
            std::chrono::microseconds input_filter_sample_rate = std::chrono::microseconds(0);
            //uint8_t input_filter_sample_period = 0;

            OutputControl            output_control             = OutputControl::on_position_counter_equal_to_compare_value;
            RevolutionCounterModulus revolution_counter_modulus = RevolutionCounterModulus::on_index_pulse;
            ModulusCounting          modulus_counting           = ModulusCounting::disabled;
            UpdatePositionRegisters  update_position_registers  = UpdatePositionRegisters::disabled;
            UpdateHoldRegisters      update_hold_registers      = UpdateHoldRegisters::disabled;

            uint32_t initial_position_value = 0;
            uint32_t modulus_position_value = 0;
            uint32_t compare_position_value = 0xFFFFFFFF;
        };

        // Type safe accessor to status flags
        using Status        = private_enc::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to signal status flags
        using SignalStatus        = private_enc::SignalStatus;
        using SignalStatusBitmask = bitmask::bitmask<SignalStatus>;

        // Type safe accessor to HOME interrupt sources
        using HomeInterrupt        = private_enc::HomeInterrupt;
        using HomeInterruptBitmask = bitmask::bitmask<HomeInterrupt>;

        // Type safe accessor to INDEX interrupt sources
        using IndexInterrupt        = private_enc::IndexInterrupt;
        using IndexInterruptBitmask = bitmask::bitmask<IndexInterrupt>;

        // Type safe accessor to watchdog timeout and simultaneous PHASEA and PHASEB change interrupt sources
        using WatchdogAndSabInterrupt        = private_enc::WatchdogAndSabInterrupt;
        using WatchdogAndSabInterruptBitmask = bitmask::bitmask<WatchdogAndSabInterrupt>;

        // Type safe accessor to compare interrupt sources
        using CompareInterrupt        = private_enc::CompareInterrupt;
        using CompareInterruptBitmask = bitmask::bitmask<CompareInterrupt>;

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        EncDriver(const PinDriver::Name input_phase_a,
                  const PinDriver::Name input_phase_b,
                  const Config&         config,
                  const PinDriver::Name input_index           = PinDriver::Name::nc,
                  const PinDriver::Name input_home            = PinDriver::Name::nc,
                  const PinDriver::Name input_trigger         = PinDriver::Name::nc,
                  const PinDriver::Name output_position_match = PinDriver::Name::nc) : PeripheralEnc(*this)
        {
            const XbaraDriver::InputPinConfig phase_a = XbaraDriver::get_input_pin_config(input_phase_a);
            const XbaraDriver::InputPinConfig phase_b = XbaraDriver::get_input_pin_config(input_phase_b);

            PinDriver::set_pin_mux(input_phase_a, phase_a.pin_mux);
            PinDriver::set_pin_mux(input_phase_b, phase_b.pin_mux);

            XbaraDriver::set_signals_connection(phase_a.input_signal, XbaraDriver::OutputSignal::enc_pha);
            XbaraDriver::set_signals_connection(phase_b.input_signal, XbaraDriver::OutputSignal::enc_phb);

            if(input_index != PinDriver::Name::nc)
            {
                const XbaraDriver::InputPinConfig index = XbaraDriver::get_input_pin_config(input_index);

                PinDriver::set_pin_mux(input_index, index.pin_mux);
                XbaraDriver::set_signals_connection(index.input_signal, XbaraDriver::OutputSignal::enc_index);
            }

            if(input_home != PinDriver::Name::nc)
            {
                const XbaraDriver::InputPinConfig home = XbaraDriver::get_input_pin_config(input_home);

                PinDriver::set_pin_mux(input_home, home.pin_mux);
                XbaraDriver::set_signals_connection(home.input_signal, XbaraDriver::OutputSignal::enc_home);
            }

            if(input_trigger != PinDriver::Name::nc)
            {
                const XbaraDriver::InputPinConfig trigger = XbaraDriver::get_input_pin_config(input_trigger);

                PinDriver::set_pin_mux(input_trigger, trigger.pin_mux);
                XbaraDriver::set_signals_connection(trigger.input_signal, XbaraDriver::OutputSignal::enc_trig);
            }

            if(output_position_match != PinDriver::Name::nc)
            {
                const XbaraDriver::OutputPinConfig position_match = XbaraDriver::get_output_pin_config(output_position_match);

                PinDriver::set_pin_mux(output_position_match, position_match.pin_mux);
                XbaraDriver::set_signals_connection(XbaraDriver::InputSignal::enc_posmatch, position_match.output_signal);
            }

            assert(config.watchdog_timeout_rate.count() >= 0);
            assert(config.watchdog_timeout_rate.count() <= get_max_watchdog_timeout_rate_us());

            const uint16_t watchdog_timeout_period = convert_us_to_period(config.watchdog_timeout_rate.count());

            assert(config.input_filter_sample_rate.count() >= 0);
            assert(config.input_filter_sample_rate.count() <= get_max_input_filter_sample_rate_us());

            const uint16_t input_filter_sample_period = convert_us_to_period(config.input_filter_sample_rate.count());

            const enc_config_t enc_config =
            {
                static_cast<bool>(config.direction_counting),
                static_cast<enc_decoder_work_mode_t>(config.phase_count_mode),
                static_cast<enc_home_trigger_mode_t>(config.home_trigger_mode),
                static_cast<enc_index_trigger_mode_t>(config.index_trigger_mode),
                static_cast<bool>(config.update_position_registers),
                static_cast<bool>(config.update_hold_registers),
                static_cast<bool>(config.watchdog),
                watchdog_timeout_period,
                static_cast<uint16_t>(config.input_filter_sample_count),
                input_filter_sample_period,
                static_cast<enc_position_match_mode_t>(config.output_control),
                config.compare_position_value,
                static_cast<enc_revolution_count_condition_t>(config.revolution_counter_modulus),
                static_cast<bool>(config.modulus_counting),
                config.modulus_position_value,
                config.initial_position_value
            };

            ENC_Init(ENC, &enc_config);

            disable_home_irq();
            disable_index_irq();
            disable_watchdog_and_sab_irq();
            disable_compare_irq();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);
        }

        ~EncDriver()
        {
            disable_home_irq();
            disable_index_irq();
            disable_watchdog_and_sab_irq();
            disable_compare_irq();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);

            ENC_Deinit(ENC);
        }

        // -------- ENABLE / DISABLE WATCHDOG ---------------------------------

        // Enable watchdog
        void enable_watchdog()
        {
            ENC_EnableWatchdog(ENC, true);
        }

        // Disable watchdog
        void disable_watchdog()
        {
            ENC_EnableWatchdog(ENC, false);
        }

        // Gets the enable watchdog state
        bool is_enable_watchdog() const
        {
            return ((ENC->CTRL & ENC_CTRL_WDE_MASK) != 0);
        }

        // -------- SOFTWARE INITIALIZE POSITION COUNTER ----------------------

        // Get initial position value
        uint32_t get_initial_position() const
        {
            return ((ENC->UINIT << 16UL) | ENC->LINIT);
        }

        // Set initial position value
        void set_initial_position(const uint32_t value)
        {
            ENC_SetInitialPositionValue(ENC, value);
        }

        // Software triggered initialization of position counter
        void software_initialize_position_counter()
        {
            ENC_DoSoftwareLoadInitialPositionValue(ENC);
        }

        // -------- POSITION / REVOLUTION -------------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        // Get the current position counter's value
        uint32_t get_position() const
        {
            return ENC_GetPositionValue(ENC);
        }

        // Set the current position counter's value
        void set_position(const uint32_t value)
        {
            ENC->UPOS = static_cast<uint16_t>(value >> 16UL);
            ENC->LPOS = static_cast<uint16_t>(value);
        }

        // Get the position difference counter's value
        uint16_t get_position_difference() const
        {
            return ENC_GetPositionDifferenceValue(ENC);
        }

        // Set the position difference counter's value
        void set_position_difference(const uint16_t value)
        {
            ENC->POSD = value;
        }

        // Get the revolution counter's value
        uint16_t get_revolution() const
        {
            return ENC_GetRevolutionValue(ENC);
        }

        // Set the revolution counter's value
        void set_revolution(const uint16_t value)
        {
            ENC->REV = value;
        }

        // -------- HOLD POSITION / REVOLUTION --------------------------------

        // NOTE: when any of the counter registers is read, the contents of each
        //       counter register is written to the corresponding hold register
        //       and the position difference counter is cleared.
        //       Taking a snapshot of the counters' values provides a consistent
        //       view of a system position and a velocity to be attained.

        // Get the hold position counter's value
        uint32_t get_hold_position() const
        {
            return ENC_GetHoldPositionValue(ENC);
        }

        // Get the hold position difference counter's value
        uint16_t get_hold_position_difference() const
        {
            return ENC_GetHoldPositionDifferenceValue(ENC);
        }

        // Get the hold revolution counter's value
        uint16_t get_hold_revolution() const
        {
            return ENC_GetHoldRevolutionValue(ENC);
        }

        // -------- COMPARE POSITION ------------------------------------------

        // Get the compare position counter's value
        uint32_t get_compare_position() const
        {
            return ((ENC->UCOMP << 16UL) | ENC->LCOMP);
        }

        // Set the compare position counter's value
        void set_compare_position(const uint32_t value)
        {
            ENC->UCOMP = static_cast<uint16_t>(value >> 16UL);
            ENC->LCOMP = static_cast<uint16_t>(value);
        }

        // -------- STATUS FLAGS ----------------------------------------------

        StatusBitmask get_status() const
        {
            return static_cast<Status>(ENC_GetStatusFlags(ENC));
        }

        void clear_status(const StatusBitmask bitmask)
        {
            ENC_ClearStatusFlags(ENC, (bitmask & Status::clear_all_bitmask).bits());
        }

        // -------- SIGNAL STATUS FLAGS ---------------------------------------

        SignalStatusBitmask get_signal_status() const
        {
            return static_cast<SignalStatus>(ENC_GetSignalStatusFlags(ENC));
        }

        // -------- HOME INTERRUPTS -------------------------------------------

        void enable_home_interrupts(const HomeInterruptBitmask bitmask)
        {
            ENC_EnableInterrupts(ENC, bitmask.bits());
        }

        void disable_home_interrupts(const HomeInterruptBitmask bitmask)
        {
            ENC_DisableInterrupts(ENC, bitmask.bits());
        }

        HomeInterruptBitmask get_home_interrupts_enabled() const
        {
            return static_cast<HomeInterrupt>(ENC_GetEnabledInterrupts(ENC));
        }

        // -------- INDEX INTERRUPTS ------------------------------------------

        void enable_index_interrupts(const IndexInterruptBitmask bitmask)
        {
            ENC_EnableInterrupts(ENC, bitmask.bits());
        }

        void disable_index_interrupts(const IndexInterruptBitmask bitmask)
        {
            ENC_DisableInterrupts(ENC, bitmask.bits());
        }

        IndexInterruptBitmask get_index_interrupts_enabled() const
        {
            return static_cast<IndexInterrupt>(ENC_GetEnabledInterrupts(ENC));
        }

        // -------- WATCHDOG AND SAB INTERRUPTS -------------------------------

        void enable_watchdog_and_sab_interrupts(const WatchdogAndSabInterruptBitmask bitmask)
        {
            ENC_EnableInterrupts(ENC, bitmask.bits());
        }

        void disable_watchdog_and_sab_interrupts(const WatchdogAndSabInterruptBitmask bitmask)
        {
            ENC_DisableInterrupts(ENC, bitmask.bits());
        }

        WatchdogAndSabInterruptBitmask get_watchdog_and_sab_interrupts_enabled() const
        {
            return static_cast<WatchdogAndSabInterrupt>(ENC_GetEnabledInterrupts(ENC));
        }

        // -------- COMPARE INTERRUPTS ----------------------------------------

        void enable_compare_interrupts(const CompareInterruptBitmask bitmask)
        {
            ENC_EnableInterrupts(ENC, bitmask.bits());
        }

        void disable_compare_interrupts(const CompareInterruptBitmask bitmask)
        {
            ENC_DisableInterrupts(ENC, bitmask.bits());
        }

        CompareInterruptBitmask get_compare_interrupts_enabled() const
        {
            return static_cast<CompareInterrupt>(ENC_GetEnabledInterrupts(ENC));
        }

        // -------- HOME IRQ / IRQ HANDLER ------------------------------------

        void enable_home_irq()
        {
            NVIC_EnableIRQ(ENC_HOME_IRQn);
        }

        void disable_home_irq()
        {
            NVIC_DisableIRQ(ENC_HOME_IRQn);
        }

        bool is_home_irq_enabled() const
        {
            return (NVIC_GetEnableIRQ(ENC_HOME_IRQn) != 0);
        }

        void set_home_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(ENC_HOME_IRQn, irq_priority);
        }

        void assign_home_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_home_irq_handler = irq_handler;
        }

        void remove_home_irq_handler()
        {
            m_home_irq_handler = nullptr;
        }

        // -------- INDEX IRQ / IRQ HANDLER -----------------------------------

        void enable_index_irq()
        {
            NVIC_EnableIRQ(ENC_INDEX_IRQn);
        }

        void disable_index_irq()
        {
            NVIC_DisableIRQ(ENC_INDEX_IRQn);
        }

        bool is_index_irq_enabled() const
        {
            return (NVIC_GetEnableIRQ(ENC_INDEX_IRQn) != 0);
        }

        void set_index_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(ENC_INDEX_IRQn, irq_priority);
        }

        void assign_index_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_index_irq_handler = irq_handler;
        }

        void remove_index_irq_handler()
        {
            m_index_irq_handler = nullptr;
        }

        // -------- WATCHDOG AND SAB IRQ / IRQ HANDLER ------------------------

        void enable_watchdog_and_sab_irq()
        {
            NVIC_EnableIRQ(ENC_WDOG_SAB_IRQn);
        }

        void disable_watchdog_and_sab_irq()
        {
            NVIC_DisableIRQ(ENC_WDOG_SAB_IRQn);
        }

        bool is_watchdog_and_sab_irq_enabled() const
        {
            return (NVIC_GetEnableIRQ(ENC_WDOG_SAB_IRQn) != 0);
        }

        void set_watchdog_and_sab_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(ENC_WDOG_SAB_IRQn, irq_priority);
        }

        void assign_watchdog_and_sab_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_watchdog_and_sab_irq_handler = irq_handler;
        }

        void remove_watchdog_and_sab_irq_handler()
        {
            m_watchdog_and_sab_irq_handler = nullptr;
        }

        // -------- COMPARE IRQ / IRQ HANDLER ---------------------------------

        void enable_compare_irq()
        {
            NVIC_EnableIRQ(ENC_COMPARE_IRQn);
        }

        void disable_compare_irq()
        {
            NVIC_DisableIRQ(ENC_COMPARE_IRQn);
        }

        bool is_compare_irq_enabled() const
        {
            return (NVIC_GetEnableIRQ(ENC_COMPARE_IRQn) != 0);
        }

        void set_compare_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(ENC_COMPARE_IRQn, irq_priority);
        }

        void assign_compare_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_compare_irq_handler = irq_handler;
        }

        void remove_compare_irq_handler()
        {
            m_compare_irq_handler = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- PERIOD CONFIGURATION --------------------------------------

        // Get period value based on supplied rate in microseconds
        static uint16_t convert_us_to_period(const int64_t rate_us)
        {
            return static_cast<uint16_t>(SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK) * rate_us / 1000000UL);
        }

        // Get the maximum allowed watchdog timeout rate in microseconds
        static int64_t get_max_watchdog_timeout_rate_us()
        {
            constexpr int64_t max_period = 0xFFFF;

            return (max_period * 1000000UL / SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
        }

        // Get the maximum allowed input filter sample rate in microseconds
        static int64_t get_max_input_filter_sample_rate_us()
        {
            constexpr int64_t max_period = 0xFF;

            return (max_period * 1000000UL / SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
        }

        // -------- PRIVATE HOME IRQ HANDLER ----------------------------------

        // HOME IRQ handler called directly by the interrupt C functions
        // (call user IRQ handler)
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t home_irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(get_reference(0).m_home_irq_handler != nullptr)
            {
                yield = get_reference(0).m_home_irq_handler();
            }

            return yield;
        }

        // -------- PRIVATE INDEX IRQ HANDLER ---------------------------------

        // INDEX IRQ handler called directly by the interrupt C functions
        // (call user IRQ handler)
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t index_irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(get_reference(0).m_index_irq_handler != nullptr)
            {
                yield = get_reference(0).m_index_irq_handler();
            }

            return yield;
        }

        // -------- PRIVATE WATCHDOG AND SAB IRQ HANDLER ----------------------

        // Watchdog timeout and simultaneous PHASEA and PHASEB change IRQ handler called directly by the interrupt C functions
        // (call user IRQ handler)
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t watchdog_and_sab_irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(get_reference(0).m_watchdog_and_sab_irq_handler != nullptr)
            {
                yield = get_reference(0).m_watchdog_and_sab_irq_handler();
            }

            return yield;
        }

        // -------- PRIVATE COMPARE IRQ HANDLER -------------------------------

        // Compare IRQ handler called directly by the interrupt C functions
        // (call user IRQ handler)
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t compare_irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(get_reference(0).m_compare_irq_handler != nullptr)
            {
                yield = get_reference(0).m_compare_irq_handler();
            }

            return yield;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        IrqHandler m_home_irq_handler;              // User defined HOME IRQ handler
        IrqHandler m_index_irq_handler;             // User defined iNDEX IRQ handler
        IrqHandler m_watchdog_and_sab_irq_handler;  // User defined watchdog timeout and simultaneous PHASEA and PHASEB change IRQ handler
        IrqHandler m_compare_irq_handler;           // User defined compare IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_ENC_HPP
