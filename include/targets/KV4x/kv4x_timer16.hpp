// ----------------------------------------------------------------------------
// @file    kv4x_timer16.hpp
// @brief   Kinetis KV4x Timer 16-bit (LPTMR) class.
// @note    Only time counter mode is implemented
// @date    4 December 2018
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

#ifndef __XARMLIB_TARGETS_KV4X_TIMER16_HPP
#define __XARMLIB_TARGETS_KV4X_TIMER16_HPP

#include "fsl_lptmr.h"
#include "targets/KV4x/kv4x_system.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <chrono>




// Forward declaration of IRQ handler
extern "C" void LPTMR0_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class Timer16Driver : private PeripheralRefCounter<Timer16Driver, TARGET_TIMER16_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void ::LPTMR0_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralTimer16 = PeripheralRefCounter<Timer16Driver, TARGET_TIMER16_COUNT>;

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        Timer16Driver() : PeripheralTimer16(*this)
        {
            // Initialize LPTMR
            const lptmr_config_t lptmr_config =
            {
                kLPTMR_TimerModeTimeCounter,    // Use time counter mode
                kLPTMR_PinSelectInput_0,        // Use input 0 as source in pulse counter mode (N/A)
                kLPTMR_PinPolarityActiveHigh,   // Pulse input pin polarity is active-high (N/A)
                false,                          // Counter resets whenever TCF flag is set
                false,                          // Use clock from prescaler
                kLPTMR_PrescalerClock_0,        // MCGIRCLK as LPTMR clock source (4 MHz)
                kLPTMR_Prescale_Glitch_1        // Prescaler divide 4
            };

            LPTMR_Init(LPTMR0, &lptmr_config);

            disable_irq();
            set_period(0);
        }

        ~Timer16Driver()
        {
            disable_irq();
            set_period(0);

            // Deinitialize LPTMR
            LPTMR_Deinit(LPTMR0);
        }

        // -------- START / STOP ----------------------------------------------

        // Start the timer with the supplied period
        void start(const std::chrono::microseconds& rate_us)
        {
            assert(rate_us.count() >= get_min_rate_us());
            assert(rate_us.count() <= get_max_rate_us());

            const uint32_t period = convert_us_to_period(rate_us.count());

            disable();
            set_period(period);
            enable();
        }

        // Reload the timer by first disabling and then enabling
        void reload()
        {
            // Ensure period is set
            assert(get_period() != 0);

            disable();
            enable();
        }

        // Stop a previously started timer
        void stop()
        {
            disable();
        }

        bool is_running() const
        {
            return is_enabled();
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_irq()
        {
            clear_irq_pending();

            LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

            NVIC_EnableIRQ(LPTMR0_IRQn);
        }

        void disable_irq()
        {
            LPTMR_DisableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

            NVIC_DisableIRQ(LPTMR0_IRQn);

            clear_irq_pending();
        }

        bool is_irq_enabled() const
        {
            return (LPTMR_GetEnabledInterrupts(LPTMR0) != 0);
        }

        bool is_irq_pending() const
        {
            return (LPTMR_GetStatusFlags(LPTMR0) != 0);
        }

        void clear_irq_pending()
        {
            LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
        }

        void set_irq_priority(const uint32_t irq_priority)
        {
            NVIC_SetPriority(LPTMR0_IRQn, irq_priority);
        }

        void assign_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_irq_handler = irq_handler;
        }

        void remove_irq_handler()
        {
            m_irq_handler = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- ENABLE / DISABLE ------------------------------------------

        void enable() { LPTMR_StartTimer(LPTMR0); }

        void disable() { LPTMR_StopTimer(LPTMR0); }

        bool is_enabled() const { return ((LPTMR0->CSR & LPTMR_CSR_TEN_MASK) != 0); }

        // -------- PERIOD CONFIGURATION --------------------------------------

        // Set timer period value
        void set_period(const uint32_t timer_period) { LPTMR0->CMR = timer_period; }

        // Get timer period value
        uint32_t get_period() const { return LPTMR0->CMR; }

        // Get timer period value (ready to load into CMR register) based on supplied rate in microseconds
        static uint16_t convert_us_to_period(const int64_t rate_us)
        {
            constexpr int32_t prescaler_output = SystemDriver::get_internal_reference_clock_frequency() / 4;

            return static_cast<uint16_t>(prescaler_output * rate_us / 1000000UL);
        }

        // Get the minimum allowed rate in microseconds
        static constexpr int64_t get_min_rate_us()
        {
            constexpr int64_t min_period = 0x01;

            constexpr int32_t prescaler_output = SystemDriver::get_internal_reference_clock_frequency() / 4;

            return (min_period * 1000000UL / prescaler_output);
        }

        // Get the maximum allowed rate in microseconds
        static constexpr int64_t get_max_rate_us()
        {
            constexpr int64_t max_period = 0xFFFF;

            constexpr int32_t prescaler_output = SystemDriver::get_internal_reference_clock_frequency() / 4;

            return (max_period * 1000000UL / prescaler_output);
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            clear_irq_pending();

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            return yield;
        }

        // IRQ handler called directly by the interrupt C function
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const std::size_t index)
        {
            return Timer16Driver::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        IrqHandler m_irq_handler;   // User defined IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_TIMER16_HPP
