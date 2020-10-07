// ----------------------------------------------------------------------------
// @file    kv4x_timer.hpp
// @brief   Kinetis KV4x Timer (PIT) class.
// @note    Timers stop in debug mode.
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

#ifndef __XARMLIB_TARGETS_KV4X_TIMER_HPP
#define __XARMLIB_TARGETS_KV4X_TIMER_HPP

#include "xarmlib_config.hpp"
#include "fsl_pit.h"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <chrono>




// Forward declaration of IRQ handlers
extern "C" void PIT0_IRQHandler(void);
extern "C" void PIT1_IRQHandler(void);
extern "C" void PIT2_IRQHandler(void);
extern "C" void PIT3_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class TimerDriver : private PeripheralRefCounter<TimerDriver, TARGET_TIMER_COUNT>
{
    public:

        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::PIT0_IRQHandler(void);
        friend void ::PIT1_IRQHandler(void);
        friend void ::PIT2_IRQHandler(void);
        friend void ::PIT3_IRQHandler(void);

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralTimer = PeripheralRefCounter<TimerDriver, TARGET_TIMER_COUNT>;

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        TimerDriver() : PeripheralTimer(*this)
        {
            // Initialize PIT if this is the first timer created
            if(get_used() == 1)
            {
                const pit_config_t pit_config =
                {
                    false // Timers stop in debug mode
                };

                PIT_Init(PIT, &pit_config);
            }

            disable();
            disable_irq();
            set_period(0);
        }

        ~TimerDriver()
        {
            disable();
            disable_irq();
            set_period(0);

            // Deinitialize PIT if this the last timer deleted
            if(get_used() == 1)
            {
                PIT_Deinit(PIT);
            }
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

        // Start the timer with maximum period
        void start_free_running()
        {
            disable();
            set_period(0xFFFFFFFF);
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

        // -------- UP COUNTER ------------------------------------------------

        std::chrono::microseconds up_counter()
        {
            // NOTES: - if the timer is disabled, timer value is unreliable
            //        - the timer uses a downcounter

            assert(is_enabled() == true);

            const uint32_t up_counter = get_period() - PIT_GetCurrentTimerCount(PIT, static_cast<pit_chnl_t>(get_index()));

            return std::chrono::microseconds(convert_period_to_us(up_counter));
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_irq()
        {
            clear_irq_pending();

            PIT_EnableInterrupts(PIT, static_cast<pit_chnl_t>(get_index()), kPIT_TimerInterruptEnable);

            const Channel channel = static_cast<Channel>(get_index());

            switch(channel)
            {
                case Channel::channel0: NVIC_EnableIRQ(PIT0_IRQn); break;
                case Channel::channel1: NVIC_EnableIRQ(PIT1_IRQn); break;
                case Channel::channel2: NVIC_EnableIRQ(PIT2_IRQn); break;
                case Channel::channel3: NVIC_EnableIRQ(PIT3_IRQn); break;
                default:                                           break;
            }
        }

        void disable_irq()
        {
            PIT_DisableInterrupts(PIT, static_cast<pit_chnl_t>(get_index()), kPIT_TimerInterruptEnable);

            const Channel channel = static_cast<Channel>(get_index());

            switch(channel)
            {
                case Channel::channel0: NVIC_DisableIRQ(PIT0_IRQn); break;
                case Channel::channel1: NVIC_DisableIRQ(PIT1_IRQn); break;
                case Channel::channel2: NVIC_DisableIRQ(PIT2_IRQn); break;
                case Channel::channel3: NVIC_DisableIRQ(PIT3_IRQn); break;
                default:                                            break;
            }

            clear_irq_pending();
        }

        bool is_irq_enabled() const
        {
            return (PIT_GetEnabledInterrupts(PIT, static_cast<pit_chnl_t>(get_index())) != 0);
        }

        bool is_irq_pending() const
        {
            return (PIT_GetStatusFlags(PIT, static_cast<pit_chnl_t>(get_index())) != 0);
        }

        void clear_irq_pending()
        {
            PIT_ClearStatusFlags(PIT, static_cast<pit_chnl_t>(get_index()), kPIT_TimerFlag);
        }

        void set_irq_priority(const uint32_t irq_priority)
        {
            const Channel channel = static_cast<Channel>(get_index());

            switch(channel)
            {
                case Channel::channel0: NVIC_SetPriority(PIT0_IRQn, irq_priority); break;
                case Channel::channel1: NVIC_SetPriority(PIT1_IRQn, irq_priority); break;
                case Channel::channel2: NVIC_SetPriority(PIT2_IRQn, irq_priority); break;
                case Channel::channel3: NVIC_SetPriority(PIT3_IRQn, irq_priority); break;
                default:                                                           break;
            }
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
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // PIT channel names
        enum class Channel
        {
            channel0 = 0,
            channel1,
            channel2,
            channel3
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable the timer channel
        void enable() { PIT_StartTimer(PIT, static_cast<pit_chnl_t>(get_index())); }

        // Disable the timer channel
        void disable() { PIT_StopTimer(PIT, static_cast<pit_chnl_t>(get_index())); }

        // Gets the enable state
        bool is_enabled() const { return ((PIT->CHANNEL[get_index()].TCTRL & PIT_TCTRL_TEN_MASK) != 0); }

        // -------- PERIOD CONFIGURATION --------------------------------------

        // Set timer period value
        void set_period(const uint32_t timer_period) { PIT_SetTimerPeriod(PIT, static_cast<pit_chnl_t>(get_index()), timer_period); }

        // Get timer period value
        uint32_t get_period() const { return PIT->CHANNEL[get_index()].LDVAL; }

        // Get timer period value (ready to load into LDVAL register) based on supplied rate in microseconds
        static uint32_t convert_us_to_period(const int64_t rate_us)
        {
            return static_cast<uint32_t>(SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK) * rate_us / 1000000UL);
        }

        // Get rate in microseconds based on timer period value
        static int64_t convert_period_to_us(const uint32_t period)
        {
            return (static_cast<int64_t>(period) * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
        }

        // Get the minimum allowed rate in microseconds
        static int64_t get_min_rate_us()
        {
            constexpr int64_t min_period = 0x01;

            return (min_period * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
        }

        // Get the maximum allowed rate in microseconds
        static int64_t get_max_rate_us()
        {
            constexpr int64_t max_period = 0xFFFFFFFF;

            return (max_period * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            clear_irq_pending();

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            return yield;
        }

        // IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const Channel channel)
        {
            const auto index = static_cast<std::size_t>(channel);

            return TimerDriver::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        IrqHandler m_irq_handler;   // User defined IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_TIMER_HPP
