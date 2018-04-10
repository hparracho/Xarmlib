// ----------------------------------------------------------------------------
// @file    lpc84x_timer.hpp
// @brief   NXP LPC84x Timer (MRT) class.
// @date    7 April 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_TIMER_HPP
#define __XARMLIB_TARGETS_LPC84X_TIMER_HPP

#include <cassert>
#include "system/chrono"
#include "system/delegate"
#include "targets/LPC84x/lpc84x_cmsis.h"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




// Forward declaration of IRQ handler
extern "C" void MRT_IRQHandler(void);




class Timer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Friends IRQ handler function to give it access to private members
        friend void MRT_IRQHandler(void);

        using CallbackType = int32_t();
        using Callback     = Delegate<CallbackType>;

        // Timer running mode selection (defined to map the CTRL register directly)
        enum class Mode
        {
            FREE_RUNNING = (0 << 1),    // Re-start at the end of counter
            SINGLE_SHOT  = (1 << 1)     // Stop at the end of counter
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Timer()
        {
            // Initialize MRT if this is the first instantiation
            if(m_timers_used == 0)
            {
                Clock::enable(Clock::Peripheral::MRT);
                Power::reset(Power::ResetPeripheral::MRT);

                NVIC_EnableIRQ(MRT_IRQn);
            }

            m_channel_index = get_available_channel(m_timers_used);
            assert(m_channel_index != -1);

            // Set pointer to the available channel structure
            m_channel = &LPC_MRT->CHANNEL[m_channel_index];

            set_interval(0);
            clear_irq_pending();

            // Set used timer channel static variables
            m_timers_used |= (1 << m_channel_index);
            m_timers_array[m_channel_index] = this;
        }

        ~Timer()
        {
            set_interval(0);
            clear_irq_pending();

            // Clear used timer channel static variables
            m_timers_used &= ~(1 << m_channel_index);
            m_timers_array[m_channel_index] = nullptr;

            // All timers deleted; disable peripheral
            if(m_timers_used == 0)
            {
                Clock::disable(Clock::Peripheral::MRT);

                NVIC_DisableIRQ(MRT_IRQn);
            }
        }

        void start(const std::chrono::microseconds& rate_us, const Mode mode)
        {
            assert(rate_us.count() >= get_min_rate_us());
            assert(rate_us.count() <= get_max_rate_us());

            m_interval = convert_us_to_interval(rate_us.count());

            set_mode(mode);
            set_interval(m_interval);
        }

        void reload()
        {
            set_interval(m_interval);
        }

        void stop()
        {
            m_interval = 0;
            set_interval(0);
        }

        bool is_running() const
        {
            return ((m_channel->STAT & STAT_RUN) != 0);
        }

        bool is_irq_pending() const
        {
            return ((m_channel->STAT & STAT_INTFLAG) != 0);
        }

        void clear_irq_pending()
        {
            m_channel->STAT |= STAT_INTFLAG;
        }

        bool is_irq_enabled() const
        {
            return ((m_channel->CTRL & CTRL_INTEN) != 0);
        }

        static void set_mrt_priority(const int32_t priority)
        {
            NVIC_SetPriority(MRT_IRQn, priority);
        }

        void assign_callback(const Callback& callback)
        {
            assert(callback != nullptr);

            m_callback = callback;
            enable_irq();
        }

        void remove_callback()
        {
            disable_irq();
            m_callback = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr int32_t get_available_channel(const uint32_t timers_used)
        {
            for(std::size_t channel = 0; channel < NUM_CHANNELS; ++channel)
            {
                if((timers_used & (1 << channel)) == 0)
                {
                    return channel;
                }
            }

            return -1;
        }

        void set_mode(const Mode mode)
        {
            m_channel->CTRL = (m_channel->CTRL & ~CTRL_MODE_MASK) | static_cast<uint32_t>(mode);
        }

        void enable_irq()
        {
            m_channel->CTRL |= CTRL_INTEN;
        }

        void disable_irq()
        {
            m_channel->CTRL &= ~CTRL_INTEN;
        }

        // Set timer interval value
        void set_interval(const uint32_t timer_interval)
        {
            // Sets the timer time interval value
            // NOTE: Setting INTVAL_LOAD bit in timer time interval register
            //       causes the time interval value to load immediately, otherwise
            //       the time interval value will be loaded in next timer cycle.
            m_channel->INTVAL = timer_interval | INTVAL_LOAD;
        }

        // Get timer interval value (ready to load into INTVAL register) based on supplied rate in microseconds
        static uint32_t convert_us_to_interval(const int64_t rate_us)
        {
            return static_cast<uint32_t>(SystemCoreClock * rate_us / 1000000UL);
        }

        // Get the minimum allowed rate in microseconds
        static int64_t get_min_rate_us()
        {
            const int64_t min_interval = 0x01;

            return (min_interval * 1000000UL / SystemCoreClock);
        }

        // Get the maximum allowed rate in microseconds
        static int64_t get_max_rate_us()
        {
            const int64_t max_interval = 0x7FFFFFFF;

            return (max_interval * 1000000UL / SystemCoreClock);
        }

        // IRQ handler for all channels
        // NOTE: Returns yield flag for FreeRTOS
        inline __attribute__((always_inline))
        static int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            for(auto& channel : m_timers_array)
            {
                if(channel != nullptr)
                {
                    if(channel->is_irq_enabled() && channel->is_irq_pending())
                    {
                        channel->clear_irq_pending();

                        if(channel->m_callback != nullptr)
                        {
                            yield |= channel->m_callback();
                        }
                    }
                }
            }

            return yield;
        }

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // MRT Time Interval Register (INTVAL) bits
        enum INTVAL : uint32_t
        {
            INTVAL_LOAD         = (1UL << 31)
        };

        // MRT Control Register (CTRL) bits and masks
        enum CTRL : uint32_t
        {
            CTRL_INTEN          = (1 << 0),
            CTRL_MODE_REPEAT    = (0 << 1),     // Free running
            CTRL_MODE_ONE_SHOT  = (1 << 1),     // Single shot
            CTRL_MODE_MASK      = (3 << 1)
        };

        // MRT Status Register (STAT) bits
        enum STAT : uint32_t
        {
            STAT_INTFLAG        = (1 << 0),
            STAT_RUN            = (1 << 1)
        };

        int32_t             m_channel_index { -1 };
        LPC_MRT_CHANNEL_T*  m_channel       { nullptr };    // Pointer to the individual MRT channel structure
        uint32_t            m_interval      { 0 };          // Last loaded interval value

        Callback            m_callback;                     // Callback handler that will be invoked at the IRQ for this channel

        // The static variables below are used to control the
        // instantiation of the available channels automatically.
        static constexpr std::size_t            NUM_CHANNELS { 4 };     // Number of available channels
        static std::array<Timer*, NUM_CHANNELS> m_timers_array;         // { nullptr }
        static uint32_t                         m_timers_used;          // { 0 }
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_TIMER_HPP
