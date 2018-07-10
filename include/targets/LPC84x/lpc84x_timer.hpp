// ----------------------------------------------------------------------------
// @file    lpc84x_timer.hpp
// @brief   NXP LPC84x Timer (MRT) class.
// @date    28 June 2018
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

#include "system/cassert"
#include "system/chrono"
#include "system/delegate"
#include "targets/peripheral_ref_counter.hpp"
#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"




// Forward declaration of IRQ handler
extern "C" void MRT_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




// Number of available timer channels
static constexpr std::size_t TIMER_COUNT { 4 };

class Timer : private PeripheralRefCounter<Timer, TIMER_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void ::MRT_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralTimer = PeripheralRefCounter<Timer, TIMER_COUNT>;

        // Timer running mode selection (defined to map the CTRL register directly)
        enum class Mode
        {
            FREE_RUNNING = (0 << 1),    // Re-start at the end of counter
            SINGLE_SHOT  = (1 << 1)     // Stop at the end of counter
        };

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        Timer() : PeripheralTimer(*this)
        {
            // Enable MRT if this is the first timer created
            if(get_used() == 1)
            {
                Clock::enable(Clock::Peripheral::MRT);
                Power::reset(Power::ResetPeripheral::MRT);

                NVIC_EnableIRQ(MRT_IRQn);
            }

            const auto channel_index = get_index();

            // Set pointer to the available channel structure
            m_channel = &LPC_MRT->CHANNEL[channel_index];

            set_interval(0);
            clear_pending_irq();
        }

        ~Timer()
        {
            set_interval(0);
            clear_pending_irq();

            // Disable MRT if this the last timer deleted
            if(get_used() == 1)
            {
                Clock::disable(Clock::Peripheral::MRT);

                NVIC_DisableIRQ(MRT_IRQn);
            }
        }

        // -------- START / STOP ----------------------------------------------

        // Start the timer with the supplied interval and mode
        void start(const std::chrono::microseconds& rate_us, const Mode mode)
        {
            assert(rate_us.count() >= get_min_rate_us());
            assert(rate_us.count() <= get_max_rate_us());

            m_interval = convert_us_to_interval(rate_us.count());

            set_mode(mode);
            set_interval(m_interval);
        }

        // Reload previously set interval and re-start the timer
        void reload()
        {
            // Ensure interval is set
            assert(m_interval != 0);

            set_interval(m_interval);
        }

        // Stop a previously started timer
        void stop()
        {
            set_interval(0);
        }

        bool is_running() const
        {
            return ((m_channel->STAT & STAT_RUN) != 0);
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_irq()
        {
            m_channel->CTRL |= CTRL_INTEN;
        }

        void disable_irq()
        {
            m_channel->CTRL &= ~CTRL_INTEN;
        }

        bool is_enabled_irq() const
        {
            return ((m_channel->CTRL & CTRL_INTEN) != 0);
        }

        bool is_pending_irq() const
        {
            return ((m_channel->STAT & STAT_INTFLAG) != 0);
        }

        void clear_pending_irq()
        {
            m_channel->STAT |= STAT_INTFLAG;
        }

        static void set_mrt_irq_priority(const uint32_t irq_priority)
        {
            NVIC_SetPriority(MRT_IRQn, irq_priority);
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

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void set_mode(const Mode mode)
        {
            m_channel->CTRL = (m_channel->CTRL & ~CTRL_MODE_MASK) | static_cast<uint32_t>(mode);
        }

        // Set timer time interval value
        // NOTE: Setting INTVAL_LOAD bit in timer time interval register
        //       causes the time interval value to load immediately, otherwise
        //       the time interval value will be loaded in next timer cycle.
        void set_interval(const uint32_t timer_interval)
        {
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

            for(std::size_t ch_index = 0; ch_index < TIMER_COUNT; ++ch_index)
            {
                auto* const channel = get_pointer(ch_index);

                if(channel != nullptr)
                {
                    if(channel->is_enabled_irq() && channel->is_pending_irq())
                    {
                        channel->clear_pending_irq();

                        if(channel->m_irq_handler != nullptr)
                        {
                            yield |= channel->m_irq_handler();
                        }
                    }
                }
            }

            return yield;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        LPC_MRT_CHANNEL_T* m_channel  { nullptr };  // Pointer to the individual MRT channel structure
        uint32_t           m_interval { 0 };        // Last loaded interval value

        IrqHandler         m_irq_handler;           // User defined IRQ handler
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_TIMER_HPP
