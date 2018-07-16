// ----------------------------------------------------------------------------
// @file    api_pin_scanner.hpp
// @brief   API pin scanner class (takes control of one available Timer).
// @date    16 July 2018
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

#ifndef __XARMLIB_API_PIN_SCANNER_HPP
#define __XARMLIB_API_PIN_SCANNER_HPP

#include "hal/hal_timer.hpp"

#include <dynarray>

namespace xarmlib
{




class PinScanner
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        // Pin source reader/transfer handler definition
        using PinSourceHandlerType = void();
        using PinSourceHandler     = Delegate<PinSourceHandlerType>;

        // Debouncer handler definition
        using DebouncerHandlerType = bool(const bool);
        using DebouncerHandler     = Delegate<DebouncerHandlerType>;

        // Pin change (new input debounced) handler definition
        using PinChangeHandlerType = int32_t();
        using PinChangeHandler     = Delegate<PinChangeHandlerType>;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void add_pin_source_handler(const PinSourceHandler& pin_source_handler)
        {
            assert(pin_source_handler != nullptr);

            // Find an empty slot to store the pin source handler
            auto result = std::find(m_pin_source_handlers.begin(), m_pin_source_handlers.end(), nullptr);

            // Assert if empty slot not found
            assert(result != m_pin_source_handlers.end());

            // Assign pin source handler
            *result = pin_source_handler;
        }

        static void add_debouncer_handler(const DebouncerHandler& debouncer_handler)
        {
            assert(debouncer_handler != nullptr);

            // Find an empty slot to store the debouncer handler
            auto result = std::find(m_debouncer_handlers.begin(), m_debouncer_handlers.end(), nullptr);

            // Assert if empty slot not found
            assert(result != m_debouncer_handlers.end());

            // Assign debouncer handler
            *result = debouncer_handler;
        }

        static void assign_pin_change_handler(const PinChangeHandler& pin_change_handler)
        {
            assert(pin_change_handler != nullptr);

            m_pin_change_handler = pin_change_handler;
        }

        static void remove_pin_change_handler()
        {
            m_pin_change_handler = nullptr;
        }

        static void start(const std::chrono::milliseconds scan_time)
        {
            if(m_timer.is_running() == false)
            {
                const auto handler = Timer::IrqHandler::create<&timer_irq_handler>();
                m_timer.assign_irq_handler(handler);
                m_timer.enable_irq();
            }

            // Start or restart the timer with the new specified scan time
            m_timer.start(scan_time, Timer::Mode::FREE_RUNNING);
        }

        static void stop()
        {
            m_timer.stop();

            while(m_timer.is_pending_irq() == true);

            m_timer.disable_irq();
            m_timer.remove_irq_handler();

            m_is_starting = true;
        }

        static void resume()
        {
            if(m_timer.is_running() == false)
            {
                const auto handler = Timer::IrqHandler::create<&timer_irq_handler>();
                m_timer.assign_irq_handler(handler);
                m_timer.enable_irq();
                m_timer.reload();
            }
        }

        static bool is_running()
        {
            return m_timer.is_running();
        }

#ifdef __TARGET_TIMER_TYPE_IS_MRT__
        // NOTE: Timer type is a multi-rate timer (single timer with multiple channels).
        //       Only one IRQ and one priority available for all channels.
        static void set_mrt_irq_priority(const int32_t irq_priority)
        {
            Timer::set_mrt_irq_priority(irq_priority);
        }
#else
        // NOTE: Timer type is an independent timer (multiple individual timers).
        //       Each timer have their own IRQ with different priorities.
        static void set_timer_irq_priority(const int32_t irq_priority)
        {
            m_timer.set_irq_priority(irq_priority);
        }
#endif

    private:

        static int32_t timer_irq_handler()
        {
            for(const auto& handler : m_pin_source_handlers)
            {
                if(handler != nullptr)
                {
                    handler();
                }
            }

            bool new_input = m_is_starting;

            for(const auto& handler : m_debouncer_handlers)
            {
                if(handler != nullptr && handler(m_is_starting) == true)
                {
                    new_input = true;
                }
            }

            int32_t yield = 0;  // Used in FreeRTOS

            if(new_input == true && m_pin_change_handler != nullptr)
            {
                yield = m_pin_change_handler();
            }

            m_is_starting = false;

            return yield;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        static Timer                           m_timer;
        static std::dynarray<PinSourceHandler> m_pin_source_handlers;
        static std::dynarray<DebouncerHandler> m_debouncer_handlers;
        static PinChangeHandler                m_pin_change_handler;
        static bool                            m_is_starting;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_SCANNER_HPP
