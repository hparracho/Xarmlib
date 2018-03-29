// ----------------------------------------------------------------------------
// @file    lpc84x_syscon_clock.hpp
// @brief   NXP LPC84x GPIO class.
// @date    28 March 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_GPIO_HPP
#define __XARMLIB_TARGETS_LPC84X_GPIO_HPP

#include "targets/LPC84x/lpc84x_pins.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




class Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Gpio(const Pin::Name      pin_name,
             const Pin::Direction pin_direction,
             const Pin::Mode      pin_mode) : m_pin     {pin_name},
                                              m_port    {-1},
                                              m_mask    {0}
        {
            if(pin_name != Pin::Name::NC)
            {
                if(pin_name < Pin::Name::P1_0)
                {
                    m_port = 0;
                    m_mask = 1 << static_cast<uint32_t>(pin_name);

                    if(m_gpio0_enabled == false)
                    {
                        m_gpio0_enabled = true;

                        // Enable GPIO port 0
                        Clock::enable(Clock::Peripheral::GPIO0);
                        Power::reset(Power::ResetPeripheral::GPIO0);
                    }
                }
                else
                {
                    m_port = 1;
                    m_mask = 1 << (static_cast<uint32_t>(pin_name) - 32);

                    if(m_gpio1_enabled == false)
                    {
                        m_gpio1_enabled = true;

                        // Enable GPIO port 1
                        Clock::enable(Clock::Peripheral::GPIO1);
                        Power::reset(Power::ResetPeripheral::GPIO1);
                    }
                }

                direction(pin_direction);
                mode(pin_mode);
            }
        }

        void direction(const Pin::Direction pin_direction)
        {
            if(m_pin != Pin::Name::NC)
            {
                switch(pin_direction)
                {
                    case Pin::Direction::INPUT : LPC_GPIO->DIR[m_port] &= ~m_mask; break;
                    case Pin::Direction::OUTPUT: LPC_GPIO->DIR[m_port] |=  m_mask; break;
                }
            }
        }

        void mode(const Pin::Mode pin_mode)
        {
            Pin::mode(m_pin, pin_mode);
        }
        
        void write(const uint32_t value)
        {
            if(m_pin != Pin::Name::NC)
            {
                if(value != 0)
                {
                    LPC_GPIO->SET[m_port] = m_mask;
                }
                else
                {
                    LPC_GPIO->CLR[m_port] = m_mask;
                }
            }
        }

        uint32_t read() const
        {
            if(m_pin != Pin::Name::NC)
            {
                return ((LPC_GPIO->PIN[m_port] & m_mask) != 0) ? 1 : 0;
            }

            return 0;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------
        static bool         m_gpio0_enabled;
        static bool         m_gpio1_enabled;

        const  Pin::Name    m_pin;
                int32_t     m_port;
               uint32_t     m_mask;
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_GPIO_HPP
