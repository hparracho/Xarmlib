// ----------------------------------------------------------------------------
// @file    FPIO32S-V1-debouncer.cpp
// @brief   FPIO32S-V1 board Debouncer example application.
// @date    20 June 2018
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

#include "xarmlib_config.hpp"

using namespace std::chrono_literals;
using namespace xarmlib;




class SignalNewDebouncedInputed
{
    public:

        SignalNewDebouncedInputed(const std::initializer_list<Pin::Name> input_pin_names,
                                  const std::chrono::milliseconds        sample_count_high,
                                  const std::chrono::milliseconds        sample_count_low,
                                  const Pin::Name                        led_pin_name) : m_group_debounced(input_pin_names),
                                                                                         m_led(led_pin_name, DigitalOut::OutputMode::PUSH_PULL_HIGH)
        {
            // Config group debounced with glitch filter.
            // Filter set to reject pulses smaller than 3 clocks using Input filter clock divider 0.
            Debouncer::config_pins(m_group_debounced,
                                   sample_count_high,
                                   sample_count_low,
                                   Gpio::InputMode::PULL_UP,
                                   Gpio::InputFilter::CLOCKS_3_CLKDIV0);

            // group debounced glitch filter divider set to maximum interval.
            // Input filter clock divider 0 set to 255 (main clock rate / 255).
            Debouncer::set_input_filter_clock_divider(Gpio::InputFilterClockDivider::CLKDIV0, 255);

            // Create and assign the new debounced handler to
            // the SignalNewDebouncedInputed::new_debounced() member function.
            auto handler = Debouncer::NewDebouncedHandler::create<SignalNewDebouncedInputed, &SignalNewDebouncedInputed::new_debounced>(this);
            Debouncer::assign_new_debounced_handler(handler);
        }

        void start(const int32_t timer_irq_priority)
        {
            Debouncer::start(timer_irq_priority);
        }

        void restart()
        {
            Debouncer::restart();
        }

        void stop()
        {
            Debouncer::stop();
        }

    private:

        int32_t new_debounced()
        {
            if(Debouncer::group_debounced(m_group_debounced) == true)
            {
                if(m_group_debounced.read() == m_group_debounced.mask())
                {
                    led_turn_on();
                }
                else
                {
                    led_turn_off();
                }
            }

            return 0;
        }

        void led_turn_on()
        {
            m_led = 0;
        }

        void led_turn_off()
        {
            m_led = 1;
        }

        GroupDebounced  m_group_debounced;
        DigitalOut      m_led;
};




int main(void)
{
    // Config input pins to debounce ('GPIO_ENABLE' and 'GPIO_LATCH' from original FPIO32S-V1 board setup),
    // filter_high = 1ms,
    // filter_low = 1s,
    // and LED pin to signal if those pins are new debounced and inputed ('GPIO_CLOCK' from original FPIO32S-V1 board setup)
    SignalNewDebouncedInputed signal_new_debounced_inputed({ Pin::Name::P0_16, Pin::Name::P0_7 }, 1ms, 1s, Pin::Name::P0_6);

    signal_new_debounced_inputed.start(1);

    while(true)
    {}

    return 0;
}
