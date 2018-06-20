// ----------------------------------------------------------------------------
// @file    FPIO32S-V1-blinky.cpp
// @brief   FPIO32S-V1 board Blinky example application.
// @date    18 June 2018
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




class Blinker : public Timer
{
    public:

        Blinker(const Pin::Name pin_name) : m_output(pin_name, DigitalOut::OutputMode::PUSH_PULL_HIGH)
        {
            // Create and assign the timer IRQ handler to
            // the Blinker::toggle() member function.
            auto handler = Timer::IrqHandler::create<Blinker, &Blinker::toggle>(this);
            assign_irq_handler(handler);

            enable_irq();
        }

        void turn_on()
        {
            m_output = 0;
        }

        void turn_off()
        {
            m_output = 1;
        }

        bool is_on()
        {
            return (m_output == 0);
        }

        bool is_off()
        {
            return (m_output == 1);
        }

    private:

        int32_t toggle()
        {
            // Toggle pin
            m_output = !m_output.read();

            return 0;
        }

        DigitalOut m_output;
};




int main(void)
{
    // Config "button" ('GPIO_ENABLE' from original FPIO32S-V1 board setup) with glitch filter.
    // Filter set to reject pulses smaller than 3 clocks using Input filter clock divider 0.
    DigitalIn button(Pin::Name::P0_16,
                     DigitalIn::InputMode::PULL_UP,
                     DigitalIn::InputFilter::CLOCKS_3_CLKDIV0);

    // Button glitch filter divider set to maximum interval.
    // Input filter clock divider 0 set to 255 (main clock rate / 255).
    DigitalIn::set_input_filter_clock_divider(DigitalIn::InputFilterClockDivider::CLKDIV0, 255);

    // Configure LED blinker ('GPIO_CLOCK' from original FPIO32S-V1 board setup)
    Blinker led(Pin::Name::P0_6);

    // Turn on the LED
    led.turn_on();

    bool button_pressed = false;

    while(true)
    {
        // When the "button" is released turn off the
        // LED and turn it on again after 3 seconds.
        if(button.read() == 0 && button_pressed == false)
        {
            button_pressed = true;
        }

        if(button.read() == 1 && button_pressed == true)
        {
            button_pressed = false;

            led.turn_off();
            led.start(3s, Timer::Mode::SINGLE_SHOT);
        }
    }

    return 0;
}
