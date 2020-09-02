// ----------------------------------------------------------------------------
// @file    LPC800-Mini-Kit-blinky.cpp
// @brief   LPC800-Mini-Kit board Blinky example application.
// @date    2 September 2020
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

#include "xarmlib_config.hpp"
#include "xarmlib.hpp"

using namespace std::chrono_literals;
using namespace xarmlib;




class Blinker : public Timer
{
    public:

        explicit Blinker(const Pin::Name pin_name) : m_output(pin_name, Gpio::OutputModeConfig{Gpio::OutputMode::push_pull_high})
        {
            // Create and assign the timer IRQ handler to the Blinker::toggle() member function
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
            m_output = !m_output;

            return 0;
        }

        DigitalOut m_output;
};




int main(void)
{
    // ------------------------------------------------------------------------
    // Demo will only run in release mode while SWD is disabled and
    // with the programmer disconnected from the board!
    // ------------------------------------------------------------------------

    // Configure ISP button with glitch filter. Filter set to reject
    // pulses smaller than 3 clocks using Input filter clock divider 0.
    DigitalIn button(Pin::Name::p0_1, Gpio::InputModeConfig{Gpio::InputMode::pull_up, Gpio::InputFilter::clocks_3_clkdiv0});

    // Button glitch filter divider set to maximum interval.
    // Input filter clock divider 0 set to 255 (main clock rate / 255).
    Gpio::set_input_filter_clock_divider(Gpio::InputFilterClockDivider::clkdiv0, 255);

    // Configure LED blinker
    static Blinker led_blue(Pin::Name::p0_2);

    // Set the flag to initialize the blinker @ 1s rate
    bool button_pressed = true;

    while(true)
    {
        // ISP button pressed
        if(button == 0 && button_pressed == false)
        {
            button_pressed = true;

            // Blink the LED @ 125ms rate while the ISP button is pressed
            led_blue.start(125ms, Timer::Mode::free_running);
        }

        // ISP button released
        if(button == 1 && button_pressed == true)
        {
            button_pressed = false;

            // Blink the LED @ 1s rate while the ISP button is released
            led_blue.start(1s, Timer::Mode::free_running);
        }
    }

    return 0;
}
