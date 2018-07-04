// ----------------------------------------------------------------------------
// @file    api_input_debouncer.hpp
// @brief   API input debouncer helper class.
// @date    3 July 2018
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

#ifndef __XARMLIB_API_INPUT_DEBOUNCER_HPP
#define __XARMLIB_API_INPUT_DEBOUNCER_HPP

#include "system/gsl"

namespace xarmlib
{




class InputDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        struct Input
        {
            int8_t      port_index     { -1 };  // Input's port index
            int8_t      pin_bit        { -1 };  // Input's bit within a port
            int16_t     filter_low_ms  { 0 };   // Milliseconds that a pin must be steady at low level to be accepted as filtered (debounced)
            int16_t     filter_high_ms { 0 };   // Milliseconds that a pin must be steady at high level to be accepted as filtered (debounced)
            int16_t     counter_ms     { 0 };   // Filter time counter
        };

        struct PortMask
        {
            uint32_t    current_read { 0 };     // Current iteration inputs
            uint32_t    last_read    { 0 };     // Previous iteration inputs
            uint32_t    filtered     { 0 };     // Filtered inputs
            uint32_t    sampling     { 0 };     // Inputs that are being sampled and not yet filtered
            uint32_t    enabled      { 0 };     // Enabled inputs
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static bool debounce(const gsl::span<Input>& inputs, const gsl::span<PortMask>& ports)
        {
            bool new_input = false;

            for(auto& input : inputs)
            {
                auto& port = ports[input.port_index];

                const uint32_t pin_mask = (1UL << input.pin_bit);

                const uint32_t current_read_bit = port.current_read & pin_mask;
                const uint32_t last_read_bit    = port.last_read    & pin_mask;

                if(current_read_bit != last_read_bit)
                {
                    // Inputs are different. Reload counter with filter time.
                    input.counter_ms = (current_read_bit == 0) ? input.filter_low_ms : input.filter_high_ms;

                    // Set sampling flag
                    port.sampling |= pin_mask;
                }
                else
                {
                    if(input.counter_ms > 0)
                    {
                        input.counter_ms--;
                    }

                    if(input.counter_ms == 0)
                    {
                        const uint32_t filtered_bit = port.filtered & pin_mask;

                        if(current_read_bit != filtered_bit)
                        {
                            // Update filtered input
                            port.filtered = (port.filtered & (~pin_mask)) | current_read_bit;

                            // Clear sampling flag
                            port.sampling &= ~pin_mask;

                            // Set new input flag
                            new_input = true;
                        }
                    }
                }

                // Update last read input
                port.last_read = (port.last_read & (~pin_mask)) | current_read_bit;
            }

            return new_input;
        }
};




} // namespace xarmlib

#endif // __XARMLIB_API_INPUT_DEBOUNCER_HPP
