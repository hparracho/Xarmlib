// ----------------------------------------------------------------------------
// @file    api_input_debouncer.hpp
// @brief   API input debouncer helper class.
// @date    26 June 2018
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

#include "system/dynarray"

namespace xarmlib
{




class InputDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        struct Input
        {
            std::size_t port_index;
            std::size_t bit_index;
            int16_t     sample_count_ms_high;   // Number of samples (in ms) to accept an input as debounced at low level
            int16_t     sample_count_ms_low;    // Number of samples (in ms) to accept an input as debounced at high level
            int16_t     sample_counter;
        };

        struct InputPort
        {
            uint32_t    current_read;
            uint32_t    last_read;
            uint32_t    current_debounced;
            uint32_t    last_debounced;
            uint32_t    sampling;               // Inputs that are being sampled and not yet filtered
        };

        template<std::size_t INPUT_PORTS_SIZE>
        static bool debounce(const dynarray<Input>& inputs, const std::array<InputPort, INPUT_PORTS_SIZE>& input_ports)
        {
            std::array<uint32_t, INPUT_PORTS_SIZE> debounced = { 0 };
            std::array<uint32_t, INPUT_PORTS_SIZE> sampling = { 0 };

            std::size_t port_index = 0;

            for(auto input_port : input_ports)
            {
                debounced[port_index] = input_port.current_debounced;

                port_index++;
            }

            for(auto input : inputs)
            {
                const uint32_t current_read_bit = input_ports[input.port_index].current_read & (1 << input.bit_index);
                const uint32_t last_read_bit    = input_ports[input.port_index].last_read    & (1 << input.bit_index);

                if(current_read_bit != last_read_bit)
                {
                    if(current_read_bit)
                    {
                        // High level

                        // Reset samples
                        input.sample_counter = input.sample_count_ms_high;
                    }
                    else
                    {
                        // Low level

                        // Reset samples
                        input.sample_counter = input.sample_count_ms_low;
                    }
                }

                if(input.sample_counter == 0)
                {
                    // Clear pin mask
                    debounced[input.port_index] &= ~(1UL << input.bit_index);

                    // Input debounced
                    debounced[input.port_index] |= current_read_bit;
                }
                else
                {
                    // Sampling
                    sampling[input.port_index] |= (1UL << input.bit_index);

                    input.sample_counter--;
                }
            }

            bool result = false;

            port_index = 0;

            for(auto input_port : input_ports)
            {
                input_port.last_read = input_port.current_read;

                if(debounced[port_index] != input_port.current_debounced)
                {
                    input_port.last_debounced = input_port.current_debounced;

                    input_port.current_debounced = debounced[port_index];

                    input_port.sampling = sampling[port_index];

                    result = true;
                }

                port_index++;
            }

            return result;
        }
};




} // namespace xarmlib

#endif // __XARMLIB_API_INPUT_DEBOUNCER_HPP
