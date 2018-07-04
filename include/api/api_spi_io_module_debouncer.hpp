// ----------------------------------------------------------------------------
// @file    api_spi_io_module_debouncer.hpp
// @brief   API SPI I/O module (FPIO8SM) debouncer class.
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

#ifndef __XARMLIB_API_SPI_IO_MODULE_DEBOUNCER_HPP
#define __XARMLIB_API_SPI_IO_MODULE_DEBOUNCER_HPP

#include "api/api_input_debouncer.hpp"
#include "api/api_input_scanner.hpp"
#include "api/api_spi_io_module.hpp"
#include "system/array"
#include "system/dynarray"
#include "system/gsl"

namespace xarmlib
{




template<std::size_t IO_MODULE_COUNT_MAX>
class SpiIoModuleDebouncer : private SpiIoModule<IO_MODULE_COUNT_MAX>
{
        using Type = typename SpiIoModule<IO_MODULE_COUNT_MAX>::Type;

    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoModuleDebouncer(      SpiMaster& spi_master,
                             const Pin::Name  latch,
                             const Pin::Name  enable);

//        SpiIoModuleDebouncer(      SpiMaster& spi_master,
//                             const Pin::Name  latch,
//                             const Pin::Name  enable) : SpiIoModule<IO_MODULE_COUNT_MAX>(spi_master, latch, enable),
//                             m_pins(8)
//        {}

        template<Pin::Name... pins>
        void assign_pins(const PinBus<pins...>&          pin_bus,
                         const std::chrono::milliseconds filter_ms_low,
                         const std::chrono::milliseconds filter_ms_high)
        {
//            config_pins(pin_bus, filter_ms_low, filter_ms_high);
        }

        // Update output value to be written in the next transfer
        void update_output(Type output_value)
        {
            m_output_value = output_value;
        }

        bool debounce_handler()
        {
//            for(std::size_t port_index = 0; port_index < Port::COUNT; ++port_index)
//            {
//                m_ports[port_index].current_read = Port::read(static_cast<Port::Name>(port_index));
//            }
//
//            return InputDebouncer::debounce((gsl::span<InputDebouncer::Input> { m_pins }).first(m_assigned_pin_count), m_ports);
            return false;
        }

        // Enable IO module
        using SpiIoModule<IO_MODULE_COUNT_MAX>::enable;

        // Disable IO module
        using SpiIoModule<IO_MODULE_COUNT_MAX>::disable;

        // Gets the enable state
        using SpiIoModule<IO_MODULE_COUNT_MAX>::is_enabled;


    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------



        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<InputDebouncer::PortMask, IO_MODULE_COUNT_MAX> m_ports {};
        dynarray<InputDebouncer::Input>                           m_pins;
        std::size_t                                               m_assigned_pin_count { 0 };

        Type                                                      m_output_value { static_cast<Type>(0xFFFFFFFFFFFFFFFF) };
};




} // namespace xarmlib

#endif // __XARMLIB_API_SPI_IO_MODULE_DEBOUNCER_HPP
