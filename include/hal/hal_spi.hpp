// ----------------------------------------------------------------------------
// @file    hal_spi.hpp
// @brief   SPI HAL interface classes (SpiMaster / SpiSlave).
// @date    14 July 2018
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

#ifndef __XARMLIB_HAL_SPI_HPP
#define __XARMLIB_HAL_SPI_HPP

#include "external/gsl.hpp"
#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <class TargetSpi>
class SpiMaster : private TargetSpi
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename TargetSpi::SpiMode;
        using DataBits     = typename TargetSpi::DataBits;
        using DataOrder    = typename TargetSpi::DataOrder;
        using LoopbackMode = typename TargetSpi::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMaster(const xarmlib::Pin::Name master_mosi,
                  const xarmlib::Pin::Name master_miso,
                  const xarmlib::Pin::Name master_sck,
                  const int32_t            max_frequency,
                  const SpiMode            spi_mode      = SpiMode::MODE3,
                  const DataBits           data_bits     = DataBits::BITS_8,
                  const DataOrder          data_order    = DataOrder::MSB_FIRST,
                  const LoopbackMode       loopback_mode = LoopbackMode::DISABLED)
       {
            // Initialize peripheral structure and pins
            TargetSpi::initialize(master_mosi, master_miso, master_sck, xarmlib::Pin::Name::NC);
            // Configure data format and operating modes
            TargetSpi::set_configuration(TargetSpi::MasterMode::MASTER, spi_mode, data_bits, data_order, TargetSpi::SselPolarity::LOW, loopback_mode);
            // Set supplied maximum frequency
            TargetSpi::set_frequency(max_frequency);

            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiMaster()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        using TargetSpi::enable;

        // Disable peripheral
        using TargetSpi::disable;

        // Gets the enable state
        using TargetSpi::is_enabled;

        // -------- TRANSFER --------------------------------------------------

        // Transfer a frame (simultaneous write and read)
        // NOTE: Return the read value
        uint32_t transfer(const uint32_t frame)
        {
            write(frame);
            return read();
        }

        // Transfer a buffer (simultaneous write and read)
        // NOTE: The read values will be placed on the same buffer, destroying the original buffer.
        void transfer(const gsl::span<uint8_t> buffer)
        {
            for(auto& frame : buffer)
            {
                frame = transfer(frame);
            }
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const gsl::span<const uint8_t> tx_buffer, const gsl::span<uint8_t> rx_buffer)
        {
            assert(tx_buffer.size() == rx_buffer.size());

            for(std::ptrdiff_t frame_index = 0; frame_index < tx_buffer.size(); ++frame_index)
            {
                rx_buffer[frame_index] = transfer(tx_buffer[frame_index]);
            }
        }

        // -------- ACCESS MUTEX ----------------------------------------------

        void mutex_take()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            xSemaphoreTake(m_rtos_mutex, portMAX_DELAY);
            #endif
        }

        void mutex_give()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            xSemaphoreGive(m_rtos_mutex);
            #endif
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- READ / WRITE ----------------------------------------------

        // Read a frame as soon as possible
        uint32_t read() const
        {
            while(TargetSpi::is_readable() == false);

            return TargetSpi::read_data();
        }

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(TargetSpi::is_writable() == false);

            TargetSpi::write_data(frame);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        #ifdef XARMLIB_ENABLE_FREERTOS
        // FreeRTOS variables
        SemaphoreHandle_t m_rtos_mutex { nullptr };     // Access mutex
        #endif
};




template <class TargetSpi>
class SpiSlave : private TargetSpi
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using SpiMode      = typename TargetSpi::SpiMode;
        using DataBits     = typename TargetSpi::DataBits;
        using DataOrder    = typename TargetSpi::DataOrder;
        using SselPolarity = typename TargetSpi::SselPolarity;
        using LoopbackMode = typename TargetSpi::LoopbackMode;

        using IrqHandler = typename TargetSpi::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlave(const xarmlib::Pin::Name slave_mosi,
                 const xarmlib::Pin::Name slave_miso,
                 const xarmlib::Pin::Name slave_sck,
                 const xarmlib::Pin::Name slave_sel,
                 const int32_t            max_frequency,
                 const SpiMode            spi_mode      = SpiMode::MODE3,
                 const DataBits           data_bits     = DataBits::BITS_8,
                 const DataOrder          data_order    = DataOrder::MSB_FIRST,
                 const SselPolarity       ssel_polarity = SselPolarity::LOW,
                 const LoopbackMode       loopback_mode = LoopbackMode::DISABLED)
        {
            assert(slave_sel != xarmlib::Pin::Name::NC);

            // Initialize peripheral structure and pins
            TargetSpi::initialize(slave_mosi, slave_miso, slave_sck, slave_sel);
            // Configure data format and operating modes
            TargetSpi::set_configuration(TargetSpi::MasterMode::SLAVE, spi_mode, data_bits, data_order, ssel_polarity, loopback_mode);
            // Set supplied maximum frequency
            TargetSpi::set_frequency(max_frequency);

            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiSlave()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        using TargetSpi::enable;

        // Disable peripheral
        using TargetSpi::disable;

        // Gets the enable state
        using TargetSpi::is_enabled;

        // -------- STATUS FLAGS ----------------------------------------------

        using TargetSpi::is_writable;
        using TargetSpi::is_readable;

        using TargetSpi::get_status;
        using TargetSpi::clear_status;

        // -------- INTERRUPTS ------------------------------------------------

        using TargetSpi::enable_interrupts;
        using TargetSpi::disable_interrupts;
        using TargetSpi::get_interrupts_enabled;

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        using TargetSpi::enable_irq;
        using TargetSpi:: disable_irq;
        using TargetSpi::is_irq_enabled;

        using TargetSpi::set_irq_priority;

        using TargetSpi::assign_irq_handler;
        using TargetSpi::remove_irq_handler;

        // -------- READ / WRITE ----------------------------------------------

        // Read a frame as soon as possible
        uint32_t read() const
        {
            while(TargetSpi::is_readable() == false);

            return TargetSpi::read_data();
        }

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(TargetSpi::is_writable() == false);

            TargetSpi::write_data(frame);
        }

        // -------- ACCESS MUTEX ----------------------------------------------

        void mutex_take()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            xSemaphoreTake(m_rtos_mutex, portMAX_DELAY);
            #endif
        }

        void mutex_give()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            xSemaphoreGive(m_rtos_mutex);
            #endif
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        #ifdef XARMLIB_ENABLE_FREERTOS
        // FreeRTOS variables
        SemaphoreHandle_t m_rtos_mutex { nullptr };     // Access mutex
        #endif
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_spi.hpp"

namespace xarmlib
{
using SpiMaster = hal::SpiMaster<targets::lpc84x::Spi>;
using SpiSlave  = hal::SpiSlave <targets::lpc84x::Spi>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_spi.hpp"

namespace xarmlib
{
using SpiMaster = hal::SpiMaster<targets::lpc81x::Spi>;
using SpiSlave  = hal::SpiSlave <targets::lpc81x::Spi>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using SpiMaster = hal::SpiMaster<targets::other_target::Spi>;
using SpiSlave  = hal::SpiSlave <targets::other_target::Spi>;
}

#endif




#endif // __XARMLIB_HAL_SPI_HPP
