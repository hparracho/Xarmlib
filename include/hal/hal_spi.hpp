// ----------------------------------------------------------------------------
// @file    hal_spi.hpp
// @brief   SPI HAL interface classes (SpiMaster / SpiSlave).
// @date    11 January 2019
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

#include "external/span.hpp"
#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <typename TargetSpiDriver>
class SpiMasterHal : protected TargetSpiDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename TargetSpiDriver::SpiMode;
        using DataBits     = typename TargetSpiDriver::DataBits;
        using DataOrder    = typename TargetSpiDriver::DataOrder;
        using LoopbackMode = typename TargetSpiDriver::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMasterHal(const xarmlib::PinHal::Name master_mosi,
                     const xarmlib::PinHal::Name master_miso,
                     const xarmlib::PinHal::Name master_sck,
                     const int32_t               max_frequency,
                     const SpiMode               spi_mode      = SpiMode::MODE3,
                     const DataBits              data_bits     = DataBits::BITS_8,
                     const DataOrder             data_order    = DataOrder::MSB_FIRST,
                     const LoopbackMode          loopback_mode = LoopbackMode::DISABLED)
       {
            // Initialize peripheral structure and pins
            TargetSpiDriver::initialize(master_mosi, master_miso, master_sck, xarmlib::PinHal::Name::nc);
            // Configure data format and operating modes
            TargetSpiDriver::set_configuration(TargetSpiDriver::MasterMode::MASTER, spi_mode, data_bits, data_order, TargetSpiDriver::SselPolarity::LOW, loopback_mode);
            // Set supplied maximum frequency
            TargetSpiDriver::set_frequency(max_frequency);

            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiMasterHal()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        using TargetSpiDriver::enable;

        // Disable peripheral
        using TargetSpiDriver::disable;

        // Gets the enable state
        using TargetSpiDriver::is_enabled;

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
        void transfer(const tcb::span<uint8_t> buffer)
        {
            for(auto& frame : buffer)
            {
                frame = transfer(frame);
            }
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const tcb::span<const uint8_t> tx_buffer, const tcb::span<uint8_t> rx_buffer)
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
            while(TargetSpiDriver::is_readable() == false);

            return TargetSpiDriver::read_data();
        }

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(TargetSpiDriver::is_writable() == false);

            TargetSpiDriver::write_data(frame);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        #ifdef XARMLIB_ENABLE_FREERTOS
        // FreeRTOS variables
        SemaphoreHandle_t m_rtos_mutex { nullptr };     // Access mutex
        #endif
};




template <typename TargetSpiDriver>
class SpiSlaveHal : protected TargetSpiDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using SpiMode      = typename TargetSpiDriver::SpiMode;
        using DataBits     = typename TargetSpiDriver::DataBits;
        using DataOrder    = typename TargetSpiDriver::DataOrder;
        using SselPolarity = typename TargetSpiDriver::SselPolarity;
        using LoopbackMode = typename TargetSpiDriver::LoopbackMode;

        using IrqHandler = typename TargetSpiDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlaveHal(const xarmlib::PinHal::Name slave_mosi,
                    const xarmlib::PinHal::Name slave_miso,
                    const xarmlib::PinHal::Name slave_sck,
                    const xarmlib::PinHal::Name slave_sel,
                    const int32_t               max_frequency,
                    const SpiMode               spi_mode      = SpiMode::MODE3,
                    const DataBits              data_bits     = DataBits::BITS_8,
                    const DataOrder             data_order    = DataOrder::MSB_FIRST,
                    const SselPolarity          ssel_polarity = SselPolarity::LOW,
                    const LoopbackMode          loopback_mode = LoopbackMode::DISABLED)
        {
            assert(slave_sel != xarmlib::PinHal::Name::nc);

            // Initialize peripheral structure and pins
            TargetSpiDriver::initialize(slave_mosi, slave_miso, slave_sck, slave_sel);
            // Configure data format and operating modes
            TargetSpiDriver::set_configuration(TargetSpiDriver::MasterMode::SLAVE, spi_mode, data_bits, data_order, ssel_polarity, loopback_mode);
            // Set supplied maximum frequency
            TargetSpiDriver::set_frequency(max_frequency);

            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiSlaveHal()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        using TargetSpiDriver::enable;

        // Disable peripheral
        using TargetSpiDriver::disable;

        // Gets the enable state
        using TargetSpiDriver::is_enabled;

        // -------- STATUS FLAGS ----------------------------------------------

        using TargetSpiDriver::is_writable;
        using TargetSpiDriver::is_readable;

        using TargetSpiDriver::get_status;
        using TargetSpiDriver::clear_status;

        // -------- INTERRUPTS ------------------------------------------------

        using TargetSpiDriver::enable_interrupts;
        using TargetSpiDriver::disable_interrupts;
        using TargetSpiDriver::get_interrupts_enabled;

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        using TargetSpiDriver::enable_irq;
        using TargetSpiDriver:: disable_irq;
        using TargetSpiDriver::is_irq_enabled;

        using TargetSpiDriver::set_irq_priority;

        using TargetSpiDriver::assign_irq_handler;
        using TargetSpiDriver::remove_irq_handler;

        // -------- READ / WRITE ----------------------------------------------

        // Read a frame as soon as possible
        uint32_t read() const
        {
            while(TargetSpiDriver::is_readable() == false);

            return TargetSpiDriver::read_data();
        }

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(TargetSpiDriver::is_writable() == false);

            TargetSpiDriver::write_data(frame);
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
using SpiMasterHal = hal::SpiMasterHal<targets::lpc84x::Spi>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::lpc84x::Spi>;
//using SpiMaster = SpiMasterHal;
//using SpiSlave  = SpiSlaveHal;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_spi.hpp"

namespace xarmlib
{
using SpiMasterHal = hal::SpiMasterHal<targets::lpc81x::Spi>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::lpc81x::Spi>;
//using SpiMaster = SpiMasterHal;
//using SpiSlave  = SpiSlaveHal;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using SpiMasterHal = hal::SpiMasterHal<targets::other_target::SpiDriver>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::other_target::SpiDriver>;
using SpiMaster = SpiMasterHal;
using SpiSlave  = SpiSlaveHal;
}

#endif




#endif // __XARMLIB_HAL_SPI_HPP
