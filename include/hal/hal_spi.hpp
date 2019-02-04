// ----------------------------------------------------------------------------
// @file    hal_spi.hpp
// @brief   SPI HAL interface classes (SpiMaster / SpiSlave).
// @date    4 February 2019
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

        using MasterConfig     = typename TargetSpiDriver::MasterConfig;

        using Status           = typename TargetSpiDriver::Status;
        using StatusBitmask    = typename TargetSpiDriver::StatusBitmask;
        using Interrupt        = typename TargetSpiDriver::Interrupt;
        using InterruptBitmask = typename TargetSpiDriver::InterruptBitmask;

        using IrqHandler       = typename TargetSpiDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMasterHal(const xarmlib::PinHal::Name master_mosi,
                     const xarmlib::PinHal::Name master_miso,
                     const xarmlib::PinHal::Name master_sck,
                     const MasterConfig&         master_config) : TargetSpiDriver(master_mosi, master_miso, master_sck, master_config)
       {
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

        inline void enable()           { TargetSpiDriver::enable(); }
        inline void disable()          { TargetSpiDriver::disable(); }
        inline bool is_enabled() const { return TargetSpiDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        inline bool is_writable() const { return TargetSpiDriver::is_writable(); }
        inline bool is_readable() const { return TargetSpiDriver::is_readable(); }

        inline StatusBitmask get_status() const                        { return TargetSpiDriver::get_status(); }
        inline void          clear_status(const StatusBitmask bitmask) { TargetSpiDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        inline void             enable_interrupts (const InterruptBitmask bitmask) { TargetSpiDriver::enable_interrupts(bitmask); }
        inline void             disable_interrupts(const InterruptBitmask bitmask) { TargetSpiDriver::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const                     { return TargetSpiDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        inline void enable_irq()     { TargetSpiDriver::enable_irq(); }
        inline void disable_irq()    { TargetSpiDriver::disable_irq(); }
        inline bool is_irq_enabled() { return TargetSpiDriver::is_irq_enabled(); }

        inline void set_irq_priority(const int32_t irq_priority) { TargetSpiDriver::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { TargetSpiDriver::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler()                              { TargetSpiDriver::remove_irq_handler(); }

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

            TargetSpiDriver::master_write_data(frame);
        }

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

        using SlaveConfig      = typename TargetSpiDriver::SlaveConfig;

        using Status           = typename TargetSpiDriver::Status;
        using StatusBitmask    = typename TargetSpiDriver::StatusBitmask;
        using Interrupt        = typename TargetSpiDriver::Interrupt;
        using InterruptBitmask = typename TargetSpiDriver::InterruptBitmask;

        using IrqHandler       = typename TargetSpiDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlaveHal(const xarmlib::PinHal::Name slave_mosi,
                    const xarmlib::PinHal::Name slave_miso,
                    const xarmlib::PinHal::Name slave_sck,
                    const xarmlib::PinHal::Name slave_sel,
                    const SlaveConfig&          slave_config) : TargetSpiDriver(slave_mosi, slave_miso, slave_sck, slave_sel, slave_config)
        {
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

        inline void enable()           { TargetSpiDriver::enable(); }
        inline void disable()          { TargetSpiDriver::disable(); }
        inline bool is_enabled() const { return TargetSpiDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        inline bool is_writable() const { return TargetSpiDriver::is_writable(); }
        inline bool is_readable() const { return TargetSpiDriver::is_readable(); }

        inline StatusBitmask get_status() const                        { return TargetSpiDriver::get_status(); }
        inline void          clear_status(const StatusBitmask bitmask) { TargetSpiDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        inline void             enable_interrupts (const InterruptBitmask bitmask) { TargetSpiDriver::enable_interrupts(bitmask); }
        inline void             disable_interrupts(const InterruptBitmask bitmask) { TargetSpiDriver::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const                     { return TargetSpiDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        inline void enable_irq()     { TargetSpiDriver::enable_irq(); }
        inline void disable_irq()    { TargetSpiDriver::disable_irq(); }
        inline bool is_irq_enabled() { return TargetSpiDriver::is_irq_enabled(); }

        inline void set_irq_priority(const int32_t irq_priority) { TargetSpiDriver::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { TargetSpiDriver::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler()                              { TargetSpiDriver::remove_irq_handler(); }

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

            TargetSpiDriver::slave_write_data(frame);
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

#if defined __KV4X__

#include "targets/KV4x/kv4x_spi.hpp"

namespace xarmlib
{
using SpiMasterHal = hal::SpiMasterHal<targets::kv4x::SpiDriver>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::kv4x::SpiDriver>;

class SpiMaster : public SpiMasterHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using CtarSelection          = typename SpiMasterHal::CtarSelection;

        using DataBits               = typename SpiMasterHal::DataBits;
        using SpiMode                = typename SpiMasterHal::SpiMode;
        using DataOrder              = typename SpiMasterHal::DataOrder;

        using ContinuousSck          = typename SpiMasterHal::ContinuousSck;
        using ModifiedTransferFormat = typename SpiMasterHal::ModifiedTransferFormat;
        using SamplePoint            = typename SpiMasterHal::SamplePoint;
        using RxFifoOverwrite        = typename SpiMasterHal::RxFifoOverwrite;

        using MasterCtarConfig       = typename SpiMasterHal::MasterCtarConfig;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMaster(const PinHal::Name  master_mosi,
                  const PinHal::Name  master_miso,
                  const PinHal::Name  master_sck,
                  const MasterConfig& master_config) : SpiMasterHal(master_mosi, master_miso, master_sck, master_config)
        {}

        // -------- MASTER CTAR CONFIGURATION ---------------------------------

        // NOTE: do not configure CTAR while the module is in the Running state
        inline void config_master_ctar(const CtarSelection ctar_selection, const MasterCtarConfig& master_ctar_config) { SpiMasterHal::config_master_ctar(ctar_selection, master_ctar_config); }

        // -------- WRITE -----------------------------------------------------

        // Write a frame as soon as possible
        void write(const uint32_t      frame,
                   const CtarSelection ctar_selection = CtarSelection::ctar0,
                   const bool          is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                   const bool          clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            while(SpiMasterHal::is_writable() == false);

            SpiMasterHal::master_write_data(frame, ctar_selection, is_end_of_queue, clear_queue_transfer_counter);
        }

        // -------- TRANSFER --------------------------------------------------

        // Transfer a frame (simultaneous write and read)
        // NOTE: Return the read value
        uint32_t transfer(const uint32_t      frame,
                          const CtarSelection ctar_selection = CtarSelection::ctar0,
                          const bool          is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                          const bool          clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            write(frame, ctar_selection, is_end_of_queue, clear_queue_transfer_counter);
            return SpiMasterHal::read();
        }

        // Transfer a buffer (simultaneous write and read)
        // NOTE: The read values will be placed on the same buffer, destroying the original buffer.
        void transfer(const tcb::span<uint8_t> buffer,
                      const CtarSelection      ctar_selection = CtarSelection::ctar0,
                      const bool               is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                      const bool               clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            for(auto& frame : buffer)
            {
                frame = transfer(frame, ctar_selection, is_end_of_queue, clear_queue_transfer_counter);
            }
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const tcb::span<const uint8_t> tx_buffer,
                      const tcb::span<uint8_t>       rx_buffer,
                      const CtarSelection            ctar_selection = CtarSelection::ctar0,
                      const bool                     is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                      const bool                     clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            assert(tx_buffer.size() == rx_buffer.size());

            for(std::ptrdiff_t frame_index = 0; frame_index < tx_buffer.size(); ++frame_index)
            {
                rx_buffer[frame_index] = transfer(tx_buffer[frame_index], ctar_selection, is_end_of_queue, clear_queue_transfer_counter);
            }
        }

        // Get the transfer counter that indicates the number of SPI transfers made
        // NOTE: this counter is intended to assist in queue management
        inline uint16_t get_queue_transfer_counter() const { return SpiMasterHal::get_queue_transfer_counter(); }

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        inline void flush_rx_fifo() { SpiMasterHal::flush_rx_fifo(); }

        // Flush transmit FIFO counter
        inline void flush_tx_fifo() { SpiMasterHal::flush_tx_fifo(); }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const { return SpiMasterHal::get_rx_fifo_counter(); }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const { return SpiMasterHal::get_tx_fifo_counter(); }
};

class SpiSlave : public SpiSlaveHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using CtarSelection          = typename SpiSlaveHal::CtarSelection;

        using DataBits               = typename SpiSlaveHal::DataBits;
        using SpiMode                = typename SpiSlaveHal::SpiMode;
        using DataOrder              = typename SpiSlaveHal::DataOrder;

        using ContinuousSck          = typename SpiSlaveHal::ContinuousSck;
        using ModifiedTransferFormat = typename SpiSlaveHal::ModifiedTransferFormat;
        using SamplePoint            = typename SpiSlaveHal::SamplePoint;
        using RxFifoOverwrite        = typename SpiSlaveHal::RxFifoOverwrite;

        using SlaveCtarConfig        = typename SpiSlaveHal::SlaveCtarConfig;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlave(const PinHal::Name slave_mosi,
                 const PinHal::Name slave_miso,
                 const PinHal::Name slave_sck,
                 const PinHal::Name slave_sel,
                 const SlaveConfig& slave_config) : SpiSlaveHal(slave_mosi, slave_miso, slave_sck, slave_sel, slave_config)
        {}

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        inline void flush_rx_fifo() { SpiSlaveHal::flush_rx_fifo(); }

        // Flush transmit FIFO counter
        inline void flush_tx_fifo() { SpiSlaveHal::flush_tx_fifo(); }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const { return SpiSlaveHal::get_rx_fifo_counter(); }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const { return SpiSlaveHal::get_tx_fifo_counter(); }
};
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_spi.hpp"

namespace xarmlib
{
using SpiMasterHal = hal::SpiMasterHal<targets::lpc84x::SpiDriver>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::lpc84x::SpiDriver>;

class SpiMaster : public SpiMasterHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename SpiMasterHal::SpiMode;
        using DataBits     = typename SpiMasterHal::DataBits;
        using DataOrder    = typename SpiMasterHal::DataOrder;
        using LoopbackMode = typename SpiMasterHal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMaster(const PinHal::Name  master_mosi,
                  const PinHal::Name  master_miso,
                  const PinHal::Name  master_sck,
                  const MasterConfig& master_config) : SpiMasterHal(master_mosi, master_miso, master_sck, master_config)
       {}
};

class SpiSlave : public SpiSlaveHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename SpiSlaveHal::SpiMode;
        using DataBits     = typename SpiSlaveHal::DataBits;
        using DataOrder    = typename SpiSlaveHal::DataOrder;
        using SselPolarity = typename SpiSlaveHal::SselPolarity;
        using LoopbackMode = typename SpiSlaveHal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlave(const PinHal::Name slave_mosi,
                 const PinHal::Name slave_miso,
                 const PinHal::Name slave_sck,
                 const PinHal::Name slave_sel,
                 const SlaveConfig& slave_config) : SpiSlaveHal(slave_mosi, slave_miso, slave_sck, slave_sel, slave_config)
       {}
};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_spi.hpp"

namespace xarmlib
{
using SpiMasterHal = hal::SpiMasterHal<targets::lpc81x::SpiDriver>;
using SpiSlaveHal  = hal::SpiSlaveHal<targets::lpc81x::SpiDriver>;

class SpiMaster : public SpiMasterHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename SpiMasterHal::SpiMode;
        using DataBits     = typename SpiMasterHal::DataBits;
        using DataOrder    = typename SpiMasterHal::DataOrder;
        using LoopbackMode = typename SpiMasterHal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMaster(const PinHal::Name  master_mosi,
                  const PinHal::Name  master_miso,
                  const PinHal::Name  master_sck,
                  const MasterConfig& master_config) : SpiMasterHal(master_mosi, master_miso, master_sck, master_config)
       {}
};

class SpiSlave : public SpiSlaveHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using SpiMode      = typename SpiSlaveHal::SpiMode;
        using DataBits     = typename SpiSlaveHal::DataBits;
        using DataOrder    = typename SpiSlaveHal::DataOrder;
        using SselPolarity = typename SpiSlaveHal::SselPolarity;
        using LoopbackMode = typename SpiSlaveHal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlave(const PinHal::Name slave_mosi,
                 const PinHal::Name slave_miso,
                 const PinHal::Name slave_sck,
                 const PinHal::Name slave_sel,
                 const SlaveConfig& slave_config) : SpiSlaveHal(slave_mosi, slave_miso, slave_sck, slave_sel, slave_config)
       {}
};
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
