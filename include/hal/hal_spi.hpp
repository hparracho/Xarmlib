// ----------------------------------------------------------------------------
// @file    hal_spi.hpp
// @brief   SPI HAL interface classes (SpiMaster / SpiSlave).
// @date    14 August 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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




template <typename SpiDriver>
class SpiMasterBase : protected SpiDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Config           = typename SpiDriver::MasterConfig;

        using Status           = typename SpiDriver::Status;
        using StatusBitmask    = typename SpiDriver::StatusBitmask;
        using Interrupt        = typename SpiDriver::Interrupt;
        using InterruptBitmask = typename SpiDriver::InterruptBitmask;

        using IrqHandler       = typename SpiDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiMasterBase(const hal::Pin::Name mosi,
                      const hal::Pin::Name miso,
                      const hal::Pin::Name sck,
                      const Config&        config) : SpiDriver(mosi, miso, sck, config)
       {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiMasterBase()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- CONFIGURATION ---------------------------------------------

        int32_t get_frequency() const { return SpiDriver::get_frequency(); }

        // NOTE: during the frequency change, the peripheral will be disabled
        void set_frequency(const int32_t frequency)
        {
            const bool enabled = is_enabled();

            disable();

            SpiDriver::set_frequency(frequency);

            if(enabled == true)
            {
                enable();
            }
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        void enable()           { SpiDriver::enable(); }
        void disable()          { SpiDriver::disable(); }
        bool is_enabled() const { return SpiDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return SpiDriver::is_writable(); }
        bool is_readable() const { return SpiDriver::is_readable(); }

        StatusBitmask get_status() const                        { return SpiDriver::get_status(); }
        void          clear_status(const StatusBitmask bitmask) { SpiDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        void             enable_interrupts (const InterruptBitmask bitmask) { SpiDriver::enable_interrupts(bitmask); }
        void             disable_interrupts(const InterruptBitmask bitmask) { SpiDriver::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const                     { return SpiDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()     { SpiDriver::enable_irq(); }
        void disable_irq()    { SpiDriver::disable_irq(); }
        bool is_irq_enabled() { return SpiDriver::is_irq_enabled(); }

        void set_irq_priority(const int32_t irq_priority) { SpiDriver::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { SpiDriver::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { SpiDriver::remove_irq_handler(); }

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
        void transfer(const std::span<uint8_t> buffer)
        {
            for(auto& frame : buffer)
            {
                frame = static_cast<uint8_t>(transfer(frame));
            }
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const std::span<const uint8_t> tx_buffer, const std::span<uint8_t> rx_buffer)
        {
            assert(tx_buffer.size() == rx_buffer.size());

            for(std::size_t frame_index = 0; frame_index < tx_buffer.size(); ++frame_index)
            {
                rx_buffer[frame_index] = static_cast<uint8_t>(transfer(tx_buffer[frame_index]));
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

    protected:

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Read a frame as soon as possible
        uint32_t read()
        {
            while(SpiDriver::is_readable() == false);

            return SpiDriver::read_data();
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(SpiDriver::is_writable() == false);

            SpiDriver::master_write_data(frame);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        #ifdef XARMLIB_ENABLE_FREERTOS
        // FreeRTOS variables
        SemaphoreHandle_t m_rtos_mutex { nullptr };     // Access mutex
        #endif
};




template <typename SpiDriver>
class SpiSlaveBase : protected SpiDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using Config           = typename SpiDriver::SlaveConfig;

        using Status           = typename SpiDriver::Status;
        using StatusBitmask    = typename SpiDriver::StatusBitmask;
        using Interrupt        = typename SpiDriver::Interrupt;
        using InterruptBitmask = typename SpiDriver::InterruptBitmask;

        using IrqHandler       = typename SpiDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSlaveBase(const hal::Pin::Name mosi,
                     const hal::Pin::Name miso,
                     const hal::Pin::Name sck,
                     const hal::Pin::Name ssel,
                     const Config&        config) : SpiDriver(mosi, miso, sck, ssel, config)
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Create access mutex
            m_rtos_mutex = xSemaphoreCreateMutex();
            #endif
        }

        ~SpiSlaveBase()
        {
            #ifdef XARMLIB_ENABLE_FREERTOS
            // Delete access mutex
            vSemaphoreDelete(m_rtos_mutex);
            #endif
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        void enable()           { SpiDriver::enable(); }
        void disable()          { SpiDriver::disable(); }
        bool is_enabled() const { return SpiDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return SpiDriver::is_writable(); }
        bool is_readable() const { return SpiDriver::is_readable(); }

        StatusBitmask get_status() const                        { return SpiDriver::get_status(); }
        void          clear_status(const StatusBitmask bitmask) { SpiDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        void             enable_interrupts (const InterruptBitmask bitmask) { SpiDriver::enable_interrupts(bitmask); }
        void             disable_interrupts(const InterruptBitmask bitmask) { SpiDriver::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const                     { return SpiDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()     { SpiDriver::enable_irq(); }
        void disable_irq()    { SpiDriver::disable_irq(); }
        bool is_irq_enabled() { return SpiDriver::is_irq_enabled(); }

        void set_irq_priority(const int32_t irq_priority) { SpiDriver::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { SpiDriver::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { SpiDriver::remove_irq_handler(); }

        // -------- READ / WRITE ----------------------------------------------

        // Read a frame as soon as possible
        uint32_t read()
        {
            while(SpiDriver::is_readable() == false);

            return SpiDriver::read_data();
        }

        // Write a frame as soon as possible
        void write(const uint32_t frame)
        {
            while(SpiDriver::is_writable() == false);

            SpiDriver::slave_write_data(frame);
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
namespace hal
{

using SpiMaster = SpiMasterBase<targets::kv4x::SpiDriver>;
using SpiSlave  = SpiSlaveBase<targets::kv4x::SpiDriver>;

} // namespace hal

class SpiMaster : public hal::SpiMaster
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiMaster;

        using DataBits               = typename Hal::DataBits;
        using SpiMode                = typename Hal::SpiMode;
        using DataOrder              = typename Hal::DataOrder;

        using ContinuousSck          = typename Hal::ContinuousSck;
        using ModifiedTransferFormat = typename Hal::ModifiedTransferFormat;
        using SamplePoint            = typename Hal::SamplePoint;
        using RxFifoOverwrite        = typename Hal::RxFifoOverwrite;

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        enum class Frame32Bits
        {
                                       bits_4 = 4, bits_5,  bits_6,  bits_7,  bits_8,
            bits_9,  bits_10, bits_11, bits_12,    bits_13, bits_14, bits_15, bits_16,
            bits_17, bits_18, bits_19, bits_20,    bits_21, bits_22, bits_23, bits_24,
            bits_25, bits_26, bits_27, bits_28,    bits_29, bits_30, bits_31, bits_32
        };

        enum class Frame64Bits
        {
                                       bits_4 = 4, bits_5,  bits_6,  bits_7,  bits_8,
            bits_9,  bits_10, bits_11, bits_12,    bits_13, bits_14, bits_15, bits_16,
            bits_17, bits_18, bits_19, bits_20,    bits_21, bits_22, bits_23, bits_24,
            bits_25, bits_26, bits_27, bits_28,    bits_29, bits_30, bits_31, bits_32,
            bits_33, bits_34, bits_35, bits_36,    bits_37, bits_38, bits_39, bits_40,
            bits_41, bits_42, bits_43, bits_44,    bits_45, bits_46, bits_47, bits_48,
            bits_49, bits_50, bits_51, bits_52,    bits_53, bits_54, bits_55, bits_56,
            bits_57, bits_58, bits_59, bits_60,    bits_61, bits_62, bits_63, bits_64
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- READ / WRITE ----------------------------------------------

        // Read a frame as soon as possible
        uint32_t read() { return Hal::read(); }

        // Write a frame as soon as possible
        void write(const uint32_t frame,
                   const bool     is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                   const bool     clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            write(frame, CtarSelection::ctar0, is_end_of_queue, clear_queue_transfer_counter);
        }

        // Get the transfer counter that indicates the number of SPI transfers made
        // NOTE: this counter is intended to assist in queue management
        uint16_t get_queue_transfer_counter() const { return Hal::get_queue_transfer_counter(); }

        // -------- TRANSFER --------------------------------------------------

        uint32_t transfer(const uint32_t frame)                                                         { return Hal::transfer(frame); }
        void     transfer(const std::span<uint8_t> buffer)                                              { Hal::transfer(buffer); }
        void     transfer(const std::span<const uint8_t> tx_buffer, const std::span<uint8_t> rx_buffer) { Hal::transfer(tx_buffer, rx_buffer); }

        // Transfer a frame (simultaneous write and read) up to 32 bits
        // NOTES: - return the read value
        //        - RX and TX FIFOs will be flushed
        uint32_t transfer(const uint32_t frame, const Frame32Bits frame_bits)
        {
            return transfer_4_to_32bits(frame, frame_bits);
        }

        // Transfer a frame (simultaneous write and read) up to 64 bits
        // NOTES: - return the read value
        //        - RX and TX FIFOs will be flushed
        uint64_t transfer(const uint64_t frame, const Frame64Bits frame_bits)
        {
            if(frame_bits <= Frame64Bits::bits_32)
            {
                return transfer_4_to_32bits(static_cast<uint32_t>(frame), static_cast<Frame32Bits>(frame_bits));
            }

            return transfer_33_to_64bits(frame, frame_bits);
        }

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        void flush_rx_fifo() { Hal::flush_rx_fifo(); }

        // Flush transmit FIFO counter
        void flush_tx_fifo() { Hal::flush_tx_fifo(); }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const { return Hal::get_rx_fifo_counter(); }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const { return Hal::get_tx_fifo_counter(); }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a frame as soon as possible
        void write(const uint32_t      frame,
                   const CtarSelection ctar_selection,
                   const bool          is_end_of_queue = false,
                   const bool          clear_queue_transfer_counter = false)
        {
            while(Hal::is_writable() == false);

            Hal::master_write_data(frame, ctar_selection, is_end_of_queue, clear_queue_transfer_counter);
        }

        uint32_t transfer_4_to_32bits(const uint32_t frame, const Frame32Bits frame_bits)
        {
            const DataBits default_ctar0_data_bits = Hal::get_ctar_data_bits(CtarSelection::ctar0);

            disable();

            flush_tx_fifo();
            flush_rx_fifo();

            uint32_t read_value = 0;

            if(frame_bits <= Frame32Bits::bits_16)
            {
                Hal::set_ctar_data_bits(CtarSelection::ctar0, static_cast<DataBits>(frame_bits));

                write(frame, CtarSelection::ctar0);

                enable();

                read_value = read();
            }
            else
            {
                const auto ctar0_data_bits = static_cast<uint32_t>(frame_bits) / 2;
                const auto ctar1_data_bits = static_cast<uint32_t>(frame_bits) - ctar0_data_bits;

                Hal::set_ctar_data_bits(CtarSelection::ctar0, static_cast<DataBits>(ctar0_data_bits));
                Hal::set_ctar_data_bits(CtarSelection::ctar1, static_cast<DataBits>(ctar1_data_bits));

                const DataOrder data_order = get_data_order(CtarSelection::ctar0);

                if(data_order == DataOrder::msb_first)
                {
                    write(frame >> ctar1_data_bits, CtarSelection::ctar0);
                    write(frame,                    CtarSelection::ctar1);

                    enable();

                    read_value  = read() << ctar1_data_bits;
                    read_value |= read();
                }
                else
                {
                    write(frame,                    CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits, CtarSelection::ctar1);

                    enable();

                    read_value  = read();
                    read_value |= read() << ctar0_data_bits;
                }
            }

            disable();
            Hal::set_ctar_data_bits(CtarSelection::ctar0, default_ctar0_data_bits);
            enable();

            return read_value;
        }

        uint64_t transfer_33_to_64bits(const uint64_t frame, const Frame64Bits frame_bits)
        {
            assert(frame_bits > Frame64Bits::bits_32);

            const DataBits default_ctar0_data_bits = Hal::get_ctar_data_bits(CtarSelection::ctar0);

            const DataOrder data_order = get_data_order(CtarSelection::ctar0);

            disable();

            flush_tx_fifo();
            flush_rx_fifo();

            uint64_t read_value = 0;

            if(frame_bits <= Frame64Bits::bits_48)
            {
                const auto ctar0_data_bits_2x = static_cast<uint32_t>(frame_bits) - 15;

                const auto ctar0_data_bits = ctar0_data_bits_2x / 2;
                const auto ctar1_data_bits = static_cast<uint32_t>(frame_bits) - ctar0_data_bits_2x;

                Hal::set_ctar_data_bits(CtarSelection::ctar0, static_cast<DataBits>(ctar0_data_bits));
                Hal::set_ctar_data_bits(CtarSelection::ctar1, static_cast<DataBits>(ctar1_data_bits));

                if(data_order == DataOrder::msb_first)
                {
                    write(frame >> (ctar0_data_bits + ctar1_data_bits), CtarSelection::ctar0);
                    write(frame >>  ctar1_data_bits,                    CtarSelection::ctar0);
                    write(frame,                                        CtarSelection::ctar1);

                    enable();

                    read_value  = static_cast<uint64_t>(read()) << (ctar0_data_bits + ctar1_data_bits);
                    read_value |= static_cast<uint64_t>(read()) <<  ctar1_data_bits;
                    read_value |= static_cast<uint64_t>(read());
                }
                else
                {
                    write(frame,                       CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits,    CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits_2x, CtarSelection::ctar1);

                    enable();

                    read_value  = static_cast<uint64_t>(read());
                    read_value |= static_cast<uint64_t>(read()) << ctar0_data_bits;
                    read_value |= static_cast<uint64_t>(read()) << ctar0_data_bits_2x;
                }
            }
            else
            {
                const auto ctar0_data_bits_3x = static_cast<uint32_t>(frame_bits) - 14;

                const auto ctar0_data_bits = ctar0_data_bits_3x / 3;
                const auto ctar1_data_bits = static_cast<uint32_t>(frame_bits) - ctar0_data_bits_3x;

                Hal::set_ctar_data_bits(CtarSelection::ctar0, static_cast<DataBits>(ctar0_data_bits));
                Hal::set_ctar_data_bits(CtarSelection::ctar1, static_cast<DataBits>(ctar1_data_bits));

                const auto ctar0_data_bits_2x = ctar0_data_bits + ctar0_data_bits;

                if(data_order == DataOrder::msb_first)
                {
                    write(frame >> (ctar0_data_bits_2x + ctar1_data_bits), CtarSelection::ctar0);
                    write(frame >> (ctar0_data_bits    + ctar1_data_bits), CtarSelection::ctar0);
                    write(frame >>  ctar1_data_bits,                       CtarSelection::ctar0);
                    write(frame,                                           CtarSelection::ctar1);

                    enable();

                    read_value  = static_cast<uint64_t>(read()) << (ctar0_data_bits_2x + ctar1_data_bits);
                    read_value |= static_cast<uint64_t>(read()) << (ctar0_data_bits    + ctar1_data_bits);
                    read_value |= static_cast<uint64_t>(read()) <<  ctar1_data_bits;
                    read_value |= static_cast<uint64_t>(read());
                }
                else
                {
                    write(frame,                       CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits,    CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits_2x, CtarSelection::ctar0);
                    write(frame >> ctar0_data_bits_3x, CtarSelection::ctar1);

                    enable();

                    read_value  = static_cast<uint64_t>(read());
                    read_value |= static_cast<uint64_t>(read()) << ctar0_data_bits;
                    read_value |= static_cast<uint64_t>(read()) << ctar0_data_bits_2x;
                    read_value |= static_cast<uint64_t>(read()) << ctar0_data_bits_3x;
                }
            }

            disable();
            Hal::set_ctar_data_bits(CtarSelection::ctar0, default_ctar0_data_bits);
            enable();

            return read_value;
        }
};

class SpiSlave : public hal::SpiSlave
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiSlave;

        using DataBits               = typename Hal::DataBits;
        using SpiMode                = typename Hal::SpiMode;

        using ContinuousSck          = typename Hal::ContinuousSck;
        using ModifiedTransferFormat = typename Hal::ModifiedTransferFormat;
        using SamplePoint            = typename Hal::SamplePoint;
        using RxFifoOverwrite        = typename Hal::RxFifoOverwrite;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        void flush_rx_fifo() { Hal::flush_rx_fifo(); }

        // Flush transmit FIFO counter
        void flush_tx_fifo() { Hal::flush_tx_fifo(); }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const { return Hal::get_rx_fifo_counter(); }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const { return Hal::get_tx_fifo_counter(); }
};

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_spi.hpp"

namespace xarmlib
{
namespace hal
{

using SpiMaster = SpiMasterBase<targets::lpc84x::SpiDriver>;
using SpiSlave  = SpiSlaveBase<targets::lpc84x::SpiDriver>;

} // namespace hal

class SpiMaster : public hal::SpiMaster
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiMaster;

        using SpiMode      = typename Hal::SpiMode;
        using DataBits     = typename Hal::DataBits;
        using DataOrder    = typename Hal::DataOrder;
        using LoopbackMode = typename Hal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

class SpiSlave : public hal::SpiSlave
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiSlave;

        using SpiMode      = typename Hal::SpiMode;
        using DataBits     = typename Hal::DataBits;
        using DataOrder    = typename Hal::DataOrder;
        using SselPolarity = typename Hal::SselPolarity;
        using LoopbackMode = typename Hal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_spi.hpp"

namespace xarmlib
{
namespace hal
{

using SpiMaster = SpiMasterBase<targets::lpc81x::SpiDriver>;
using SpiSlave  = SpiSlaveBase<targets::lpc81x::SpiDriver>;

} // namespace hal

class SpiMaster : public hal::SpiMaster
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiMaster;

        using SpiMode      = typename Hal::SpiMode;
        using DataBits     = typename Hal::DataBits;
        using DataOrder    = typename Hal::DataOrder;
        using LoopbackMode = typename Hal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

class SpiSlave : public hal::SpiSlave
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::SpiSlave;

        using SpiMode      = typename Hal::SpiMode;
        using DataBits     = typename Hal::DataBits;
        using DataOrder    = typename Hal::DataOrder;
        using SselPolarity = typename Hal::SselPolarity;
        using LoopbackMode = typename Hal::LoopbackMode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using SpiMaster = SpiMasterBase<targets::other_target::SpiDriver>;
using SpiSlave  = SpiSlaveBase<targets::other_target::SpiDriver>;

} // namespace hal

using SpiMaster = hal::SpiMaster;
using SpiSlave  = hal::SpiSlave;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_SPI_HPP
