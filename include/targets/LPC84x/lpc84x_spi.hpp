// ----------------------------------------------------------------------------
// @file    lpc84x_spi.hpp
// @brief   NXP LPC84x SPI class.
// @date    9 April 2019
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

#ifndef __XARMLIB_TARGETS_LPC84X_SPI_HPP
#define __XARMLIB_TARGETS_LPC84X_SPI_HPP

#include "external/bitmask.hpp"
#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <array>




// Forward declaration of IRQ handlers
extern "C" void SPI0_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




namespace private_spi
{

// SPI Status Register (STAT) bits
enum class Status
{
    rx_ready          = (1 << 0), // Receiver Ready flag
    tx_ready          = (1 << 1), // Transmitter Ready flag
    rx_overrun        = (1 << 2), // Receiver Overrun interrupt flag (applies only to slave mode)
    tx_underrun       = (1 << 3), // Transmitter Underrun interrupt flag (applies only to slave mode)
    ssel_assert       = (1 << 4), // Slave Select Assert flag
    ssel_deassert     = (1 << 5), // Slave Select Deassert flag
    stalled           = (1 << 6), // Stalled status flag
    end_transfer      = (1 << 7), // End Transfer control bit
    master_idle       = (1 << 8), // Master idle status flag
    clear_all_bitmask = 0x0BC,    // Clear all bitmask (0'1011'1100)
    bitmask           = 0x1FF     // Full bitmask (1'1111'1111)
};

// SPI Interrupt Register (INTENSET and INTENCLR) bits
enum class Interrupt
{
    rx_ready      = (1 << 0), // Receiver data is available
    tx_ready      = (1 << 1), // Transmitter holding register is available
    rx_overrun    = (1 << 2), // Receiver overrun occurs (applies only to slave mode)
    tx_underrun   = (1 << 3), // Transmitter underrun occurs (applies only to slave mode)
    ssel_assert   = (1 << 4), // Slave Select is asserted
    ssel_deassert = (1 << 5), // Slave Select is deasserted
    bitmask       = 0x3F      // Full bitmask (11'1111)
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_spi




class SpiDriver : private PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void ::SPI0_IRQHandler(void);
        friend void ::SPI1_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralSpi = PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT>;

        // Master modes selection (defined to map the CFG register directly)
        enum class MasterMode
        {
            slave  = (0 << 2),
            master = (1 << 2)
        };

        // Numbered operating mode selection (defined to map the CFG register directly)
        enum class SpiMode
        {
            //        CPOL   |   CPHA
            mode0 = (0 << 5) | (0 << 4),    // CPOL = 0 | CPHA = 0
            mode1 = (0 << 5) | (1 << 4),    // CPOL = 0 | CPHA = 1
            mode2 = (1 << 5) | (0 << 4),    // CPOL = 1 | CPHA = 0
            mode3 = (1 << 5) | (1 << 4)     // CPOL = 1 | CPHA = 1
        };

        // Data length selection (defined to map the TXCTL register directly)
        enum class DataBits
        {
            bit_1   = (0x00 << 24),
            bits_2  = (0x01 << 24),
            bits_3  = (0x02 << 24),
            bits_4  = (0x03 << 24),
            bits_5  = (0x04 << 24),
            bits_6  = (0x05 << 24),
            bits_7  = (0x06 << 24),
            bits_8  = (0x07 << 24),
            bits_9  = (0x08 << 24),
            bits_10 = (0x09 << 24),
            bits_11 = (0x0a << 24),
            bits_12 = (0x0b << 24),
            bits_13 = (0x0c << 24),
            bits_14 = (0x0d << 24),
            bits_15 = (0x0e << 24),
            bits_16 = (0x0f << 24)
        };

        // Data order selection (defined to map the CFG register directly)
        enum class DataOrder
        {
            msb_first = (0 << 3),
            lsb_first = (1 << 3)
        };

        // Slave select polarity selection (defined to map the CFG register directly)
        enum class SselPolarity
        {
            low  = (0 << 8),
            high = (1 << 8)
        };

        // Loopback mode selection (defined to map the CFG register directly)
        enum class LoopbackMode
        {
            disabled = (0 << 7),
            enabled  = (1 << 7)
        };

        struct MasterConfig
        {
            int32_t      max_frequency = 500000;
            SpiMode      spi_mode      = SpiMode::mode3;
            DataBits     data_bits     = DataBits::bits_8;
            DataOrder    data_order    = DataOrder::msb_first;
            LoopbackMode loopback_mode = LoopbackMode::disabled;
        };

        struct SlaveConfig
        {
            int32_t      max_frequency = 500000;
            SpiMode      spi_mode      = SpiMode::mode3;
            DataBits     data_bits     = DataBits::bits_8;
            DataOrder    data_order    = DataOrder::msb_first;
            SselPolarity ssel_polarity = SselPolarity::low;
            LoopbackMode loopback_mode = LoopbackMode::disabled;
        };

        // Type safe accessor to STAT register
        using Status        = private_spi::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to INTENSET / INTENCLR registers
        using Interrupt        = private_spi::Interrupt;
        using InterruptBitmask = bitmask::bitmask<Interrupt>;

        // IRQ handler definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        // Master mode constructor
        SpiDriver(const PinDriver::Name master_mosi,
                  const PinDriver::Name master_miso,
                  const PinDriver::Name master_sck,
                  const MasterConfig&   master_config) : PeripheralSpi(*this)
       {
            // Initialize peripheral structure and pins
            initialize(master_mosi, master_miso, master_sck, PinDriver::Name::nc);

            // Configure data format and operating modes
            set_configuration(MasterMode::master, master_config.spi_mode, master_config.data_bits, master_config.data_order, SselPolarity::low, master_config.loopback_mode);

            // Set supplied maximum frequency
            set_frequency(master_config.max_frequency);
        }

        // Slave mode constructor
        SpiDriver(const PinDriver::Name slave_mosi,
                  const PinDriver::Name slave_miso,
                  const PinDriver::Name slave_sck,
                  const PinDriver::Name slave_sel,
                  const SlaveConfig&    slave_config) : PeripheralSpi(*this)
        {
            assert(slave_sel != PinDriver::Name::nc);

            // Initialize peripheral structure and pins
            initialize(slave_mosi, slave_miso, slave_sck, slave_sel);

            // Configure data format and operating modes
            set_configuration(MasterMode::slave, slave_config.spi_mode, slave_config.data_bits, slave_config.data_order, slave_config.ssel_polarity, slave_config.loopback_mode);

            // Set supplied maximum frequency
            set_frequency(slave_config.max_frequency);
        }

        ~SpiDriver()
        {
            // Disable peripheral
            disable();

            const Name name = static_cast<Name>(get_index());

            // Disable peripheral clock sources and interrupts
            switch(name)
            {
                case Name::spi0: ClockDriver::disable(ClockDriver::Peripheral::spi0); NVIC_DisableIRQ(SPI0_IRQn); break;
                case Name::spi1: ClockDriver::disable(ClockDriver::Peripheral::spi1); NVIC_DisableIRQ(SPI1_IRQn); break;
                default:                                                                                          break;
            }
        }

        // -------- INITIALIZATION / CONFIGURATION ----------------------------

        // Initialize SPI peripheral structure and pins (constructor helper function)
        void initialize(const PinDriver::Name mosi,
                        const PinDriver::Name miso,
                        const PinDriver::Name sck,
                        const PinDriver::Name slave_sel)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::spi0:
                {
                    // Set pointer to the available SPI structure
                    m_spi = LPC_SPI0;

                    ClockDriver::enable(ClockDriver::Peripheral::spi0);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::spi0);

                    ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::spi0,
                                                             ClockDriver::PeripheralClockSource::main_clk);

                    SwmDriver::assign(SwmDriver::PinMovable::spi0_mosi_io, mosi);
                    SwmDriver::assign(SwmDriver::PinMovable::spi0_miso_io, miso);
                    SwmDriver::assign(SwmDriver::PinMovable::spi0_sck_io, sck);

                    if(slave_sel != PinDriver::Name::nc)
                    {
                        SwmDriver::assign(SwmDriver::PinMovable::spi0_ssel0_io, slave_sel);
                    }
                }   break;

                case Name::spi1:
                {
                    // Set pointer to the available SPI structure
                    m_spi = LPC_SPI1;

                    ClockDriver::enable(ClockDriver::Peripheral::spi1);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::spi1);

                    ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::spi1,
                                                             ClockDriver::PeripheralClockSource::main_clk);

                    SwmDriver::assign(SwmDriver::PinMovable::spi1_mosi_io, mosi);
                    SwmDriver::assign(SwmDriver::PinMovable::spi1_miso_io, miso);
                    SwmDriver::assign(SwmDriver::PinMovable::spi1_sck_io, sck);

                    if(slave_sel != PinDriver::Name::nc)
                    {
                        SwmDriver::assign(SwmDriver::PinMovable::spi1_ssel0_io, slave_sel);
                    }
                }   break;

                default: break;
            }
        }

        // Configure SPI data format and operating modes (constructor helper function)
        void set_configuration(const MasterMode   master_mode,
                               const SpiMode      spi_mode,
                               const DataBits     data_bits,
                               const DataOrder    data_order,
                               const SselPolarity ssel_polarity,
                               const LoopbackMode loopback_mode)
        {
            // Set the MASTER, LSBF, CPHA, CPOL, LOOP and SPOL0 bits
            m_spi->CFG = static_cast<uint32_t>(master_mode  ) |
                         static_cast<uint32_t>(data_order   ) |
                         static_cast<uint32_t>(spi_mode     ) |
                         static_cast<uint32_t>(loopback_mode) |
                         static_cast<uint32_t>(ssel_polarity);

            // Set the LEN bits
            m_spi->TXCTL = static_cast<uint32_t>(data_bits);

            // Configure the SPI delay register (DLY)
            // Pre-delay = 0 clocks / post-delay = 0 clocks / frame-delay = 0 clocks / transfer-delay = 0 clocks
            m_spi->DLY = 0x0000;

            // Clear all interrupt flags
            m_spi->INTENCLR = 0x3F;
        }

        // Set SPI maximum frequency (constructor helper function)
        // NOTE: If the maximum frequency cannot be obtained it will set
        //       the closest frequency that is below the target frequency.
        void set_frequency(const int32_t max_frequency)
        {
            const int32_t clock_freq = ClockDriver::get_main_clock_frequency();

            assert(max_frequency >= (clock_freq / 65536) &&  max_frequency <= clock_freq);

            // Integer ceiling of clock_freq / max_frequency
            const int32_t divval = (clock_freq / max_frequency) + ((clock_freq % max_frequency) != 0);

            // Configure the SPI clock divider
            // NOTE: DIVVAL is -1 encoded such that the value 0 results in PCLK/1, the
            //       value 1 results in PCLK/2, up to the maximum possible divide value
            //       of 0xFFFF, which results in PCLK/65536.
            m_spi->DIV = (divval - 1) & 0xFFFF;
        }

        // -------- READ / WRITE ----------------------------------------------

        // Read data that has been received
        uint32_t read_data() const
        {
            return m_spi->RXDAT & 0x0000FFFF;
        }

        // Write data to be transmitted in master mode
        void master_write_data(const uint32_t value)
        {
            write_data(value);
        }

        // Write data to be transmitted in slave mode
        void slave_write_data(const uint32_t value)
        {
            write_data(value);
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        void enable() { m_spi->CFG |= cfg_enable; }

        // Disable peripheral
        void disable() { m_spi->CFG &= ~cfg_enable; }

        // Gets the enable state
        bool is_enabled() const { return (m_spi->CFG & cfg_enable) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return (get_status() & Status::tx_ready) != 0; }
        bool is_readable() const { return (get_status() & Status::rx_ready) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(m_spi->STAT);
        }

        void clear_status(const StatusBitmask bitmask)
        {
            m_spi->STAT = (bitmask & Status::clear_all_bitmask).bits();
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            m_spi->INTENSET = bitmask.bits();
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            m_spi->INTENCLR = bitmask.bits();
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(m_spi->INTSTAT);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::spi0: NVIC_EnableIRQ(SPI0_IRQn); break;
                case Name::spi1: NVIC_EnableIRQ(SPI1_IRQn); break;
                default:                                    break;
            }
        }

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::spi0: NVIC_DisableIRQ(SPI0_IRQn); break;
                case Name::spi1: NVIC_DisableIRQ(SPI1_IRQn); break;
                default:                                     break;
            }
        }

        bool is_irq_enabled()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::spi0: return (__NVIC_GetEnableIRQ(SPI0_IRQn) != 0); break;
                case Name::spi1: return (__NVIC_GetEnableIRQ(SPI1_IRQn) != 0); break;
                default:         return false;                                 break;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::spi0: NVIC_SetPriority(SPI0_IRQn, irq_priority); break;
                case Name::spi1: NVIC_SetPriority(SPI1_IRQn, irq_priority); break;
                default:                                                    break;
            }
        }

        void assign_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_irq_handler = irq_handler;
        }

        void remove_irq_handler()
        {
            m_irq_handler = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // SPI peripheral names selection
        enum class Name
        {
            spi0 = 0,
            spi1
        };

        // SPI Configuration Register (CFG) bits
        enum CFG : uint32_t
        {
            cfg_enable = (1 << 0),
            cfg_master = (1 << 2),
            cfg_lsbf   = (1 << 3),
            cfg_cpha   = (1 << 4),
            cfg_cpol   = (1 << 5),
            cfg_loop   = (1 << 7),
            cfg_spol0  = (1 << 8),
            cfg_spol1  = (1 << 9),
            cfg_spol2  = (1 << 10),
            cfg_spol3  = (1 << 11)
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- WRITE -----------------------------------------------------

        // Write data to be transmitted
        void write_data(const uint32_t value)
        {
            m_spi->TXDAT = value & 0x0000FFFF;
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            if(m_irq_handler != nullptr)
            {
                return m_irq_handler();
            }

            return 0;
        }

        // IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return SpiDriver::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        LPC_SPI_T* m_spi { nullptr };   // Pointer to the CMSIS SPI structure
        IrqHandler m_irq_handler;       // User defined IRQ handler
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SPI_HPP
