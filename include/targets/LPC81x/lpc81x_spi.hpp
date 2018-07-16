// ----------------------------------------------------------------------------
// @file    lpc81x_spi.hpp
// @brief   NXP LPC81x SPI class.
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

#ifndef __XARMLIB_TARGETS_LPC81X_SPI_HPP
#define __XARMLIB_TARGETS_LPC81X_SPI_HPP

#include "external/bitmask.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <array>




// Forward declaration of IRQ handler for all LPC81x packages
extern "C" void SPI0_IRQHandler(void);

#if (TARGET_SPI_COUNT == 2)
// Forward declaration of additional IRQ handlers
extern "C" void SPI1_IRQHandler(void);
#endif




namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




namespace private_spi
{

// SPI Status Register (STAT) bits
enum class Status
{
    RX_READY      = (1 << 0), // Receiver Ready flag
    TX_READY      = (1 << 1), // Transmitter Ready flag
    RX_OVERRUN    = (1 << 2), // Receiver Overrun interrupt flag (applies only to slave mode)
    TX_UNDERRUN   = (1 << 3), // Transmitter Underrun interrupt flag (applies only to slave mode)
    SSEL_ASSERT   = (1 << 4), // Slave Select Assert flag
    SSEL_DEASSERT = (1 << 5), // Slave Select Deassert flag
    STALLED       = (1 << 6), // Stalled status flag
    END_TRANSFER  = (1 << 7), // End Transfer control bit
    MASTER_IDLE   = (1 << 8), // Master idle status flag
    ALL           = 0x1FF     // 1'1111'1111
};

// SPI Interrupt Register (INTENSET and INTENCLR) bits
enum class Interrupt
{
    RX_READY      = (1 << 0), // Receiver data is available
    TX_READY      = (1 << 1), // Transmitter holding register is available
    RX_OVERRUN    = (1 << 2), // Receiver overrun occurs (applies only to slave mode)
    TX_UNDERRUN   = (1 << 3), // Transmitter underrun occurs (applies only to slave mode)
    SSEL_ASSERT   = (1 << 4), // Slave Select is asserted
    SSEL_DEASSERT = (1 << 5), // Slave Select is deasserted
    ALL           = 0x3F      // 11'1111
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::ALL))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::ALL))

} // namespace private_spi




// Number of available SPI peripherals
static constexpr std::size_t SPI_COUNT { TARGET_SPI_COUNT };

class Spi : private PeripheralRefCounter<Spi, SPI_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void ::SPI0_IRQHandler(void);
#if (TARGET_SPI_COUNT == 2)
        friend void ::SPI1_IRQHandler(void);
#endif

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralSpi = PeripheralRefCounter<Spi, SPI_COUNT>;

        // SPI peripheral names selection
        enum class Name
        {
            SPI0 = 0,
#if (TARGET_SPI_COUNT == 2)
            SPI1
#endif
        };

        // Master modes selection (defined to map the CFG register directly)
        enum class MasterMode
        {
            SLAVE  = (0 << 2),
            MASTER = (1 << 2)
        };

        // Numbered operating mode selection (defined to map the CFG register directly)
        enum class SpiMode
        {
            //        CPOL   |   CPHA
            MODE0 = (0 << 5) | (0 << 4),    // CPOL = 0 | CPHA = 0
            MODE1 = (0 << 5) | (1 << 4),    // CPOL = 0 | CPHA = 1
            MODE2 = (1 << 5) | (0 << 4),    // CPOL = 1 | CPHA = 0
            MODE3 = (1 << 5) | (1 << 4)     // CPOL = 1 | CPHA = 1
        };

        // Data length selection (defined to map the TXCTL register directly)
        enum class DataBits
        {
            BIT_1   = (0x00 << 24),
            BITS_2  = (0x01 << 24),
            BITS_3  = (0x02 << 24),
            BITS_4  = (0x03 << 24),
            BITS_5  = (0x04 << 24),
            BITS_6  = (0x05 << 24),
            BITS_7  = (0x06 << 24),
            BITS_8  = (0x07 << 24),
            BITS_9  = (0x08 << 24),
            BITS_10 = (0x09 << 24),
            BITS_11 = (0x0A << 24),
            BITS_12 = (0x0B << 24),
            BITS_13 = (0x0C << 24),
            BITS_14 = (0x0D << 24),
            BITS_15 = (0x0E << 24),
            BITS_16 = (0x0F << 24)
        };

        // Data order selection (defined to map the CFG register directly)
        enum class DataOrder
        {
            MSB_FIRST = (0 << 3),
            LSB_FIRST = (1 << 3)
        };

        // Slave select polarity selection (defined to map the CFG register directly)
        enum class SselPolarity
        {
            LOW  = (0 << 8),
            HIGH = (1 << 8)
        };

        // Loopback mode selection (defined to map the CFG register directly)
        enum class LoopbackMode
        {
            DISABLED = (0 << 7),
            ENABLED  = (1 << 7)
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

        Spi() : PeripheralSpi(*this)
        {}

        ~Spi()
        {
            // Disable peripheral
            disable();

            const Name name = static_cast<Name>(get_index());

            // Disable peripheral clock sources and interrupts
            switch(name)
            {
                case Name::SPI0: Clock::disable(Clock::Peripheral::SPI0); NVIC_DisableIRQ(SPI0_IRQn); break;
#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1: Clock::disable(Clock::Peripheral::SPI1); NVIC_DisableIRQ(SPI1_IRQn); break;
#endif
                default:                                                                              break;
            }
        }

        // -------- INITIALIZATION / CONFIGURATION ----------------------------

        // Initialize SPI peripheral structure and pins (constructor helper function)
        void initialize(const Pin::Name mosi,
                        const Pin::Name miso,
                        const Pin::Name sck,
                        const Pin::Name slave_sel)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0:
                {
                    // Set pointer to the available SPI structure
                    m_spi = LPC_SPI0;

                    Clock::enable(Clock::Peripheral::SPI0);
                    Power::reset(Power::ResetPeripheral::SPI0);

                    Swm::assign(Swm::PinMovable::SPI0_MOSI_IO, mosi);
                    Swm::assign(Swm::PinMovable::SPI0_MISO_IO, miso);
                    Swm::assign(Swm::PinMovable::SPI0_SCK_IO, sck);

                    if(slave_sel != Pin::Name::NC)
                    {
                        Swm::assign(Swm::PinMovable::SPI0_SSEL_IO, slave_sel);
                    }
                }   break;

#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1:
                {
                    // Set pointer to the available SPI structure
                    m_spi = LPC_SPI1;

                    Clock::enable(Clock::Peripheral::SPI1);
                    Power::reset(Power::ResetPeripheral::SPI1);

                    Swm::assign(Swm::PinMovable::SPI1_MOSI_IO, mosi);
                    Swm::assign(Swm::PinMovable::SPI1_MISO_IO, miso);
                    Swm::assign(Swm::PinMovable::SPI1_SCK_IO, sck);

                    if(slave_sel != Pin::Name::NC)
                    {
                        Swm::assign(Swm::PinMovable::SPI1_SSEL_IO, slave_sel);
                    }
                }   break;
#endif

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
            const int32_t clock_freq = Clock::get_system_clock_frequency();

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

        // Write data to be transmitted
        void write_data(const uint32_t value)
        {
            m_spi->TXDAT = value & 0x0000FFFF;
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        void enable() { m_spi->CFG |= CFG_ENABLE; }

        // Disable peripheral
        void disable() { m_spi->CFG &= ~CFG_ENABLE; }

        // Gets the enable state
        bool is_enabled() const { return (m_spi->CFG & CFG_ENABLE) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return (get_status() & Status::TX_READY) != 0; }
        bool is_readable() const { return (get_status() & Status::RX_READY) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(m_spi->STAT);
        }

        void clear_status(const StatusBitmask bitmask)
        {
            // Only the following bits can be cleared
            const StatusBitmask clear_all_mask = Status::RX_OVERRUN
                                               | Status::TX_UNDERRUN
                                               | Status::SSEL_ASSERT
                                               | Status::SSEL_DEASSERT
                                               | Status::END_TRANSFER;

            m_spi->STAT = bitmask.bits() & clear_all_mask.bits();
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
                case Name::SPI0: NVIC_EnableIRQ(SPI0_IRQn); break;
#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1: NVIC_EnableIRQ(SPI1_IRQn); break;
#endif
                default:                                    break;
            }
        }

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: NVIC_DisableIRQ(SPI0_IRQn); break;
#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1: NVIC_DisableIRQ(SPI1_IRQn); break;
#endif
                default:                                     break;
            }
        }

        bool is_irq_enabled()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: return (__NVIC_GetEnableIRQ(SPI0_IRQn) != 0); break;
#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1: return (__NVIC_GetEnableIRQ(SPI1_IRQn) != 0); break;
#endif
                default:         return false;                                 break;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: NVIC_SetPriority(SPI0_IRQn, irq_priority); break;
#if (TARGET_SPI_COUNT == 2)
                case Name::SPI1: NVIC_SetPriority(SPI1_IRQn, irq_priority); break;
#endif
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

        // SPI Configuration Register (CFG) bits
        enum CFG : uint32_t
        {
            CFG_ENABLE = (1 << 0),
            CFG_MASTER = (1 << 2),
            CFG_LSBF   = (1 << 3),
            CFG_CPHA   = (1 << 4),
            CFG_CPOL   = (1 << 5),
            CFG_LOOP   = (1 << 7),
            CFG_SPOL   = (1 << 8)
        };

        // SPI Status Register (STAT) bits
        enum STAT : uint32_t
        {
            STAT_RXRDY       = (1 << 0),    // Receiver Ready flag
            STAT_TXRDY       = (1 << 1),    // Transmitter Ready flag
            STAT_RXOV        = (1 << 2),    // Receiver Overrun interrupt flag (applies only to slave mode)
            STAT_TXUR        = (1 << 3),    // Transmitter Underrun interrupt flag (applies only to slave mode)
            STAT_SSA         = (1 << 4),    // Slave Select Assert flag
            STAT_SSD         = (1 << 5),    // Slave Select Deassert flag
            STAT_STALLED     = (1 << 6),    // Stalled status flag
            STAT_ENDTRANSFER = (1 << 7),    // End Transfer control bit
            STAT_MSTIDLE     = (1 << 8)     // Master idle status flag
        };

        // SPI Interrupt Enable get, set and clear bits (defined to map INTENSET and INTENCLR registers directly)
        enum INTEN : uint32_t
        {
            INTEN_RXRDY = (1 << 0),         // Receiver data is available
            INTEN_TXRDY = (1 << 1),         // Transmitter holding register is available
            INTEN_RXOV  = (1 << 2),         // Receiver overrun occurs (applies only to slave mode)
            INTEN_TXUR  = (1 << 3),         // Transmitter underrun occurs (applies only to slave mode)
            INTEN_SSA   = (1 << 4),         // Slave Select is asserted
            INTEN_SSD   = (1 << 5)          // Slave Select is deasserted
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (manage interrupt flags and call user IRQ handlers)
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

            return Spi::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        LPC_SPI_T* m_spi { nullptr };   // Pointer to the CMSIS SPI structure
        IrqHandler m_irq_handler;       // User defined IRQ handler
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_SPI_HPP
