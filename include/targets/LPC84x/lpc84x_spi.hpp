// ----------------------------------------------------------------------------
// @file    lpc84x_spi.hpp
// @brief   NXP LPC84x SPI class.
// @date    3 May 2018
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

#include "system/array"
#include "system/delegate"
#include "targets/peripheral_ref_counter.hpp"
#include "targets/LPC84x/lpc84x_cmsis.h"
#include "targets/LPC84x/lpc84x_pins.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




// Forward declaration of IRQ handlers
extern "C" void SPI0_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void);




// Number of available SPI peripherals
static constexpr std::size_t SPI_COUNT { 2 };

class Spi : private PeripheralRefCounter<Spi, SPI_COUNT>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralSpi = PeripheralRefCounter<Spi, SPI_COUNT>;

        // SPI peripheral names selection
        enum class Name
        {
            SPI0 = 0,
            SPI1
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

        class IrqFlags
        {
            public:

                IrqFlags(const uint32_t intstat) : m_intstat { intstat }
                {}

                bool is_rx_ready     () const { return (m_intstat & STAT_RXRDY) != 0; }
                bool is_tx_ready     () const { return (m_intstat & STAT_TXRDY) != 0; }
                bool is_rx_overrun   () const { return (m_intstat & STAT_RXOV ) != 0; }
                bool is_tx_underrun  () const { return (m_intstat & STAT_TXUR ) != 0; }
                bool is_ssel_assert  () const { return (m_intstat & STAT_SSA  ) != 0; }
                bool is_ssel_deassert() const { return (m_intstat & STAT_SSD  ) != 0; }

            private:

                uint32_t m_intstat;
        };

        // IRQ handler definition
        using IrqHandlerType = int32_t(const IrqFlags& irq_flags);
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        void enable() { m_spi->CFG |= CFG_ENABLE; }

        // Disable peripheral
        void disable() { m_spi->CFG &= ~CFG_ENABLE; }

        // Gets the enable state
        bool is_enabled() const { return (m_spi->CFG & CFG_ENABLE) != 0; }

        // -------- GET STATUS FLAGS ------------------------------------------

        bool is_writable     () const { return (m_spi->STAT & STAT_TXRDY      ) != 0; }
        bool is_readable     () const { return (m_spi->STAT & STAT_RXRDY      ) != 0; }
        bool is_rx_overrun   () const { return (m_spi->STAT & STAT_RXOV       ) != 0; }
        bool is_tx_underrun  () const { return (m_spi->STAT & STAT_TXUR       ) != 0; }
        bool is_ssel_assert  () const { return (m_spi->STAT & STAT_SSA        ) != 0; }
        bool is_ssel_deassert() const { return (m_spi->STAT & STAT_SSD        ) != 0; }
        bool is_stalled      () const { return (m_spi->STAT & STAT_STALLED    ) != 0; }
        bool is_end_transfer () const { return (m_spi->STAT & STAT_ENDTRANSFER) != 0; }
        bool is_master_idle  () const { return (m_spi->STAT & STAT_MSTIDLE    ) != 0; }

        // -------- CLEAR STATUS FLAGS ----------------------------------------

        void clear_rx_overrun   () { m_spi->STAT |= STAT_RXOV;        }
        void clear_tx_underrun  () { m_spi->STAT |= STAT_TXUR;        }
        void clear_ssel_assert  () { m_spi->STAT |= STAT_SSA;         }
        void clear_ssel_deassert() { m_spi->STAT |= STAT_SSD;         }
        void clear_end_transfer () { m_spi->STAT |= STAT_ENDTRANSFER; }

        // -------- ENABLE INTERRUPTS -----------------------------------------

        void enable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: NVIC_EnableIRQ(SPI0_IRQn); break;
                case Name::SPI1: NVIC_EnableIRQ(SPI1_IRQn); break;
                default:                                    break;
            }
        }

        void enable_irq_rx_ready     () { m_spi->INTENSET |= INTEN_RXRDY; }
        void enable_irq_tx_ready     () { m_spi->INTENSET |= INTEN_TXRDY; }
        void enable_irq_rx_overrun   () { m_spi->INTENSET |= INTEN_RXOV;  }
        void enable_irq_tx_underrun  () { m_spi->INTENSET |= INTEN_TXUR;  }
        void enable_irq_ssel_assert  () { m_spi->INTENSET |= INTEN_SSA;   }
        void enable_irq_ssel_deassert() { m_spi->INTENSET |= INTEN_SSD;   }

        // -------- DISABLE INTERRUPTS ----------------------------------------

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: NVIC_DisableIRQ(SPI0_IRQn); break;
                case Name::SPI1: NVIC_DisableIRQ(SPI1_IRQn); break;
                default:                                     break;
            }
        }

        void disable_irq_rx_ready     () { m_spi->INTENCLR |= INTEN_RXRDY; }
        void disable_irq_tx_ready     () { m_spi->INTENCLR |= INTEN_TXRDY; }
        void disable_irq_rx_overrun   () { m_spi->INTENCLR |= INTEN_RXOV;  }
        void disable_irq_tx_underrun  () { m_spi->INTENCLR |= INTEN_TXUR;  }
        void disable_irq_ssel_assert  () { m_spi->INTENCLR |= INTEN_SSA;   }
        void disable_irq_ssel_deassert() { m_spi->INTENCLR |= INTEN_SSD;   }

        // -------- GET ENABLED INTERRUPTS ------------------------------------

        bool is_enabled_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: return (__NVIC_GetEnableIRQ(SPI0_IRQn) != 0); break;
                case Name::SPI1: return (__NVIC_GetEnableIRQ(SPI1_IRQn) != 0); break;
                default:         return false;                                 break;
            }
        }

        bool is_enabled_irq_rx_ready     () const { return (m_spi->INTENSET & INTEN_RXRDY) != 0; }
        bool is_enabled_irq_tx_ready     () const { return (m_spi->INTENSET & INTEN_TXRDY) != 0; }
        bool is_enabled_irq_rx_overrun   () const { return (m_spi->INTENSET & INTEN_RXOV ) != 0; }
        bool is_enabled_irq_tx_underrun  () const { return (m_spi->INTENSET & INTEN_TXUR ) != 0; }
        bool is_enabled_irq_ssel_assert  () const { return (m_spi->INTENSET & INTEN_SSA  ) != 0; }
        bool is_enabled_irq_ssel_deassert() const { return (m_spi->INTENSET & INTEN_SSD  ) != 0; }

        // -------- IRQ HANDLER ASSIGNMENT ------------------------------------

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::SPI0: NVIC_SetPriority(SPI0_IRQn, irq_priority); break;
                case Name::SPI1: NVIC_SetPriority(SPI0_IRQn, irq_priority); break;
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

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Master modes selection (defined to map the CFG register directly)
        enum class MasterMode
        {
            SLAVE  = (0 << 2),
            MASTER = (1 << 2)
        };

        // SPI Configuration Register (CFG) bits
        enum CFG : uint32_t
        {
            CFG_ENABLE = (1 << 0),
            CFG_MASTER = (1 << 2),
            CFG_LSBF   = (1 << 3),
            CFG_CPHA   = (1 << 4),
            CFG_CPOL   = (1 << 5),
            CFG_LOOP   = (1 << 7),
            CFG_SPOL0  = (1 << 8),
            CFG_SPOL1  = (1 << 9),
            CFG_SPOL2  = (1 << 10),
            CFG_SPOL3  = (1 << 11)
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
                case Name::SPI1: Clock::disable(Clock::Peripheral::SPI1); NVIC_DisableIRQ(SPI1_IRQn); break;
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

                    Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::SPI0,
                                                       Clock::PeripheralClockSource::MAIN_CLK);

                    Swm::assign(Swm::PinMovable::SPI0_MOSI_IO, mosi);
                    Swm::assign(Swm::PinMovable::SPI0_MISO_IO, miso);
                    Swm::assign(Swm::PinMovable::SPI0_SCK_IO, sck);

                    if(slave_sel != Pin::Name::NC)
                    {
                        Swm::assign(Swm::PinMovable::SPI0_SSEL0_IO, slave_sel);
                    }
                }   break;

                case Name::SPI1:
                {
                    // Set pointer to the available SPI structure
                    m_spi = LPC_SPI1;

                    Clock::enable(Clock::Peripheral::SPI1);
                    Power::reset(Power::ResetPeripheral::SPI1);

                    Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::SPI1,
                                                       Clock::PeripheralClockSource::MAIN_CLK);

                    Swm::assign(Swm::PinMovable::SPI1_MOSI_IO, mosi);
                    Swm::assign(Swm::PinMovable::SPI1_MISO_IO, miso);
                    Swm::assign(Swm::PinMovable::SPI1_SCK_IO, sck);

                    if(slave_sel != Pin::Name::NC)
                    {
                        Swm::assign(Swm::PinMovable::SPI1_SSEL0_IO, slave_sel);
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
            const int32_t clock_freq = Clock::get_main_clock_frequency();

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

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void SPI0_IRQHandler(void);
        friend void SPI1_IRQHandler(void);

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (manage interrupt flags and call user IRQ handlers)
        int32_t irq_handler()
        {
            const IrqFlags irq_flags { m_spi->INTSTAT };

            if(m_irq_handler != nullptr)
            {
                return m_irq_handler(irq_flags);
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

        LPC_SPI_T* m_spi { nullptr };   // Pointer to the low level SPI structure

        IrqHandler m_irq_handler;       // User defined IRQ handler
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SPI_HPP
