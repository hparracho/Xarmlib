// ----------------------------------------------------------------------------
// @file    kv5x_spi.hpp
// @brief   Kinetis KV5x SPI class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          Both sizes are 4.
// @date    16 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_KV5X_SPI_HPP
#define __XARMLIB_TARGETS_KV5X_SPI_HPP

#include "xarmlib_config.hpp"
#include "external/bitmask.hpp"
#include "fsl_dspi.h"
#include "targets/KV5x/kv5x_pin.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handlers for all KV5x packages
extern "C" void SPI0_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void);
extern "C" void SPI2_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv5x
{




namespace private_spi
{

// SPI status flags
enum class Status : uint32_t
{
    tx_complete           = SPI_SR_TCF_MASK,    // Transfer Complete Flag
    tx_and_rx_status      = SPI_SR_TXRXS_MASK,  // The module is in Stopped/Running state
    end_of_queue          = SPI_SR_EOQF_MASK,   // End of Queue Flag
    tx_fifo_underflow     = SPI_SR_TFUF_MASK,   // Transmit FIFO Underflow Flag
    tx_fifo_fill_request  = SPI_SR_TFFF_MASK,   // Transmit FIFO Fill Flag
    rx_fifo_overflow      = SPI_SR_RFOF_MASK,   // Receive FIFO Overflow Flag
    rx_fifo_drain_request = SPI_SR_RFDF_MASK,   // Receive FIFO Drain Flag
    clear_all_bitmask     = tx_complete
                          | end_of_queue
                          | tx_fifo_underflow
                          | tx_fifo_fill_request
                          | rx_fifo_overflow
                          | rx_fifo_drain_request,
    bitmask               = tx_complete
                          | tx_and_rx_status
                          | end_of_queue
                          | tx_fifo_underflow
                          | tx_fifo_fill_request
                          | rx_fifo_overflow
                          | rx_fifo_drain_request
};

// SPI interrupt sources
enum class Interrupt : uint32_t
{
    tx_complete           = SPI_RSER_TCF_RE_MASK,   // TCF  interrupt enable
    end_of_queue          = SPI_RSER_EOQF_RE_MASK,  // EOQF interrupt enable
    tx_fifo_underflow     = SPI_RSER_TFUF_RE_MASK,  // TFUF interrupt enable
    tx_fifo_fill_request  = SPI_RSER_TFFF_RE_MASK,  // TFFF interrupt enable, DMA disable
    rx_fifo_overflow      = SPI_RSER_RFOF_RE_MASK,  // RFOF interrupt enable
    rx_fifo_drain_request = SPI_RSER_RFDF_RE_MASK,  // RFDF interrupt enable, DMA disable
    bitmask               = tx_complete
                          | end_of_queue
                          | tx_fifo_underflow
                          | tx_fifo_fill_request
                          | rx_fifo_overflow
                          | rx_fifo_drain_request
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_spi




static constexpr uint32_t TARGET_SPI_MASK  = (1UL << TARGET_SPI_COUNT) - 1;

class SpiDriver : private PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT, TARGET_SPI_MASK>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTION DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::SPI0_IRQHandler(void);
        friend void ::SPI1_IRQHandler(void);
        friend void ::SPI2_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralSpi = PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT, TARGET_SPI_MASK>;

        // Clock and Transfer Attributes Register (CTAR) selection
        enum class CtarSelection
        {
            ctar0 = 0,  // selection option for master or slave mode
                        // NOTE: CTAR0 and CTAR0_SLAVE are the same register address
            ctar1       // selection option for master mode only
        };

        // Data length selection for a given CTAR
        enum class DataBits
        {
            bits_4 = 4,
            bits_5,
            bits_6,
            bits_7,
            bits_8,
            bits_9,
            bits_10,
            bits_11,
            bits_12,
            bits_13,
            bits_14,
            bits_15,
            bits_16
        };

        // Numbered operating mode selection for a given CTAR
        enum class SpiMode
        {
            //        CPOL   |   CPHA
            mode0 = (0 << 1) | (0 << 0),    // CPOL = 0 | CPHA = 0
            mode1 = (0 << 1) | (1 << 0),    // CPOL = 0 | CPHA = 1
            mode2 = (1 << 1) | (0 << 0),    // CPOL = 1 | CPHA = 0
            mode3 = (1 << 1) | (1 << 0)     // CPOL = 1 | CPHA = 1
        };

        // Data order selection for a given CTAR
        enum class DataOrder
        {
            msb_first = 0,
            lsb_first
        };

        // Run continuously Serial Communication Clock (SCK) mode selection
        // NOTE: this is only supported for CPHA = 1.
        enum class ContinuousSck
        {
            disabled = 0,
            enabled
        };

        // Modified transfer format mode selection
        enum class ModifiedTransferFormat
        {
            disabled = 0,
            enabled
        };

        // Sample point mode selection
        // Controls when the SPI master samples SIN in the Modified Transfer Format.
        // NOTE: this is valid only when CPHA = 0.
        enum class SamplePoint
        {
            sck_to_sin_0_clock = 0, // 0 system clocks between SCK edge and SIN sample
            sck_to_sin_1_clock,     // 1 system clock  between SCK edge and SIN sample
            sck_to_sin_2_clock      // 2 system clocks between SCK edge and SIN sample
        };

        // Receive FIFO overflow overwrite mode selection
        enum class RxFifoOverwrite
        {
            disabled = 0,   // Incoming data (the data from the transfer, generating the overflow) is ignored
            enabled         // Incoming data is shifted into the shift register
        };

        struct MasterConfig
        {
            // Master CTAR configurations
            // NOTE: in master mode both CTARs are configured with the same configurations
            int32_t   frequency  = 500000;
            SpiMode   spi_mode   = SpiMode::mode3;
            DataBits  data_bits  = DataBits::bits_8;
            DataOrder data_order = DataOrder::msb_first;

            ContinuousSck          continuous_sck           = ContinuousSck::disabled;
            ModifiedTransferFormat modified_transfer_format = ModifiedTransferFormat::disabled;
            SamplePoint            sample_point             = SamplePoint::sck_to_sin_0_clock;
            RxFifoOverwrite        rx_fifo_overwrite        = RxFifoOverwrite::disabled;
        };

        struct SlaveConfig
        {
            // Slave CTAR configurations
            // NOTES: - in slave mode only CTAR0 (same register as CTAR0_SLAVE) is used
            //        - DataOrder LSB first is not supported in slave mode
            SpiMode  spi_mode  = SpiMode::mode3;
            DataBits data_bits = DataBits::bits_8;

            ContinuousSck          continuous_sck           = ContinuousSck::disabled;
            ModifiedTransferFormat modified_transfer_format = ModifiedTransferFormat::disabled;
            SamplePoint            sample_point             = SamplePoint::sck_to_sin_0_clock;
            RxFifoOverwrite        rx_fifo_overwrite        = RxFifoOverwrite::disabled;
        };

        // Type safe accessor to status flags
        using Status        = private_spi::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to interrupt sources
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
                  const MasterConfig&   master_config) : PeripheralSpi(*this, get_peripheral_index(master_mosi, master_miso, master_sck))
       {
            const auto pin_config = get_pin_config(master_mosi, master_miso, master_sck);

            PinDriver::set_pin_mux(master_mosi, pin_config.pin_mux);
            PinDriver::set_pin_mux(master_miso, pin_config.pin_mux);
            PinDriver::set_pin_mux(master_sck,  pin_config.pin_mux);

            m_spi_name = pin_config.spi_name;

            switch(m_spi_name)
            {
                case Name::spi0: m_spi_base = SPI0; break;
                case Name::spi1: m_spi_base = SPI1; break;
                case Name::spi2: m_spi_base = SPI2; break;
            };

            initialize(master_config);

            m_frequency = master_config.frequency;

            disable_irq();

            flush_rx_fifo();
            flush_tx_fifo();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);
        }

        // Slave mode constructor
        SpiDriver(const PinDriver::Name slave_mosi,
                  const PinDriver::Name slave_miso,
                  const PinDriver::Name slave_sck,
                  const PinDriver::Name slave_sel,
                  const SlaveConfig&    slave_config) : PeripheralSpi(*this, get_peripheral_index(slave_mosi, slave_miso, slave_sck, slave_sel))
        {
            const auto pin_config = get_pin_config(slave_mosi, slave_miso, slave_sck, slave_sel);

            PinDriver::set_pin_mux(slave_mosi, pin_config.pin_mux);
            PinDriver::set_pin_mux(slave_miso, pin_config.pin_mux);
            PinDriver::set_pin_mux(slave_sck,  pin_config.pin_mux);
            PinDriver::set_pin_mux(slave_sel,  pin_config.pin_mux);

            m_spi_name = pin_config.spi_name;

            switch(m_spi_name)
            {
                case Name::spi0: m_spi_base = SPI0; break;
                case Name::spi1: m_spi_base = SPI1; break;
                case Name::spi2: m_spi_base = SPI2; break;
            };

            initialize(slave_config);

            disable_irq();

            flush_rx_fifo();
            flush_tx_fifo();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);
        }

        ~SpiDriver()
        {
            disable_irq();

            flush_rx_fifo();
            flush_tx_fifo();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);

            DSPI_Deinit(m_spi_base);
        }

        // -------- CONFIGURATION ---------------------------------------------

        // NOTE: only in master mode!
        int32_t get_frequency() const
        {
            assert(is_master() == true);

            return m_frequency;
        }

        // NOTES: - the module must be in the stopped state
        //        - only in master mode!
        //        - the frequency will be set in both CTARs
        void set_frequency(const int32_t frequency)
        {
            assert(is_enabled() == false);
            assert(is_master()  == true);
            assert(frequency     > 0);

            DSPI_MasterSetBaudRate(m_spi_base, kDSPI_Ctar0, static_cast<uint32_t>(frequency), SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

            // Copy CTAR1 after CTAR0 was changed
            m_spi_base->CTAR[kDSPI_Ctar1] = m_spi_base->CTAR[kDSPI_Ctar0];

            m_frequency = frequency;
        }

        DataBits get_ctar_data_bits(const CtarSelection ctar_selection) const
        {
            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            return static_cast<DataBits>(((m_spi_base->CTAR[ctar_index] & SPI_CTAR_FMSZ_MASK) >> SPI_CTAR_FMSZ_SHIFT) + 1);
        }

        // NOTE: to configure CTAR the module must be in the stopped state
        void set_ctar_data_bits(const CtarSelection ctar_selection, const DataBits data_bits)
        {
            assert(is_enabled() == false);

            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            const auto frame_size = static_cast<uint32_t>(data_bits) - 1;

            m_spi_base->CTAR[ctar_index] = (m_spi_base->CTAR[ctar_index] & ~SPI_CTAR_FMSZ_MASK) | SPI_CTAR_FMSZ(frame_size);
        }

        DataOrder get_data_order(const CtarSelection ctar_selection) const
        {
            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            return static_cast<DataOrder>((m_spi_base->CTAR[ctar_index] & SPI_CTAR_LSBFE_MASK) >> SPI_CTAR_LSBFE_SHIFT);
        }

        // NOTE: this is only supported for CPHA = 1
        void set_continuous_sck(const ContinuousSck continuous_sck)
        {
            m_spi_base->MCR = (m_spi_base->MCR & ~SPI_MCR_CONT_SCKE_MASK) | SPI_MCR_CONT_SCKE(static_cast<uint32_t>(continuous_sck));
        }

        // -------- READ / WRITE ----------------------------------------------

        // Read data that has been received
        uint32_t read_data()
        {
            const uint32_t data = DSPI_ReadData(m_spi_base);

            clear_status(Status::rx_fifo_drain_request);

            return data;
        }

        // Write data to be transmitted in master mode
        void master_write_data(const uint32_t      value,
                               const CtarSelection ctar_selection = CtarSelection::ctar0,
                               const bool          is_end_of_queue = false,              // Signals that the current transfer is the last in the queue
                               const bool          clear_queue_transfer_counter = false) // Clears the transfer counter before transmission starts
        {
            assert(is_master() == true);

            dspi_command_data_config_t spi_command_data_config =
            {
                false,      // Option to enable the continuous assertion of the chip select between transfers
                static_cast<dspi_ctar_selection_t>(ctar_selection),
                kDSPI_Pcs0, // The desired PCS signal to use for the data transfer
                is_end_of_queue,
                clear_queue_transfer_counter
            };

            DSPI_MasterWriteData(m_spi_base, &spi_command_data_config, static_cast<uint16_t>(value));

            clear_status(Status::tx_fifo_fill_request);
        }

        // Write data to be transmitted in slave mode
        void slave_write_data(const uint32_t value)
        {
            assert(is_master() == false);

            DSPI_SlaveWriteData(m_spi_base, value);

            clear_status(Status::tx_fifo_fill_request);
        }

        // Get the transfer counter that indicates the number of SPI transfers made
        // NOTE: this counter is intended to assist in queue management
        uint16_t get_queue_transfer_counter() const
        {
            return ((m_spi_base->TCR & SPI_TCR_SPI_TCNT_MASK) >> SPI_TCR_SPI_TCNT_SHIFT);
        }

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        void flush_rx_fifo()
        {
            DSPI_FlushFifo(m_spi_base, false, true);
        }

        // Flush transmit FIFO counter
        void flush_tx_fifo()
        {
            DSPI_FlushFifo(m_spi_base, true, false);
        }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const
        {
            return ((m_spi_base->SR & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT);
        }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const
        {
            return ((m_spi_base->SR & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT);
        }

        // -------- ENABLE / DISABLE RUNNING STATE ----------------------------

        // Start frame transfers
        void enable()
        {
            // Status::end_of_queue bit must be cleared
            // (further info: 50.5.1 Start and Stop of module transfers from KV5x Reference Manual)
            clear_status(Status::end_of_queue);

            DSPI_StartTransfer(m_spi_base);
        }

        // Stop frame transfers
        // NOTE: stopping occur on the next frame boundary if a transfer is in
        //       progress, or immediately if no transfers are in progress
        void disable()
        {
            DSPI_StopTransfer(m_spi_base);

            flush_tx_fifo();
            flush_rx_fifo();

            clear_status(Status::clear_all_bitmask);

            while(is_enabled() == true);
        }

        // Gets the TX and RX run status
        bool is_enabled() const { return (get_status() & Status::tx_and_rx_status) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return (get_status() & Status::tx_fifo_fill_request) != 0; }
        bool is_readable() const { return (get_status() & Status::rx_fifo_drain_request) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(DSPI_GetStatusFlags(m_spi_base) & static_cast<uint32_t>(Status::bitmask));
        }

        void clear_status(const StatusBitmask bitmask)
        {
            DSPI_ClearStatusFlags(m_spi_base, (bitmask & Status::clear_all_bitmask).bits());
        }

        // -------- INTERRUPTS ------------------------------------------------

        // NOTE: Do not enable/disable interrupts while the module is in the Running state

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            assert(is_enabled() == false);

            DSPI_EnableInterrupts(m_spi_base, bitmask.bits());
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            assert(is_enabled() == false);

            DSPI_DisableInterrupts(m_spi_base, bitmask.bits());
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(m_spi_base->RSER);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            switch(m_spi_name)
            {
                case Name::spi0: NVIC_EnableIRQ(SPI0_IRQn); break;
                case Name::spi1: NVIC_EnableIRQ(SPI1_IRQn); break;
                case Name::spi2: NVIC_EnableIRQ(SPI2_IRQn); break;
                default:                                    break;
            }
        }

        void disable_irq()
        {
            switch(m_spi_name)
            {
                case Name::spi0: NVIC_DisableIRQ(SPI0_IRQn); break;
                case Name::spi1: NVIC_DisableIRQ(SPI1_IRQn); break;
                case Name::spi2: NVIC_DisableIRQ(SPI2_IRQn); break;
                default:                                     break;
            }
        }

        bool is_irq_enabled()
        {
            switch(m_spi_name)
            {
                case Name::spi0: return (NVIC_GetEnableIRQ(SPI0_IRQn) != 0); break;
                case Name::spi1: return (NVIC_GetEnableIRQ(SPI1_IRQn) != 0); break;
                case Name::spi2: return (NVIC_GetEnableIRQ(SPI2_IRQn) != 0); break;
                default:         return false;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            switch(m_spi_name)
            {
                case Name::spi0: NVIC_SetPriority(SPI0_IRQn, irq_priority); break;
                case Name::spi1: NVIC_SetPriority(SPI1_IRQn, irq_priority); break;
                case Name::spi2: NVIC_SetPriority(SPI2_IRQn, irq_priority); break;
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

        enum class Name
        {
            spi0 = 0,
            spi1,
            spi2
        };

        // Master modes selection
        /*enum class MasterMode
        {
            slave = 0,
            master
        };*/

        struct PinSignals
        {
            PinDriver::Name mosi;
            PinDriver::Name miso;
            PinDriver::Name sck;
            PinDriver::Name sel;
        };

        struct PinConfig
        {
            Name              spi_name;
            PinDriver::PinMux pin_mux;
        };

        // Pin map type
        using PinMap = std::pair<PinSignals, PinConfig>;

        // Pin map array type
        template <std::size_t Size>
        using PinMapArray = std::array<PinMap, Size>;

        static constexpr std::size_t m_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 144) ? 9 : 8 };

        static constexpr PinMapArray<m_pin_map_array_size> m_pin_map_array
        { { //                     MOSI                    MISO                    SCK                     SEL0
              { { PinDriver::Name::pe_1,  PinDriver::Name::pe_3,  PinDriver::Name::pe_2,  PinDriver::Name::pe_4  }, { Name::spi1, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pe_18, PinDriver::Name::pe_19, PinDriver::Name::pe_17, PinDriver::Name::pe_16 }, { Name::spi0, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pa_16, PinDriver::Name::pa_17, PinDriver::Name::pa_15, PinDriver::Name::pa_14 }, { Name::spi0, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pb_16, PinDriver::Name::pb_17, PinDriver::Name::pb_11, PinDriver::Name::pb_10 }, { Name::spi1, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pb_22, PinDriver::Name::pb_23, PinDriver::Name::pb_21, PinDriver::Name::pb_20 }, { Name::spi2, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pc_6,  PinDriver::Name::pc_7,  PinDriver::Name::pc_5,  PinDriver::Name::pc_4  }, { Name::spi0, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pd_2,  PinDriver::Name::pd_3,  PinDriver::Name::pd_1,  PinDriver::Name::pd_0  }, { Name::spi0, PinDriver::PinMux::alt2 } },
              { { PinDriver::Name::pd_6,  PinDriver::Name::pd_7,  PinDriver::Name::pd_5,  PinDriver::Name::pd_4  }, { Name::spi1, PinDriver::PinMux::alt7 } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { { PinDriver::Name::pd_13, PinDriver::Name::pd_14, PinDriver::Name::pd_12, PinDriver::Name::pd_11 }, { Name::spi2, PinDriver::PinMux::alt2 } }
#endif
        } };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONFIGURATION / INITIALIZATION ----------------------------

        // Return the peripheral index according to the master pins (master constructor helper function):
        static constexpr int32_t get_peripheral_index(const PinDriver::Name mosi, const PinDriver::Name miso, const PinDriver::Name sck)
        {
            const auto pin_config = get_pin_config(mosi, miso, sck);

            return static_cast<int32_t>(pin_config.spi_name);
        }

        // Get the pin config struct if the specified mosi, miso and sck are SPI pins
        static constexpr PinConfig get_pin_config(const PinDriver::Name mosi, const PinDriver::Name miso, const PinDriver::Name sck)
        {
            std::size_t index = 0;

            for(; index < m_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_pin_map_array[index];

                if(std::get<0>(pin_map).mosi == mosi && std::get<0>(pin_map).miso == miso && std::get<0>(pin_map).sck == sck)
                {
                    return std::get<1>(pin_map);
                }
            }

            // Assert mosi, miso and sck are SPI pins
            assert(index < m_pin_map_array.size());

            return { Name::spi0, PinDriver::PinMux::pin_disabled_or_analog };
        }

        // Return the peripheral index according to the slave pins (slave constructor helper function):
        static constexpr int32_t get_peripheral_index(const PinDriver::Name mosi, const PinDriver::Name miso, const PinDriver::Name sck, const PinDriver::Name sel)
        {
            const auto pin_config = get_pin_config(mosi, miso, sck, sel);

            return static_cast<int32_t>(pin_config.spi_name);
        }

        // Get the pin config struct if the specified mosi, miso, sck and sel are SPI pins
        static constexpr PinConfig get_pin_config(const PinDriver::Name mosi, const PinDriver::Name miso, const PinDriver::Name sck, const PinDriver::Name sel)
        {
            std::size_t index = 0;

            for(; index < m_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_pin_map_array[index];

                if(std::get<0>(pin_map).mosi == mosi && std::get<0>(pin_map).miso == miso && std::get<0>(pin_map).sck == sck && std::get<0>(pin_map).sel == sel)
                {
                    return std::get<1>(pin_map);
                }
            }

            // Assert mosi, miso, sck and sel are SPI pins
            assert(index < m_pin_map_array.size());

            return { Name::spi0, PinDriver::PinMux::pin_disabled_or_analog };
        }

        void initialize(const MasterConfig& master_config)
        {
            assert(master_config.frequency > 0);

            const uint32_t frequency = static_cast<uint32_t>(master_config.frequency);

            // Delay between the negation of the PCS signal at the end of a frame and
            // the assertion of PCS at the beginning of the next frame
            // NOTE: 50 ns is the minimum delay needed by the peripheral @ Clock::xtal_94mhz
            const uint32_t delay_between_transfer_ns = (500000000 / frequency) - 50;

            /*
             * NOTA (Emanuel Pinto @ 26 February 2019):
             * Em vez de se introduzir um delay entre transferências pode-se
             * ligar o Continuous SCK e usar o PCS também em modo contínuo até
             * à última transferência.
             * Código exemplo para ambos CTAR com DataBits::bits_10
             * (relembro que CPHA tem de ser 1!):

               flush_tx_fifo();
               flush_rx_fifo();

               write(0x1F, SpiMaster::CtarSelection::ctar0, true);
               write(0x1F, SpiMaster::CtarSelection::ctar1, false);

               set_continuous_sck(SpiMaster::ContinuousSck::enabled);
               enable();

               while(get_tx_fifo_counter() > 0);

               set_continuous_sck(SpiMaster::ContinuousSck::disabled);
               disable();
             */

            const dspi_master_ctar_config_t spi_master_ctar0_config =
            {
                frequency,
                static_cast<uint32_t>(master_config.data_bits),
                static_cast<dspi_clock_polarity_t>(static_cast<uint8_t>(master_config.spi_mode) >> 1),
                static_cast<dspi_clock_phase_t>(static_cast<uint8_t>(master_config.spi_mode) & 1),
                static_cast<dspi_shift_direction_t>(master_config.data_order),
                0,  // Delay between assertion of PCS and the first edge of the SCK in nanoseconds
                    // NOTE: 0 sets the minimum delay. It also sets the boundary value if out of range
                0,  // Delay between the last edge of SCK and the negation of PCS in nanoseconds
                    // NOTE: 0 sets the minimum delay. It also sets the boundary value if out of range
                delay_between_transfer_ns
            };

            const dspi_master_config_t spi_master_config =
            {
                kDSPI_Ctar0,
                spi_master_ctar0_config,
                kDSPI_Pcs0,         // The desired Peripheral Chip Select (PCS)
                kDSPI_PcsActiveLow, // The desired PCS active high or low
                static_cast<bool>(master_config.continuous_sck),
                static_cast<bool>(master_config.rx_fifo_overwrite),
                static_cast<bool>(master_config.modified_transfer_format),
                static_cast<dspi_master_sample_point_t>(master_config.sample_point)
            };

            DSPI_MasterInit(m_spi_base, &spi_master_config, SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

            // Stop frame transfers
            // ( DSPI_MasterInit(...) FSL's function decided to start frame
            // transfers by own initiative -_- )
            disable();

            // Configure CTAR1 after CTAR0 was configured in initialize
            m_spi_base->CTAR[kDSPI_Ctar1] = m_spi_base->CTAR[kDSPI_Ctar0];
        }

        void initialize(const SlaveConfig&  slave_config)
        {
            const dspi_slave_ctar_config_t spi_slave_ctar_config =
            {
                static_cast<uint32_t>(slave_config.data_bits),
                static_cast<dspi_clock_polarity_t>(static_cast<uint8_t>(slave_config.spi_mode) >> 1),
                static_cast<dspi_clock_phase_t>(static_cast<uint8_t>(slave_config.spi_mode) & 1)
            };

            const dspi_slave_config_t spi_slave_config =
            {
                kDSPI_Ctar0,
                spi_slave_ctar_config,
                static_cast<bool>(slave_config.continuous_sck),
                static_cast<bool>(slave_config.rx_fifo_overwrite),
                static_cast<bool>(slave_config.modified_transfer_format),
                static_cast<dspi_master_sample_point_t>(slave_config.sample_point)
            };

            DSPI_SlaveInit(m_spi_base, &spi_slave_config);

            // Stop frame transfers
            // ( DSPI_SlaveInit(...) FSL's function decided to start frame
            // transfers by own initiative -_- )
            disable();
        }

        bool is_master() const
        {
            return DSPI_IsMaster(m_spi_base);
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            return yield;
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

        Name       m_spi_name;
        SPI_Type*  m_spi_base { nullptr };  // Pointer to the CMSIS SPI structure
        int32_t    m_frequency { 0 };       // User defined frequency
        IrqHandler m_irq_handler;           // User defined IRQ handler
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV5X_SPI_HPP
