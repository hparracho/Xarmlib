// ----------------------------------------------------------------------------
// @file    kv4x_spi.hpp
// @brief   Kinetis KV4x SPI class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          Both sizes are 4.
// @date    20 September 2020
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

#ifndef __XARMLIB_TARGETS_KV4X_SPI_HPP
#define __XARMLIB_TARGETS_KV4X_SPI_HPP

#include "xarmlib_config.hpp"
#include "external/bitmask.hpp"
#include "fsl_dspi.h"
#include "targets/KV4x/kv4x_pin.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handler for all KV4x packages
extern "C" void SPI_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
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




class SpiDriver : private PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTION DECLARATION
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void ::SPI_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralSpi = PeripheralRefCounter<SpiDriver, TARGET_SPI_COUNT>;

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
                  const MasterConfig&   master_config) : PeripheralSpi(*this)
       {
            const auto mosi_pin_mux = get_mosi_pin_mux(master_mosi);
            const auto miso_pin_mux = get_miso_pin_mux(master_miso);
            const auto sck_pin_mux  = get_sck_pin_mux(master_sck);

            PinDriver::set_pin_mux(master_mosi, mosi_pin_mux);
            PinDriver::set_pin_mux(master_miso, miso_pin_mux);
            PinDriver::set_pin_mux(master_sck,  sck_pin_mux);

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
                  const SlaveConfig&    slave_config) : PeripheralSpi(*this)
        {
            assert(slave_sel != PinDriver::Name::nc);

            const auto mosi_pin_mux = get_mosi_pin_mux(slave_mosi);
            const auto miso_pin_mux = get_miso_pin_mux(slave_miso);
            const auto sck_pin_mux  = get_sck_pin_mux(slave_sck);
            const auto sel_pin_mux  = get_slave_sel_pin_mux(slave_sel);

            PinDriver::set_pin_mux(slave_mosi, mosi_pin_mux);
            PinDriver::set_pin_mux(slave_miso, miso_pin_mux);
            PinDriver::set_pin_mux(slave_sck,  sck_pin_mux);
            PinDriver::set_pin_mux(slave_sel,  sel_pin_mux);

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

            DSPI_Deinit(SPI);
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

            DSPI_MasterSetBaudRate(SPI, kDSPI_Ctar0, static_cast<uint32_t>(frequency), SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

            // Copy CTAR1 after CTAR0 was changed
            SPI->CTAR[kDSPI_Ctar1] = SPI->CTAR[kDSPI_Ctar0];

            m_frequency = frequency;
        }

        DataBits get_ctar_data_bits(const CtarSelection ctar_selection) const
        {
            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            return static_cast<DataBits>(((SPI->CTAR[ctar_index] & SPI_CTAR_FMSZ_MASK) >> SPI_CTAR_FMSZ_SHIFT) + 1);
        }

        // NOTE: to configure CTAR the module must be in the stopped state
        void set_ctar_data_bits(const CtarSelection ctar_selection, const DataBits data_bits)
        {
            assert(is_enabled() == false);

            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            const auto frame_size = static_cast<uint32_t>(data_bits) - 1;

            SPI->CTAR[ctar_index] = (SPI->CTAR[ctar_index] & ~SPI_CTAR_FMSZ_MASK) | SPI_CTAR_FMSZ(frame_size);
        }

        DataOrder get_data_order(const CtarSelection ctar_selection) const
        {
            const auto ctar_index = static_cast<std::size_t>(ctar_selection);

            return static_cast<DataOrder>((SPI->CTAR[ctar_index] & SPI_CTAR_LSBFE_MASK) >> SPI_CTAR_LSBFE_SHIFT);
        }

        // NOTE: this is only supported for CPHA = 1
        void set_continuous_sck(const ContinuousSck continuous_sck)
        {
            SPI->MCR = (SPI->MCR & ~SPI_MCR_CONT_SCKE_MASK) | SPI_MCR_CONT_SCKE(static_cast<uint32_t>(continuous_sck));
        }

        // -------- READ / WRITE ----------------------------------------------

        // Read data that has been received
        uint32_t read_data()
        {
            const uint32_t data = DSPI_ReadData(SPI);

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

            DSPI_MasterWriteData(SPI, &spi_command_data_config, static_cast<uint16_t>(value));

            clear_status(Status::tx_fifo_fill_request);
        }

        // Write data to be transmitted in slave mode
        void slave_write_data(const uint32_t value)
        {
            assert(is_master() == false);

            DSPI_SlaveWriteData(SPI, value);

            clear_status(Status::tx_fifo_fill_request);
        }

        // Get the transfer counter that indicates the number of SPI transfers made
        // NOTE: this counter is intended to assist in queue management
        uint16_t get_queue_transfer_counter() const
        {
            return ((SPI->TCR & SPI_TCR_SPI_TCNT_MASK) >> SPI_TCR_SPI_TCNT_SHIFT);
        }

        // -------- FIFOS -----------------------------------------------------

        // Flush receive FIFO counter
        void flush_rx_fifo()
        {
            DSPI_FlushFifo(SPI, false, true);
        }

        // Flush transmit FIFO counter
        void flush_tx_fifo()
        {
            DSPI_FlushFifo(SPI, true, false);
        }

        // Get the number of valid entries in the RX FIFO
        std::size_t get_rx_fifo_counter() const
        {
            return ((SPI->SR & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT);
        }

        // Get the number of valid entries in the TX FIFO
        std::size_t get_tx_fifo_counter() const
        {
            return ((SPI->SR & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT);
        }

        // -------- ENABLE / DISABLE RUNNING STATE ----------------------------

        // Start frame transfers
        // NOTE: Status::end_of_queue bit must be cleared
        void enable()
        {
            DSPI_StartTransfer(SPI);

            assert(is_enabled() == true);
        }

        // Stop frame transfers
        // NOTE: stopping occur on the next frame boundary if a transfer is in
        //       progress, or immediately if no transfers are in progress
        void disable()
        {
            DSPI_StopTransfer(SPI);
        }

        // Gets the TX and RX run status
        bool is_enabled() const { return (get_status() & Status::tx_and_rx_status) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_writable() const { return (get_status() & Status::tx_fifo_fill_request) != 0; }
        bool is_readable() const { return (get_status() & Status::rx_fifo_drain_request) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(DSPI_GetStatusFlags(SPI) & static_cast<uint32_t>(Status::bitmask));
        }

        void clear_status(const StatusBitmask bitmask)
        {
            DSPI_ClearStatusFlags(SPI, (bitmask & Status::clear_all_bitmask).bits());
        }

        // -------- INTERRUPTS ------------------------------------------------

        // NOTE: Do not enable/disable interrupts while the module is in the Running state

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            assert(is_enabled() == false);

            DSPI_EnableInterrupts(SPI, bitmask.bits());
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            assert(is_enabled() == false);

            DSPI_DisableInterrupts(SPI, bitmask.bits());
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(SPI->RSER);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            NVIC_EnableIRQ(SPI_IRQn);
        }

        void disable_irq()
        {
            NVIC_DisableIRQ(SPI_IRQn);
        }

        bool is_irq_enabled()
        {
            return (NVIC_GetEnableIRQ(SPI_IRQn) != 0);
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(SPI_IRQn, irq_priority);
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

        // Master modes selection
        /*enum class MasterMode
        {
            slave = 0,
            master
        };*/

        // Pin map type
        using PinMap = std::pair<PinDriver::Name, PinDriver::PinMux>;

        // Pin map array type
        template <std::size_t Size>
        using PinMapArray = std::array<PinMap, Size>;

        // -------- AVAILABLE MOSI PINS ---------------------------------------

        static constexpr std::size_t m_mosi_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 100) ? 5 : 4 };

        static constexpr PinMapArray<m_mosi_pin_map_array_size> m_mosi_pin_map_array
        { {
              { PinDriver::Name::pe_18, PinDriver::PinMux::alt2 },
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pa_16, PinDriver::PinMux::alt2 },
#endif
              { PinDriver::Name::pc_6,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_2,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_6,  PinDriver::PinMux::alt7 }
        } };

        // -------- AVAILABLE MISO PINS ---------------------------------------

        static constexpr std::size_t m_miso_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 100) ? 5 : 4 };

        static constexpr PinMapArray<m_miso_pin_map_array_size> m_miso_pin_map_array
        { {
              { PinDriver::Name::pe_19, PinDriver::PinMux::alt2 },
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pa_17, PinDriver::PinMux::alt2 },
#endif
              { PinDriver::Name::pc_7,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_3,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_7,  PinDriver::PinMux::alt7 }
        } };

        // -------- AVAILABLE SCK PINS ----------------------------------------

        static constexpr std::size_t m_sck_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 100) ? 5 : 4 };

        static constexpr PinMapArray<m_sck_pin_map_array_size> m_sck_pin_map_array
        { {
              { PinDriver::Name::pe_17, PinDriver::PinMux::alt2 },
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pa_15, PinDriver::PinMux::alt2 },
#endif
              { PinDriver::Name::pc_5,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_1,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_5,  PinDriver::PinMux::alt7 }
        } };

        // -------- AVAILABLE SLAVE SELECT PINS -------------------------------

        static constexpr std::size_t m_slave_sel_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 100) ? 6 : 5 };

        static constexpr PinMapArray<m_slave_sel_pin_map_array_size> m_slave_sel_pin_map_array
        { {
              { PinDriver::Name::pe_16, PinDriver::PinMux::alt2 },
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pa_14, PinDriver::PinMux::alt2 },
#endif
              { PinDriver::Name::pc_0,  PinDriver::PinMux::alt7 },
              { PinDriver::Name::pc_4,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_0,  PinDriver::PinMux::alt2 },
              { PinDriver::Name::pd_4,  PinDriver::PinMux::alt7 }
        } };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONFIGURATION / INITIALIZATION ----------------------------

        // Get the pin mux enumerator if the specified pin name has MOSI mux
        static constexpr PinDriver::PinMux get_mosi_pin_mux(const PinDriver::Name mosi)
        {
            return get_pin_mux(mosi, m_mosi_pin_map_array);
        }

        // Get the pin mux enumerator if the specified pin name has MISO mux
        static constexpr PinDriver::PinMux get_miso_pin_mux(const PinDriver::Name miso)
        {
            return get_pin_mux(miso, m_miso_pin_map_array);
        }

        // Get the pin mux enumerator if the specified pin name has SCK mux
        static constexpr PinDriver::PinMux get_sck_pin_mux(const PinDriver::Name sck)
        {
            return get_pin_mux(sck, m_sck_pin_map_array);
        }

        // Get the pin mux enumerator if the specified pin name has slave select mux
        static constexpr PinDriver::PinMux get_slave_sel_pin_mux(const PinDriver::Name slave_sel)
        {
            return get_pin_mux(slave_sel, m_slave_sel_pin_map_array);
        }

        template<std::size_t Size>
        static constexpr PinDriver::PinMux get_pin_mux(const PinDriver::Name pin_name, const PinMapArray<Size>& pin_map_array)
        {
            std::size_t index = 0;

            for(; index < Size; ++index)
            {
                const auto pin_map = pin_map_array[index];

                if(std::get<0>(pin_map) == pin_name)
                {
                    return std::get<1>(pin_map);
                }
            }

            // Assert pin name has SPI mux
            assert(index < Size);

            return PinDriver::PinMux::pin_disabled_or_analog;
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

            DSPI_MasterInit(SPI, &spi_master_config, SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

            // Stop frame transfers
            // ( DSPI_MasterInit(...) FSL's function decided to start frame
            // transfers by own initiative -_- )
            disable();

            // Configure CTAR1 after CTAR0 was configured in initialize
            SPI->CTAR[kDSPI_Ctar1] = SPI->CTAR[kDSPI_Ctar0];
        }

        void initialize(const SlaveConfig& slave_config)
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

            DSPI_SlaveInit(SPI, &spi_slave_config);

            // Stop frame transfers
            // ( DSPI_SlaveInit(...) FSL's function decided to start frame
            // transfers by own initiative -_- )
            disable();
        }

        bool is_master() const
        {
            return DSPI_IsMaster(SPI);
        }

        // -------- PRIVATE IRQ HANDLER ---------------------------------------

        // IRQ handler called directly by the interrupt C functions
        // (call user IRQ handler)
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(get_reference(0).m_irq_handler != nullptr)
            {
                yield = get_reference(0).m_irq_handler();
            }

            return yield;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        int32_t    m_frequency { 0 };   // User defined frequency
        IrqHandler m_irq_handler;       // User defined IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_SPI_HPP
