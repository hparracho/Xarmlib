// ----------------------------------------------------------------------------
// @file    kv5x_spi.cpp
// @brief   Kinetis KV5x SPI class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          Both sizes are 4.
// @date    17 January 2020
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

#include "core/target_specs.hpp"

#ifdef __KV5X__

#include "xarmlib_config.hpp"
#include "targets/KV5x/kv5x_spi.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void SpiDriver::initialize(const MasterConfig& master_config)
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

void SpiDriver::initialize(const SlaveConfig& slave_config)
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




void SpiDriver::set_frequency(const int32_t frequency)
{
    assert(is_enabled() == false);
    assert(is_master()  == true);
    assert(frequency     > 0);

    DSPI_MasterSetBaudRate(m_spi_base, kDSPI_Ctar0, static_cast<uint32_t>(frequency), SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

    // Copy CTAR1 after CTAR0 was changed
    m_spi_base->CTAR[kDSPI_Ctar1] = m_spi_base->CTAR[kDSPI_Ctar0];

    m_frequency = frequency;
}




} // namespace kv5x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv5x;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void SPI0_IRQHandler(void)
{
    const int32_t yield = SpiDriver::irq_handler(SpiDriver::Name::spi0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void SPI1_IRQHandler(void)
{
    const int32_t yield = SpiDriver::irq_handler(SpiDriver::Name::spi1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void SPI2_IRQHandler(void)
{
    const int32_t yield = SpiDriver::irq_handler(SpiDriver::Name::spi2);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV5X__
