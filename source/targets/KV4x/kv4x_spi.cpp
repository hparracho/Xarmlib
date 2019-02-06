// ----------------------------------------------------------------------------
// @file    kv4x_spi.cpp
// @brief   Kinetis KV4x SPI class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          Both sizes are 4.
// @date    5 February 2019
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

#include "core/target_specs.hpp"

#ifdef __KV4X__

#include "xarmlib_config.hpp"
#include "targets/KV4x/kv4x_spi.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void SpiDriver::initialize(const MasterConfig& master_config)
{
    const dspi_master_ctar_config_t spi_master_ctar0_config =
    {
        static_cast<uint32_t>(master_config.ctars_config.baudrate),
        static_cast<uint32_t>(master_config.ctars_config.data_bits),
        static_cast<dspi_clock_polarity_t>(static_cast<uint8_t>(master_config.ctars_config.spi_mode) >> 1),
        static_cast<dspi_clock_phase_t>(static_cast<uint8_t>(master_config.ctars_config.spi_mode) & 1),
        static_cast<dspi_shift_direction_t>(master_config.ctars_config.data_order),
        0,  // PCS to SCK delay time in nanoseconds (0 is the minimum delay)
        0,  // The last SCK to PCS delay time in nanoseconds (0 is the minimum delay)
        0   // After the SCK delay time in nanoseconds (0 is the minimum delay)
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
}

void SpiDriver::initialize(const SlaveConfig& slave_config)
{
    const dspi_slave_ctar_config_t spi_slave_ctar_config =
    {
        static_cast<uint32_t>(slave_config.ctar_config.data_bits),
        static_cast<dspi_clock_polarity_t>(static_cast<uint8_t>(slave_config.ctar_config.spi_mode) >> 1),
        static_cast<dspi_clock_phase_t>(static_cast<uint8_t>(slave_config.ctar_config.spi_mode) & 1)
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
}




void SpiDriver::config_master_ctar(const CtarSelection ctar_selection, const MasterCtarConfig& master_ctar_config)
{
    assert(is_master() == true);
    assert(is_enabled() == false);

    const auto ctar_index = static_cast<std::size_t>(ctar_selection);

    const auto frame_size     = static_cast<uint32_t>(master_ctar_config.data_bits) - 1;
    const auto clock_polarity = static_cast<uint32_t>(master_ctar_config.spi_mode) >> 1;
    const auto clock_phase    = static_cast<uint32_t>(master_ctar_config.spi_mode) & 1;
    const auto lsb_first      = static_cast<uint32_t>(master_ctar_config.data_order);

    SPI->CTAR[ctar_index] = SPI_CTAR_FMSZ(frame_size)
                          | SPI_CTAR_CPOL(clock_polarity)
                          | SPI_CTAR_CPHA(clock_phase)
                          | SPI_CTAR_LSBFE(lsb_first);

    const uint32_t result = DSPI_MasterSetBaudRate(SPI, static_cast<dspi_ctar_selection_t>(ctar_selection),
                                                        static_cast<uint32_t>(master_ctar_config.baudrate),
                                                        SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

    assert(result != 0);
}




} // namespace kv4x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv4x;

// ----------------------------------------------------------------------------
// IRQ HANDLER
// ----------------------------------------------------------------------------

extern "C" void SPI_IRQHandler(void)
{
    const int32_t yield = SpiDriver::irq_handler();

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV4X__
