// ----------------------------------------------------------------------------
// @file    kv5x_i2c.cpp
// @brief   Kinetis KV5x I2C class.
// @note    Only master mode is implemented.
// @date    23 January 2020
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
#include "targets/KV5x/kv5x_i2c.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void I2cDriver::initialize(const MasterConfig& master_config)
{
    assert(master_config.baudrate > 0);

    const uint32_t baudrate = static_cast<uint32_t>(master_config.baudrate);

    const i2c_master_config_t i2c_master_config =
    {
        false,  // Disable the peripheral at initialization time
        false,  // Disable the stop hold
        baudrate,
        0       // No glitch filter
    };

    I2C_MasterInit(m_i2c_base, &i2c_master_config, SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
}




} // namespace kv5x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv5x;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void I2C0_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void I2C1_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV5X__
