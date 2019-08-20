// ----------------------------------------------------------------------------
// @file    kv4x_i2c.cpp
// @brief   Kinetis KV4x I2C class.
// @note    Only master mode is implemented.
// @date    5 April 2019
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
#include "targets/KV4x/kv4x_i2c.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
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

    I2C_MasterInit(I2C, &i2c_master_config, SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
}




} // namespace kv4x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv4x;

// ----------------------------------------------------------------------------
// IRQ HANDLER
// ----------------------------------------------------------------------------

extern "C" void I2C_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler();

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV4X__
