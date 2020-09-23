// ----------------------------------------------------------------------------
// @file    lpc84x_i2c.cpp
// @brief   NXP LPC84x I2C class.
// @note    Only master mode is implemented.
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

#include "core/target_specs.hpp"

#ifdef __LPC84X__

#include "core/os_support.hpp"
#include "targets/LPC84x/lpc84x_i2c.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::lpc84x;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void I2C0_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c0);

    Os::yield_from_isr(yield);
}




extern "C" void I2C1_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c1);

    Os::yield_from_isr(yield);
}




#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */

extern "C" void I2C2_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c2);

    Os::yield_from_isr(yield);
}




extern "C" void I2C3_IRQHandler(void)
{
    const int32_t yield = I2cDriver::irq_handler(I2cDriver::Name::i2c3);

    Os::yield_from_isr(yield);
}

#endif // (TARGET_I2C_COUNT == 4) /* __LPC845__ */




#endif // __LPC84X__
