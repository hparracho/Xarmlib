// ----------------------------------------------------------------------------
// @file    kv5x_can.cpp
// @brief   Kinetis KV5x Flex Controller Area Network (FlexCAN) class.
// @notes   Rx FIFO is always used (up to 6 Message Buffers).
//          6 Message Buffers are defined as Tx MB.
//          16 Rx FIFO ID filter table elements are available as Type A
//          (one full ID (standard and extended) per ID Filter element).
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

#ifdef __KV5X__

#include "core/os_support.hpp"
#include "targets/KV5x/kv5x_can.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::kv5x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    const int32_t yield = CanDriver::ored_message_buffer_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}

extern "C" void CAN0_Bus_Off_IRQHandler(void)
{
    const int32_t yield = CanDriver::bus_off_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}

extern "C" void CAN0_Error_IRQHandler(void)
{
    const int32_t yield = CanDriver::error_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}

extern "C" void CAN0_Tx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::tx_warning_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}

extern "C" void CAN0_Rx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::rx_warning_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}

/*extern "C" void CAN0_Wake_Up_IRQHandler(void)
{
    const int32_t yield = CanDriver::wake_up_irq_handler(CanDriver::Name::can0);

    Os::yield_from_isr(yield);
}*/




extern "C" void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    const int32_t yield = CanDriver::ored_message_buffer_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}

extern "C" void CAN1_Bus_Off_IRQHandler(void)
{
    const int32_t yield = CanDriver::bus_off_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}

extern "C" void CAN1_Error_IRQHandler(void)
{
    const int32_t yield = CanDriver::error_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}

extern "C" void CAN1_Tx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::tx_warning_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}

extern "C" void CAN1_Rx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::rx_warning_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}

/*extern "C" void CAN1_Wake_Up_IRQHandler(void)
{
    const int32_t yield = CanDriver::wake_up_irq_handler(CanDriver::Name::can1);

    Os::yield_from_isr(yield);
}*/




#if (TARGET_CAN_COUNT == 3)

extern "C" void CAN2_ORed_Message_buffer_IRQHandler(void)
{
    const int32_t yield = CanDriver::ored_message_buffer_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}

extern "C" void CAN2_Bus_Off_IRQHandler(void)
{
    const int32_t yield = CanDriver::bus_off_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}

extern "C" void CAN2_Error_IRQHandler(void)
{
    const int32_t yield = CanDriver::error_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}

extern "C" void CAN2_Tx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::tx_warning_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}

extern "C" void CAN2_Rx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::rx_warning_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}

/*extern "C" void CAN2_Wake_Up_IRQHandler(void)
{
    const int32_t yield = CanDriver::wake_up_irq_handler(CanDriver::Name::can2);

    Os::yield_from_isr(yield);
}*/

#endif // (TARGET_CAN_COUNT == 3)




#endif // __KV5X__
