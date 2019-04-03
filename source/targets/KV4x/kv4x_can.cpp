// ----------------------------------------------------------------------------
// @file    kv4x_can.cpp
// @brief   Kinetis KV4x Flex Controller Area Network (FlexCAN) class.
// @notes   Rx FIFO is always used (up to 6 Message Buffers).
//          6 Message Buffers are defined as Tx MB.
//          16 Rx FIFO ID filter table elements are available as Type A
//          (one full ID (standard and extended) per ID Filter element).
// @date    22 March 2019
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
#include "targets/KV4x/kv4x_can.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void CanDriver::initialize(const Config& config)
{
    const flexcan_timing_config_t timing_config =
    {
        // According to the http://www.bittiming.can-wiki.info/#Freescale
        // with clock rate at 80 MHz, Sample-Point at 87.5 % and SJW 1,
        // and a bit time consisting of 16 time quanta, the protocol
        // timing configuration is:
        0,  // Clock Pre-scaler Division Factor (it will be calculated)
        1,  // Re-Sync Jump Width
        7,  // Phase Segment 1
        1,  // Phase Segment 2
        4   // Propagation Segment
    };

    const flexcan_config_t can_config =
    {
        static_cast<uint32_t>(config.baudrate),
        kFLEXCAN_ClkSrcPeri,
        kFLEXCAN_WakeupSrcUnfiltered,
        16,     // Maximum number of Message Buffers
        static_cast<bool>(config.loop_back_mode),
        true,   // Enable timer synchronization
        false,  // Disable Self Wakeup Mode
        true,   // Enable Rx Individual Mask
        false,  // Disable Doze Mode
        timing_config
    };

    FLEXCAN_Init(m_can_base, &can_config, SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_1), true);
    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_2), true);
    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_3), true);
    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_4), true);
    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_5), true);
    FLEXCAN_SetTxMbConfig(m_can_base, static_cast<uint8_t>(TxMessageBuffer::number_6), true);
}




} // namespace kv4x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv4x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    const int32_t yield = CanDriver::ored_message_buffer_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN0_Bus_Off_IRQHandler(void)
{
    const int32_t yield = CanDriver::bus_off_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN0_Error_IRQHandler(void)
{
    const int32_t yield = CanDriver::error_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN0_Tx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::tx_warning_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN0_Rx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::rx_warning_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

/*extern "C" void CAN0_Wake_Up_IRQHandler(void)
{
    const int32_t yield = CanDriver::wake_up_irq_handler(CanDriver::Name::can0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}*/




#if (TARGET_CAN_COUNT == 2)

extern "C" void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    const int32_t yield = CanDriver::ored_message_buffer_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN1_Bus_Off_IRQHandler(void)
{
    const int32_t yield = CanDriver::bus_off_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN1_Error_IRQHandler(void)
{
    const int32_t yield = CanDriver::error_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN1_Tx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::tx_warning_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void CAN1_Rx_Warning_IRQHandler(void)
{
    const int32_t yield = CanDriver::rx_warning_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

/*extern "C" void CAN1_Wake_Up_IRQHandler(void)
{
    const int32_t yield = CanDriver::wake_up_irq_handler(CanDriver::Name::can1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}*/

#endif // (TARGET_CAN_COUNT == 2)




#endif // __KV4X__
