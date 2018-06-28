// ----------------------------------------------------------------------------
// @file    lpc84x_faim.hpp
// @brief   NXP LPC84x Fast Initialization Memory (FAIM) class.
// @date    21 June 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_FAIM_HPP
#define __XARMLIB_TARGETS_LPC84X_FAIM_HPP

#include "system/array"
#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "targets/LPC84x/lpc84x_iap.hpp"
#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_system.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class Faim
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Boot type selection
        enum class Boot
        {
            NORMAL = 0,
            LOW_POWER
        };

        // Pin configuration type
        using PinConfig = std::pair<Pin::Name, Pin::FunctionMode>;

        // Pin configuration array type
        template <std::size_t Size>
        using PinConfigArray = std::array<PinConfig, Size>;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Make sure the supplied parameters are the ones saved into FAIM
        // NOTE: If the configuration is updated a system reset is
        //       performed immediately after the FAIM Memory is written.
        template<std::size_t SIZE>
        static bool ensures(const System::Swd           swd_config,
                            const Boot                  boot_config,
                            const Pin::Name             isp_uart0_tx,
                            const Pin::Name             isp_uart0_rx,
                            const PinConfigArray<SIZE>& pin_config)
        {
            // Get intended configuration words
            const auto faim_words = get_config_words(swd_config, boot_config, isp_uart0_tx, isp_uart0_rx, pin_config);

            bool updated = false;

            for(std::size_t word_idx = 0; word_idx < faim_words.size(); ++word_idx)
            {
                uint32_t faim_value {};

                // Read current configuration
                if(Iap::read_faim_word(word_idx, faim_value) == false)
                {
                    return false;
                }

                // Compare intended with current configuration and update if needed
                if(faim_value != faim_words[word_idx])
                {
                    if(Iap::write_faim_word(word_idx, faim_words[word_idx]) == false)
                    {
                        return false;
                    }

                    updated = true;
                }
            }

            if(updated == true)
            {
                // Configuration was updated
                // Perform a system reset to make active
                NVIC_SystemReset();
            }

            return true;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Make and return the 8 words of FAIM accordingly to the supplied parameters
        template<std::size_t SIZE>
        static constexpr auto get_config_words(const System::Swd           swd_config,
                                               const Boot                  boot_config,
                                               const Pin::Name             isp_uart0_tx,
                                               const Pin::Name             isp_uart0_rx,
                                               const PinConfigArray<SIZE>& pin_config)
        {
            // Helder Parracho @ 27 March 2018
            // @REVIEW: Need GCC 7 to make the following 'static_assert()' work.
            //          Leave it commented for now.
            //static_assert(isp_uart0_tx != isp_uart0_rx);

            std::array<uint32_t, 8>faim_data    // FAIM 8 words
            {
                /* WORD0 */ (static_cast<uint32_t>(swd_config)  << WORD0_SWD_BIT)  |
                            (static_cast<uint32_t>(boot_config) << WORD0_BOOT_BIT) | WORD0_CONTENT_VALID | WORD0_ISP_UART0,
                /* WORD1 */ get_isp_uart0_word1(isp_uart0_tx, isp_uart0_rx),
                /* WORD2 */ 0x00000000,     // RESERVED
                /* WORD3 */ 0x00000000,     // RESERVED
                /* WORD4 */ 0xAAA00000,     // [PIO1_16 - PIO1_21] (default: all pins with pull-up)
                /* WORD5 */ 0xAAAAAAAA,     // [PIO1_0  - PIO1_15] (default: all pins with pull-up)
                /* WORD6 */ 0xAAAAAAAA,     // [PIO0_16 - PIO0_31] (default: all pins with pull-up)
                /* WORD7 */ 0xAAAAAAAA      // [PIO0_0  - PIO0_15] (default: all pins with pull-up)
            };

            for(std::size_t i = 0; i < pin_config.size(); ++i)
            {
                const Pin::Name pin_name = std::get<0>(pin_config[i]);

                if(pin_name != Pin::Name::NC)
                {
                    const int32_t pin_word = ((63 - static_cast<int32_t>(pin_name)) / 16) + 4;
                    const int32_t pin_bit  = ((63 - static_cast<int32_t>(pin_name)) % 16) * 2;

                    // Clear the default pin mode
                    faim_data[pin_word] &= ~(0x03 << pin_bit);

                    // Set new value
                    const Pin::FunctionMode pin_mode = std::get<1>(pin_config[i]);
                    faim_data[pin_word] |= static_cast<int32_t>(pin_mode) << pin_bit;
                }
            }

            return faim_data;
        }

        // Make and return the word1 of FAIM accordingly to the supplied parameters
        static constexpr uint32_t get_isp_uart0_word1(const Pin::Name isp_uart0_tx,
                                                      const Pin::Name isp_uart0_rx)
        {
            uint32_t pin_tx = static_cast<uint32_t>(Pin::Name::P0_25); // Default ISP USART0 TX pin: PIO0_25
            uint32_t pin_rx = static_cast<uint32_t>(Pin::Name::P0_24); // Default ISP USART0 RX pin: PIO0_24

            if(isp_uart0_tx != Pin::Name::NC)
            {
                pin_tx = static_cast<uint32_t>(isp_uart0_tx);
            }

            if(isp_uart0_rx != Pin::Name::NC)
            {
                pin_rx = static_cast<uint32_t>(isp_uart0_rx);
            }

            uint32_t word1 {};

            if(pin_tx < 32)
            {
                // PORT0
                word1 = (pin_tx << WORD1_ISP_UART0_TX_PIN_BIT) | (0 << WORD1_ISP_UART0_TX_PORT_BIT);
            }
            else
            {
                // PORT1
                word1 = ((pin_tx - 32) << WORD1_ISP_UART0_TX_PIN_BIT) | (1 << WORD1_ISP_UART0_TX_PORT_BIT);
            }

            if(pin_rx < 32)
            {
                // PORT0
                word1 |= (pin_rx << WORD1_ISP_UART0_RX_PIN_BIT) | (0 << WORD1_ISP_UART0_RX_PORT_BIT);
            }
            else
            {
                // PORT1
                word1 |= ((pin_rx - 32) << WORD1_ISP_UART0_RX_PIN_BIT) | (1 << WORD1_ISP_UART0_RX_PORT_BIT);
            }

            return word1;
        }

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // FAIM word0 bits
        enum WORD0 : uint32_t
        {
            WORD0_SWD_BIT               = (0),          // 0 => SWD disabled
                                                        // 1 => SWD enabled

            WORD0_BOOT_BIT              = (1),          // 0 => FRO/2 (12 MHz by default)
                                                        // 1 => FRO/16 (1.5 MHz by default)

            WORD0_CONTENT_VALID         = (5UL << 27),  // Bits 27 and 29 (28 is reserved; always zero)
                                                        // 101       => content valid
                                                        // Any other => content invalid

            WORD0_ISP_UART0             = (0UL << 30),  // ISP interface: UART
            WORD0_ISP_I2C0              = (1UL << 30),  // ISP interface: I2C (not implemented)
            WORD0_ISP_SPI0              = (2UL << 30)   // ISP interface: SPI (not implemented)
        };

        // FAIM word1 bits
        enum WORD1 : uint32_t
        {
            WORD1_ISP_UART0_RX_PIN_BIT  = 0,            // ISP UART0 Rx pin location within word
            WORD1_ISP_UART0_RX_PORT_BIT = 5,            // ISP UART0 Rx port location within word

            WORD1_ISP_UART0_TX_PIN_BIT  = 8,            // ISP UART0 Tx pin location within word
            WORD1_ISP_UART0_TX_PORT_BIT = 13            // ISP UART0 Tx port location within word
        };
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_FAIM_HPP
