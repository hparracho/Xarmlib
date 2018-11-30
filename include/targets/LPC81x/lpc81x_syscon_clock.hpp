// ----------------------------------------------------------------------------
// @file    lpc81x_syscon_clock.hpp
// @brief   NXP LPC81x SYSCON clock control class.
// @date    29 November 2018
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

#ifndef __XARMLIB_TARGETS_LPC81X_SYSCON_CLOCK_HPP
#define __XARMLIB_TARGETS_LPC81X_SYSCON_CLOCK_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_system.hpp"

#include <array>
#include <chrono>

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class ClockDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // System and peripheral clocks
        enum class Peripheral
        {
            SYS = 0,
            ROM,
            RAM,
            FLASHREG,
            FLASH,
            I2C,
            GPIO,
            SWM,
            SCT,
            WKT,
            MRT,
            SPI0,
            SPI1,
            CRC,
            USART0,
            USART1,
            USART2,
            WWDT,
            IOCON,
            ACOMP
        };

        // System PLL clock source selection
        enum class SystemPllSource
        {
            IRC = 0,                // Internal oscillator
            SYS_OSC_CLK = 1,        // System oscillator clock input (crystal)
            CLK_IN = 3              // Clock In pin input
        };

        // Main clock source selection
        enum class MainClockSource
        {
            IRC = 0,                // Internal oscillator
            SYS_PLL_IN_CLK,         // System PLL input
            WDT_OSC_CLK,            // Watchdog oscillator
            SYS_PLL_OUT_CLK         // System PLL output
        };

        // IOCON glitch filter clock divider selection
        // NOTE: The clock selection is actually reversed. This is not a bug!
        enum class IoconClockDividerSelect
        {
            CLKDIV6 = 0,            // IOCONCLKDIV6
            CLKDIV5,                // IOCONCLKDIV5
            CLKDIV4,                // IOCONCLKDIV4
            CLKDIV3,                // IOCONCLKDIV3
            CLKDIV2,                // IOCONCLKDIV2
            CLKDIV1,                // IOCONCLKDIV1
            CLKDIV0                 // IOCONCLKDIV0
        };

        // CLKOUT clock source selection
        enum class ClockoutSource
        {
            IRC = 0,                // Internal oscillator
            SYS_OSC_CLK,            // System oscillator clock input (crystal)
            WDT_OSC_CLK,            // Watchdog oscillator
            MAIN_CLK                // Main system clock
        };

        // Watchdog oscillator analog output frequency selection (plus or minus 40%)
        enum class WatchdogFrequency
        {
            OSC_ILLEGAL = 0,
            OSC_0_60,               // 0.6  MHz watchdog rate
            OSC_1_05,               // 1.05 MHz watchdog rate
            OSC_1_40,               // 1.4  MHz watchdog rate
            OSC_1_75,               // 1.75 MHz watchdog rate
            OSC_2_10,               // 2.1  MHz watchdog rate
            OSC_2_40,               // 2.4  MHz watchdog rate
            OSC_2_70,               // 2.7  MHz watchdog rate
            OSC_3_00,               // 3.0  MHz watchdog rate
            OSC_3_25,               // 3.25 MHz watchdog rate
            OSC_3_50,               // 3.5  MHz watchdog rate
            OSC_3_75,               // 3.75 MHz watchdog rate
            OSC_4_00,               // 4.0  MHz watchdog rate
            OSC_4_20,               // 4.2  MHz watchdog rate
            OSC_4_40,               // 4.4  MHz watchdog rate
            OSC_4_60                // 4.6  MHz watchdog rate
        };

        // Helper structure for automatic Watchdog clock configuration
        struct WatchdogConfig
        {
            WatchdogFrequency   frequency;
            uint8_t             divider;
            uint32_t            counter;
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // ------------ ENABLE / DISABLE PERIPHERAL CLOCKS --------------------

        // Enable system or peripheral clock
        static void enable(const Peripheral peripheral)
        {
            LPC_SYSCON->SYSAHBCLKCTRL |= (1 << static_cast<uint32_t>(peripheral));
        }

        // Disable system or peripheral clock
        static void disable(const Peripheral peripheral)
        {
            LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << static_cast<uint32_t>(peripheral));
        }

        // Get the system or peripheral clock state
        static bool is_enabled(const Peripheral peripheral)
        {
            return (LPC_SYSCON->SYSAHBCLKCTRL & (1 << static_cast<uint32_t>(peripheral))) != 0;
        }

        // ------------ SYSTEM OSCILLATOR -------------------------------------

        // Set System Oscillator
        // @bypass:    Flag to bypass oscillator
        // @high_freq: Flag to set oscillator range from 15-25 MHz
        // NOTE:       If bypass enabled, PLL input (sys_osc_clk) is fed directly from
        //             the XTALIN pin bypassing the oscillator. Use this mode when using
        //             an external clock source instead of the crystal oscillator.
        static void set_system_oscillator(const bool bypass, const bool high_freq)
        {
            uint32_t ctrl = 0;

            if(bypass)
            {
                ctrl |= (1 << 0);
            }

            if(high_freq)
            {
                ctrl |= (1 << 1);
            }

            LPC_SYSCON->SYSOSCCTRL = ctrl;
        }

        // ------------ SYSTEM PLL --------------------------------------------

        // Set clock source
        static void set_system_pll_source(const SystemPllSource source)
        {
            LPC_SYSCON->SYSPLLCLKSEL = static_cast<uint32_t>(source);

            // Enable system PLL clock source update (change from 0 to 1)
            LPC_SYSCON->SYSPLLCLKUEN = 0;
            LPC_SYSCON->SYSPLLCLKUEN = 1;

            // Wait for update to take effect
            while((LPC_SYSCON->SYSPLLCLKUEN & 0x01) == 0) __NOP();
        }

        // Get clock source
        static SystemPllSource get_system_pll_source()
        {
            return static_cast<SystemPllSource>(LPC_SYSCON->SYSPLLCLKSEL & 0x03);
        }

        // Set divider values
        // @msel: PLL feedback divider value
        // @psel: PLL post divider value (this doesn't divide the output of the PLL, part of feedback)
        static void set_system_pll_divider(const uint8_t msel, const uint8_t psel)
        {
            LPC_SYSCON->SYSPLLCTRL = static_cast<uint32_t>(msel & 0x1F) |
                                    (static_cast<uint32_t>(psel & 0x03) << 5);
        }

        // Get clock input rate
        static int32_t get_system_pll_in_frequency()
        {
            switch(get_system_pll_source())
            {
                case SystemPllSource::IRC:         return SystemDriver::IRC_12MHZ_FREQ;     break; // 12 MHz internal RC oscillator
                case SystemPllSource::SYS_OSC_CLK: return SystemDriver::CRYSTAL_12MHZ_FREQ; break; // System oscillator (crystal)
                case SystemPllSource::CLK_IN:      return SystemDriver::CLK_INPUT_PIN_FREQ; break; // CLK_IN (clock input pin)
                default:                           return 0;                                break;
            }
        }

        // Get clock output frequency
        static int32_t get_system_pll_out_frequency()
        {
            const uint32_t msel = (LPC_SYSCON->SYSPLLCTRL & 0x1F) + 1;

            return get_system_pll_in_frequency() * static_cast<int32_t>(msel);
        }

        // Wait for the system PLL to lock
        static void wait_system_pll_lock()
        {
            while ((LPC_SYSCON->SYSPLLSTAT & 0x1) == 0)
            {}
        }

        // ------------ MAIN CLOCK --------------------------------------------

        // Set clock source
        static void set_main_clock_source(const MainClockSource source)
        {
            LPC_SYSCON->MAINCLKSEL = static_cast<uint32_t>(source);

            // Enable main clock source update (change from 0 to 1)
            LPC_SYSCON->MAINCLKUEN = 0;
            LPC_SYSCON->MAINCLKUEN = 1;

            // Wait for update to take effect
            while((LPC_SYSCON->MAINCLKUEN & 0x01) == 0) __NOP();
        }

        // Get clock source
        static MainClockSource get_main_clock_source()
        {
            return static_cast<MainClockSource>(LPC_SYSCON->MAINCLKSEL & 0x03);
        }

        // Get clock input frequency
        static int32_t get_main_clock_frequency()
        {
            switch(get_main_clock_source())
            {
                case MainClockSource::IRC:             return SystemDriver::IRC_12MHZ_FREQ;   break;
                case MainClockSource::SYS_PLL_IN_CLK:  return get_system_pll_in_frequency();  break;
                case MainClockSource::WDT_OSC_CLK:     return get_watchdog_osc_frequency();   break;
                case MainClockSource::SYS_PLL_OUT_CLK: return get_system_pll_out_frequency(); break;
                default:                               return 0;                              break;
            }
        }

        // ------------ SYSTEM CLOCK ------------------------------------------

        // Set system clock divider
        // @div: Divider for system clock
        // NOTE: Use 0 to disable, or a divider value of 1 to 255.
        //       The system clock rate is the main system clock divided by this value.
        static void set_system_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->SYSAHBCLKDIV = static_cast<uint32_t>(div);
        }

        // Get system clock frequency
        static int32_t get_system_clock_frequency()
        {
            // No point in checking for divide by 0
            return get_main_clock_frequency() / LPC_SYSCON->SYSAHBCLKDIV;
        }

        // -------- USART(S) CLOCK --------------------------------------------

        // Set clock divider
        // @div: Divider for USART clock
        // NOTE: Use 0 to disable, or a divider value of 1 to 255.
        //       The USART clock rate is the main system clock divided by this value.
        static void set_usart_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->UARTCLKDIV = static_cast<uint32_t>(div);
        }

        // Set fractional rate generator clock divider
        // @mult: Numerator of the fractional divider. MULT is equal to the
        //        programmed value.
        // @div:  Denominator of the fractional divider. DIV is equal to the
        //        programmed value +1. Always set to 0xFF to use with the
        //        fractional baud rate generator.
        static void set_usart_frg_clock_divider(const uint8_t mult, const uint8_t div)
        {
            LPC_SYSCON->UARTFRGMULT = static_cast<uint32_t>(mult);
            LPC_SYSCON->UARTFRGDIV  = static_cast<uint32_t>(div);
        }

        // Get fractional rate generator clock input frequency
        static int32_t get_usart_frg_clock_in_frequency()
        {
            return get_main_clock_frequency() / LPC_SYSCON->UARTCLKDIV;
        }

        // Get fractional rate generator clock output frequency
        static int32_t get_usart_frg_clock_out_frequency()
        {
            // Get FRG clock input rate
            int64_t input_freq = get_usart_frg_clock_in_frequency();

            if(input_freq != 0)
            {
                const int32_t div = static_cast<int32_t>(LPC_SYSCON->UARTFRGDIV & 0xFF);

                if(div == 0xFF)
                {
                    // Fractional part is enabled, get multiplier
                    const int32_t mult = static_cast<int32_t>(LPC_SYSCON->UARTFRGMULT & 0xFF);

                    // Get fractional error
                    input_freq = (input_freq * 256) / (256 + mult);
                }
            }

            return static_cast<int32_t>(input_freq);
        }

        // -------- IOCON CLOCK -----------------------------------------------

        // Set the value of the supplied IOCON clock divider
        static void set_iocon_clock_divider(const IoconClockDividerSelect clock_div, const uint8_t div)
        {
            LPC_SYSCON->IOCONCLKDIV[static_cast<int32_t>(clock_div)] = static_cast<uint32_t>(div);
        }

        // ------------ CLOCKOUT ----------------------------------------------

        // Set clock source
        static void set_clockout_source(const ClockoutSource source)
        {
            LPC_SYSCON->CLKOUTSEL = static_cast<uint32_t>(source);

            // Enable CLKOUT clock source update (change from 0 to 1)
            LPC_SYSCON->CLKOUTUEN = 0;
            LPC_SYSCON->CLKOUTUEN = 1;

            // Wait for update to take effect
            while((LPC_SYSCON->CLKOUTUEN & 0x01) == 0) __NOP();
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The CLKOUT clock rate is the clock source divided by the divider.
        static void set_clockout_divider(const uint8_t div)
        {
            LPC_SYSCON->CLKOUTDIV = static_cast<uint32_t>(div);
        }

        // ------------ WATCHDOG OSCILLATOR -----------------------------------

        // Set watchdog oscillator analog output frequency and divider
        // @frequency: Selected watchdog analog output frequency.
        // @div:       Watchdog divider value, even value between 2 and 64.
        static void set_watchdog_osc_frequency(const WatchdogFrequency frequency, const uint8_t div)
        {
            // Watchdog output frequency = frequency / div
            LPC_SYSCON->WDTOSCCTRL = ((static_cast<uint32_t>(frequency) << 5) | (((div >> 1) - 1) & 0x1F));
        }

        // Get the estimated watchdog oscillator frequency
        // NOTE: This frequency is accurate to +/- 40%.
        static int32_t get_watchdog_osc_frequency()
        {
            // Get WDT oscillator settings
            const auto frequency = static_cast<std::size_t>((LPC_SYSCON->WDTOSCCTRL >> 5) & 0x0F);

            const auto div = static_cast<int32_t>(LPC_SYSCON->WDTOSCCTRL & 0x1F);

            // Compute clock rate and divided by divide value
            return m_watchdog_osc_frequency[frequency] / ((div + 1) << 1);
        }

        // Calculate watchdog oscillator configuration based on
        // the specified duration value of timeout interval.
        template<int64_t duration_value, typename Duration = std::chrono::microseconds>
        static constexpr WatchdogConfig get_watchdog_osc_config()
        {
            static_assert(std::chrono::is_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

            constexpr int64_t duration_us = std::chrono::duration_cast<std::chrono::microseconds>(Duration(duration_value)).count();

            static_assert(duration_us >= wdt_min_timeout_us(), "Watchdog timeout less than minimum allowed.");
            static_assert(duration_us <= wdt_max_timeout_us(), "Watchdog timeout greater than maximum allowed.");

            for(std::size_t idx = 1; idx < m_watchdog_osc_frequency.size(); ++idx)
            {
                for(uint8_t freq_div = 64; freq_div >= 2; --freq_div)
                {
                    const uint64_t freq = m_watchdog_osc_frequency[idx] / freq_div / 4;

                    const uint32_t timer_count = static_cast<uint32_t>(freq * duration_us / 1000000);

                    if(timer_count >= 0xFF && timer_count <= 0xFFFFFF)
                    {
                        return {static_cast<WatchdogFrequency>(idx), freq_div, timer_count};
                    }
                }
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Get the minimum allowed watchdog timeout interval in microseconds
        static constexpr int64_t wdt_min_timeout_us()
        {
            const int64_t min_timer_count = 0xFF;
            const int32_t min_freq_div = 2;
            const int32_t max_freq = m_watchdog_osc_frequency.back() / min_freq_div / 4;

            return min_timer_count * 1000000 / max_freq;
        }

        // Get the maximum allowed watchdog timeout interval in microseconds
        static constexpr int64_t wdt_max_timeout_us()
        {
            const int64_t max_timer_count = 0xFFFFFF;
            const int32_t max_freq_div = 64;
            const int32_t min_freq = m_watchdog_osc_frequency[1] / max_freq_div / 4;

            return max_timer_count * 1000000 / min_freq;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        // Imprecise clock rates for the watchdog oscillator
        static constexpr std::array<int32_t, 16> m_watchdog_osc_frequency
        {
            0,                      // OSC_ILLEGAL
            600000,                 // OSC_0_60
            1050000,                // OSC_1_05
            1400000,                // OSC_1_40
            1750000,                // OSC_1_75
            2100000,                // OSC_2_10
            2400000,                // OSC_2_40
            2700000,                // OSC_2_70
            3000000,                // OSC_3_00
            3250000,                // OSC_3_25
            3500000,                // OSC_3_50
            3750000,                // OSC_3_75
            4000000,                // OSC_4_00
            4200000,                // OSC_4_20
            4400000,                // OSC_4_40
            4600000                 // OSC_4_60
        };
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif  // __XARMLIB_TARGETS_LPC81X_SYSCON_CLOCK_HPP
