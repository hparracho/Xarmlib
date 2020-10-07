// ----------------------------------------------------------------------------
// @file    lpc84x_syscon_clock.hpp
// @brief   NXP LPC84x SYSCON clock control class.
// @date    21 August 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_LPC84X_SYSCON_CLOCK_HPP
#define __XARMLIB_TARGETS_LPC84X_SYSCON_CLOCK_HPP

#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "targets/LPC84x/lpc84x_flash_iap.hpp"
#include "targets/LPC84x/lpc84x_system.hpp"

#include <array>
#include <chrono>

namespace xarmlib
{
namespace targets
{
namespace lpc84x
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
            // SYSAHBCLKCTRL0, System clock control 0 register
            sys = 0,
            rom,
            ram,
         // RESERVED
            flash = 4,
            i2c0,
            gpio0,
            swm,
            sct,
            wkt,
            mrt,
            spi0,
            spi1,
            crc,
            usart0,
            usart1,
            usart2,
            wwdt,
            iocon,
            acomp,
            gpio1,
            i2c1,
            i2c2,
            i2c3,
            adc,
            ctimer0,
            mtb,
            dac0,
            gpioint,
            dma,
            usart3,
            usart4,

            // SYSAHBCLKCTRL1, System clock control 1 register
            capt = 32,
            dac1,
        };

        // FRO (internal oscillator) available frequencies
        enum class FroFrequency
        {
            freq_18mhz = 0,
            freq_24mhz,
            freq_30mhz
        };

        // External clock source selection
        enum class ExternalClockSource
        {
            sys_osc_clk = 0,        // System oscillator clock input (crystal)
            clk_in,                 // Clock In pin input
        };

        // System PLL clock source selection
        enum class SystemPllSource
        {
            fro = 0,                // Internal oscillator
            external_clk,           // External clock input
            wdt_osc_clk,            // Watchdog oscillator
            fro_div                 // FRO clock divider
        };

        // Main clock source selection
        enum class MainClockSource
        {
            fro = 0,                // Internal oscillator
            external_clk,           // External clock input
            wdt_osc_clk,            // Watchdog oscillator
            fro_div                 // FRO clock divider
        };

        // Main clock PLL source selection
        enum class MainClockPllSource
        {
            main_clk_pre_pll = 0,   // Main clock select output
            sys_pll_clk,            // System PLL output
            none = 3                // No clock input
        };

        // CLKOUT clock source selection
        enum class ClockoutSource
        {
            fro = 0,                // Internal oscillator
            main_clk,               // Main system clock
            sys_pll_clk,            // System PLL output
            external_clk,           // External clock
            wdt_osc_clk,            // Watchdog oscillator
            none = 7                // No clock input
        };

        // SCT clock source selection
        enum class SctClockSource
        {
            fro = 0,                // Internal oscillator
            main_clk,               // Main system clock
            sys_pll_clk,            // System PLL output
            none = 3                // No clock input
        };

        // CapTouch clock source selection
        enum class CaptClockSource
        {
            fro = 0,                // Internal oscillator
            main_clk,               // Main system clock
            sys_pll_clk,            // System PLL output
            fro_div,                // FRO clock divider
            wdt_osc_clk,            // Watchdog oscillator
            none = 7                // No clock input
        };

        // ADC clock source selection
        enum class AdcClockSource
        {
            fro = 0,                // Internal oscillator
            sys_pll_clk,            // System PLL output
            none = 3                // No clock input
        };

        // Fractional Rate Generator selection
        enum class FrgClockSelect
        {
            frg0 = 0,
            frg1
        };

        // Fractional Rate Generator clock source selection
        enum class FrgClockSource
        {
            fro = 0,                // Internal oscillator
            main_clk,               // Main system clock
            sys_pll_clk,            // System PLL output
            none = 3                // No clock input
        };

        // Peripheral to select respective clock source
        enum class PeripheralClockSelect
        {
            usart0 = 0,
            usart1,
            usart2,
            usart3,
            usart4,
            i2c0,
            i2c1,
            i2c2,
            i2c3,
            spi0,
            spi1
        };

        // Peripheral clock source selection
        enum class PeripheralClockSource
        {
            fro = 0,                // Internal oscillator
            main_clk,               // Main system clock
            frg0_clk,               // FRG0 output
            frg1_clk,               // FRG1 output
            fro_div,                // FRO clock divider
            none = 7                // No clock input
        };

        // Watchdog oscillator analog output frequency selection (plus or minus 40%)
        enum class WatchdogFrequency
        {
            osc_illegal = 0,
            osc_0_60,               // 0.6  MHz watchdog rate
            osc_1_05,               // 1.05 MHz watchdog rate
            osc_1_40,               // 1.4  MHz watchdog rate
            osc_1_75,               // 1.75 MHz watchdog rate
            osc_2_10,               // 2.1  MHz watchdog rate
            osc_2_40,               // 2.4  MHz watchdog rate
            osc_2_70,               // 2.7  MHz watchdog rate
            osc_3_00,               // 3.0  MHz watchdog rate
            osc_3_25,               // 3.25 MHz watchdog rate
            osc_3_50,               // 3.5  MHz watchdog rate
            osc_3_75,               // 3.75 MHz watchdog rate
            osc_4_00,               // 4.0  MHz watchdog rate
            osc_4_20,               // 4.2  MHz watchdog rate
            osc_4_40,               // 4.4  MHz watchdog rate
            osc_4_60                // 4.6  MHz watchdog rate
        };

        // Helper structure for automatic Watchdog clock configuration
        struct WatchdogConfig
        {
            WatchdogFrequency   frequency;
            uint8_t             divider;
            uint32_t            counter;
        };

        // IOCON glitch filter clock divider selection
        // NOTE: The clock selection is actually reversed. This is not a bug!
        enum class IoconClockDividerSelect
        {
            clkdiv6 = 0,            // IOCONCLKDIV6
            clkdiv5,                // IOCONCLKDIV5
            clkdiv4,                // IOCONCLKDIV4
            clkdiv3,                // IOCONCLKDIV3
            clkdiv2,                // IOCONCLKDIV2
            clkdiv1,                // IOCONCLKDIV1
            clkdiv0                 // IOCONCLKDIV0
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // ------------ ENABLE / DISABLE PERIPHERAL CLOCKS --------------------

        // Enable system or peripheral clock
        static void enable(const Peripheral peripheral)
        {
            if(static_cast<uint32_t>(peripheral) < 32)
            {
                LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << static_cast<uint32_t>(peripheral));
            }
            else
            {
                LPC_SYSCON->SYSAHBCLKCTRL1 |= (1 << (static_cast<uint32_t>(peripheral) - 32));
            }
        }

        // Disable system or peripheral clock
        static void disable(const Peripheral peripheral)
        {
            if(static_cast<uint32_t>(peripheral) < 32)
            {
                LPC_SYSCON->SYSAHBCLKCTRL0 &= ~(1 << static_cast<uint32_t>(peripheral));
            }
            else
            {
                LPC_SYSCON->SYSAHBCLKCTRL1 &= ~(1 << (static_cast<uint32_t>(peripheral) - 32));
            }
        }

        // Get the system or peripheral clock state
        static bool is_enabled(const Peripheral peripheral)
        {
            if(static_cast<uint32_t>(peripheral) < 32)
            {
                return (LPC_SYSCON->SYSAHBCLKCTRL0 & (1 << static_cast<uint32_t>(peripheral))) != 0;
            }
            else
            {
                return (LPC_SYSCON->SYSAHBCLKCTRL1 & (1 << (static_cast<uint32_t>(peripheral) - 32))) != 0;
            }
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

        // ------------ SYSTEM CLOCK ------------------------------------------

        // Set system clock divider
        // @div: Divider for system clock.
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
            return get_main_clock_pll_frequency() / LPC_SYSCON->SYSAHBCLKDIV;
        }

        // ------------ FREE RUNNING OSCILLATOR (FRO) -------------------------

        // Set clock frequency
        // NOTE: If fro_direct = false, FRO also depends on the FAIM setting.
        static void set_fro_frequency(const FroFrequency fro_frequency, const bool fro_direct)
        {
            uint32_t freq_khz;

            switch(fro_frequency)
            {
                case FroFrequency::freq_18mhz: freq_khz = 18000; /*kHz*/ break;
                case FroFrequency::freq_30mhz: freq_khz = 30000; /*kHz*/ break;
                case FroFrequency::freq_24mhz:
                default:                       freq_khz = 24000; /*kHz*/ break;
            }

            LPC_ROM_PWR_API->set_fro_frequency(freq_khz);

            if(fro_direct)
            {
                LPC_SYSCON->FROOSCCTRL |= frooscctrl_fro_direct;
            }
            else
            {
                LPC_SYSCON->FROOSCCTRL &= ~frooscctrl_fro_direct;
            }

            // Enable FRO direct update (change from 0 to 1)
            LPC_SYSCON->FRODIRECTCLKUEN = 0;
            LPC_SYSCON->FRODIRECTCLKUEN = 1;

            // Wait for update to take effect
            while((LPC_SYSCON->FRODIRECTCLKUEN & 0x01) == 0) __NOP();
        }

        // Get FRO (internal oscillator) clock frequency
        static int32_t get_fro_frequency()
        {
            // Grant FRO direct is updated to read next (change from 0 to 1)
            LPC_SYSCON->FRODIRECTCLKUEN = 0;
            LPC_SYSCON->FRODIRECTCLKUEN = 1;

            const auto fro_frequency = static_cast<FroFrequency>(LPC_SYSCON->FROOSCCTRL & 0x03);

            int32_t freq_hz;

            switch(fro_frequency)
            {
                case FroFrequency::freq_18mhz: freq_hz = 18000000; /*Hz*/ break;
                case FroFrequency::freq_30mhz: freq_hz = 30000000; /*Hz*/ break;
                case FroFrequency::freq_24mhz:
                default:                       freq_hz = 24000000; /*Hz*/ break;
            }

            if((LPC_SYSCON->FROOSCCTRL & frooscctrl_fro_direct) == 0)
            {
                uint32_t faim_word0 {};

                // Read current FAIM word 0
                if(FlashIapDriver::read_faim_word(0, faim_word0) == false)
                {
                    return 0;
                }

                if((faim_word0 & (1 << 1)) == 0)
                {
                    // FAIM word0, low power boot bit = 0 -> divide by 2
                    freq_hz /= 2;
                }
                else
                {
                    // FAIM word0, low power boot bit = 1 -> divide by 16
                    freq_hz /= 16;
                }
            }

            return freq_hz;
        }

        // Get FRO DIV (internal oscillator divided by 2) clock frequency
        static int32_t get_fro_div_frequency()
        {
            return get_fro_frequency() / 2;
        }

        // ------------ EXTERNAL CLOCK (CRYSTAL / CLOCK INPUT PIN) ------------

        // Set clock source
        static void set_external_clock_source(const ExternalClockSource source)
        {
            LPC_SYSCON->EXTCLKSEL = static_cast<uint32_t>(source);
        }

        // Get clock source
        static ExternalClockSource get_external_clock_source()
        {
            return static_cast<ExternalClockSource>(LPC_SYSCON->EXTCLKSEL & 0x01);
        }

        // Get clock input frequency
        static int32_t get_external_clock_frequency()
        {
            // The frequencies are fixed and defined in 'lpc84x_target.hpp'
            switch(get_external_clock_source())
            {
                case ExternalClockSource::sys_osc_clk: return SystemDriver::CRYSTAL_12MHZ_FREQ; break; // System oscillator (crystal)
                case ExternalClockSource::clk_in:      return SystemDriver::CLK_INPUT_PIN_FREQ; break; // CLK_IN (clock input pin)
                default:                               return 0;                                break;
            }
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
                case SystemPllSource::fro:          return get_fro_frequency();            break;
                case SystemPllSource::external_clk: return get_external_clock_frequency(); break;
                case SystemPllSource::wdt_osc_clk:  return get_watchdog_osc_frequency();   break;
                case SystemPllSource::fro_div:      return get_fro_div_frequency();        break;
                default:                            return 0;                              break;
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
                case MainClockSource::fro:          return get_fro_frequency();            break;
                case MainClockSource::external_clk: return get_external_clock_frequency(); break;
                case MainClockSource::wdt_osc_clk:  return get_watchdog_osc_frequency();   break;
                case MainClockSource::fro_div:      return get_fro_div_frequency();        break;
                default:                            return 0;                              break;
            }
        }

        // ------------ MAIN CLOCK PLL ----------------------------------------

        // Set clock source
        static void set_main_clock_pll_source(const MainClockPllSource source)
        {
            LPC_SYSCON->MAINCLKPLLSEL = static_cast<uint32_t>(source);

            // Enable main clock PLL source update (change from 0 to 1)
            LPC_SYSCON->MAINCLKPLLUEN = 0;
            LPC_SYSCON->MAINCLKPLLUEN = 1;

            // Wait for update to take effect
            while((LPC_SYSCON->MAINCLKPLLUEN & 0x01) == 0) __NOP();
        }

        // Get clock source
        static MainClockPllSource get_main_clock_pll_source()
        {
            return static_cast<MainClockPllSource>(LPC_SYSCON->MAINCLKPLLSEL & 0x03);
        }

        // Get clock input frequency
        static int32_t get_main_clock_pll_frequency()
        {
            switch(get_main_clock_pll_source())
            {
                 case MainClockPllSource::main_clk_pre_pll: return get_main_clock_frequency();     break;
                 case MainClockPllSource::sys_pll_clk:      return get_system_pll_out_frequency(); break;
                 default:                                   return 0;                              break;
            }
        }

        // ------------ CLOCKOUT ---------------------------------------------

        // Set clock source
        static void set_clockout_source(const ClockoutSource source)
        {
            LPC_SYSCON->CLKOUTSEL = static_cast<uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The CLKOUT clock rate is the clock source divided by the divider.
        static void set_clockout_divider(const uint8_t div)
        {
            LPC_SYSCON->CLKOUTDIV = static_cast<uint32_t>(div);
        }

        // ------------ SCT CLOCK --------------------------------------------

        // Set clock source
        static void set_sct_clock_source(const SctClockSource source)
        {
            LPC_SYSCON->SCTCLKSEL = static_cast<uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The SCT clock rate is the clock source divided by the divider.
        static void set_sct_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->SCTCLKDIV = static_cast<uint32_t>(div);
        }

        // ------------ CAPTOUCH CLOCK ---------------------------------------

        // Set clock source
        static void set_capt_clock_source(const CaptClockSource source)
        {
            LPC_SYSCON->CAPTCLKSEL = static_cast<uint32_t>(source);
        }

        // ------------ ADC CLOCK --------------------------------------------

        // Set clock source
        static void set_adc_clock_source(const AdcClockSource source)
        {
            LPC_SYSCON->ADCCLKSEL = static_cast<uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The ADC clock rate is the clock source divided by the divider.
        static void set_adc_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->ADCCLKDIV = static_cast<uint32_t>(div);
        }

        // ------------ FRACTIONAL RATE GENERATOR (FRG) CLOCK -----------------

        // Set clock source
        static void set_frg_clock_source(const FrgClockSelect frg, const FrgClockSource source)
        {
            if(frg == FrgClockSelect::frg0)
            {
                LPC_SYSCON->FRG0CLKSEL = static_cast<uint32_t>(source);
            }
            else
            {
                LPC_SYSCON->FRG1CLKSEL = static_cast<uint32_t>(source);
            }
        }

        // Get clock source
        static FrgClockSource get_frg_clock_source(const FrgClockSelect frg)
        {
            if(frg == FrgClockSelect::frg0)
            {
                return static_cast<FrgClockSource>(LPC_SYSCON->FRG0CLKSEL & 0x03);
            }
            else
            {
                return static_cast<FrgClockSource>(LPC_SYSCON->FRG1CLKSEL & 0x03);
            }
        }

        // Set clock fractional divider
        // @mult: Numerator of the fractional divider. MULT is equal to the
        //        programmed value.
        // @div:  Denominator of the fractional divider. DIV is equal to the
        //        programmed value +1. Always set to 0xFF to use with the
        //        fractional baud rate generator.
        static void set_frg_clock_divider(const FrgClockSelect frg, const uint8_t mult, const uint8_t div)
        {
            if(frg == FrgClockSelect::frg0)
            {
                LPC_SYSCON->FRG0MULT = static_cast<uint32_t>(mult);
                LPC_SYSCON->FRG0DIV  = static_cast<uint32_t>(div);
            }
            else
            {
                LPC_SYSCON->FRG1MULT = static_cast<uint32_t>(mult);
                LPC_SYSCON->FRG1DIV  = static_cast<uint32_t>(div);
            }
        }

        // Get clock input frequency
        static int32_t get_frg_clock_in_frequency(const FrgClockSelect frg)
        {
            switch(get_frg_clock_source(frg))
            {
                case FrgClockSource::fro:         return get_fro_frequency();            break;
                case FrgClockSource::main_clk:    return get_main_clock_pll_frequency(); break;
                case FrgClockSource::sys_pll_clk: return get_system_pll_out_frequency(); break;
                default:                          return 0;                              break;
            }
        }

        // Get clock output frequency
        static int32_t get_frg_clock_out_frequency(const FrgClockSelect frg)
        {
            // Get FRG clock input rate
            int64_t input_freq = get_frg_clock_in_frequency(frg);

            if(input_freq != 0)
            {
                int32_t mult, div;

                if(frg == FrgClockSelect::frg0)
                {
                    mult = static_cast<int32_t>(LPC_SYSCON->FRG0MULT & 0xFF);
                    div  = static_cast<int32_t>(LPC_SYSCON->FRG0DIV  & 0xFF);
                }
                else
                {
                    mult = static_cast<int32_t>(LPC_SYSCON->FRG1MULT & 0xFF);
                    div  = static_cast<int32_t>(LPC_SYSCON->FRG1DIV  & 0xFF);
                }

                if(div== 0xFF)
                {
                    // Fractional divider is enabled. Get final frequency
                    // taking into account the fractional divider.
                    input_freq = (input_freq * 256) / (256 + mult);
                }
            }

            return static_cast<int32_t>(input_freq);
        }

        // ------------ PERIPHERALS CLOCK -------------------------------------

        // Set clock source
        static void set_peripheral_clock_source(const PeripheralClockSelect peripheral, const PeripheralClockSource source)
        {
            const int32_t index = static_cast<int32_t>(peripheral);

            LPC_SYSCON->FCLKSEL[index] = static_cast<uint32_t>(source);
        }

        // Get clock source
        static PeripheralClockSource get_peripheral_clock_source(const PeripheralClockSelect peripheral)
        {
            const int32_t index = static_cast<int32_t>(peripheral);

            return static_cast<PeripheralClockSource>(LPC_SYSCON->FCLKSEL[index] & 0x07);
        }

        // Get clock output frequency
        static int32_t get_peripheral_clock_frequency(const PeripheralClockSelect peripheral)
        {
            switch(get_peripheral_clock_source(peripheral))
            {
                case PeripheralClockSource::fro:      return get_fro_frequency();                               break;
                case PeripheralClockSource::main_clk: return get_main_clock_pll_frequency();                    break;
                case PeripheralClockSource::frg0_clk: return get_frg_clock_out_frequency(FrgClockSelect::frg0); break;
                case PeripheralClockSource::frg1_clk: return get_frg_clock_out_frequency(FrgClockSelect::frg1); break;
                case PeripheralClockSource::fro_div:  return get_fro_div_frequency();                           break;
                default:                              return 0;                                                 break;
            }
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
        // the // specified duration value of timeout interval.
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

        // -------- IOCON CLOCK DIVIDER SELECTION -----------------------------

        // Set the value of the supplied IOCON clock divider
        static void set_iocon_clock_divider(const IoconClockDividerSelect clock_div, const uint8_t div)
        {
            LPC_SYSCON->IOCONCLKDIV[static_cast<int32_t>(clock_div)] = div;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // FRO Oscillator Control Register (FROOSCCTRL) bits
        enum FROOSCCTRL : uint32_t
        {
            frooscctrl_fro_direct = (1 << 17)
        };

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




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif  // __XARMLIB_TARGETS_LPC84X_SYSCON_CLOCK_HPP
