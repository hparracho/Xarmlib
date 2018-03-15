// ----------------------------------------------------------------------------
// @file    lpc84x_syscon_clock.hpp
// @brief   NXP LPC84x SYSCON clock control class.
// @date    15 March 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
//
// Emanuel Pinto(emanuelangelopinto@gmail.com) is an official contributor of
// this library and some of the following code is based on his original work.
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

#include <cstddef>
#include <array>

#include "xarmlib_config.h"
#include "xarmlib_chrono.hpp"
#include "system/cmsis.h"

namespace xarmlib
{
namespace lpc84x
{




class Clock
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // System and peripheral clocks
        enum class Peripheral
        {
            // SYSAHBCLKCTRL0, System clock control 0 register
            SYS = 0,
            ROM,
            RAM,
         // RESERVED
            FLASH = 4,
            I2C0,
            GPIO0,
            SWM,
            SCT,
            WKT,
            MRT,
            SPI0,
            SPI1,
            CRC,
            UART0,
            UART1,
            UART2,
            WWDT,
            IOCON,
            ACOMP,
            GPIO1,
            I2C1,
            I2C2,
            I2C3,
            ADC,
            CTIMER0,
            MTB,
            DAC0,
            GPIOINT,
            DMA,
            UART3,
            UART4,

            // SYSAHBCLKCTRL1, System clock control 1 register
            CAPT = 32,
            DAC1,
        };

        // FRO (internal oscillator) available frequencies
        enum class FroFrequency
        {
            FREQ_18MHZ = 0,
            FREQ_24MHZ,
            FREQ_30MHZ
        };

        // External clock source selection
        enum class ExternalClockSource
        {
            SYS_OSC_CLK = 0,        // System oscillator clock input (crystal)
            CLK_IN,                 // Clock In pin input
        };

        // System PLL clock source selection
        enum class SystemPllSource
        {
            FRO = 0,                // Internal oscillator
            EXTERNAL_CLK,           // External clock input
            WDT_OSC_CLK,            // Watchdog oscillator
            FRO_DIV                 // FRO clock divider
        };

        // Main clock source selection
        enum class MainClockSource
        {
            FRO = 0,                // Internal oscillator
            EXTERNAL_CLK,           // External clock input
            WDT_OSC_CLK,            // Watchdog oscillator
            FRO_DIV                 // FRO clock divider
        };

        // Main clock PLL source selection
        enum class MainClockPllSource
        {
            MAIN_CLK_PRE_PLL = 0,   // Main clock select output
            SYS_PLL_CLK,            // System PLL output
            NONE = 3                // No clock input
        };

        // CLKOUT clock source selection
        enum class ClockoutSource
        {
            FRO = 0,                // Internal oscillator
            MAIN_CLK,               // Main system clock
            SYS_PLL_CLK,            // System PLL output
            EXTERNAL_CLK,           // External clock
            WDT_OSC_CLK,            // Watchdog oscillator
            NONE = 7                // No clock input
        };

        // SCT clock source selection
        enum class SctClockSource
        {
            FRO = 0,                // Internal oscillator
            MAIN_CLK,               // Main system clock
            SYS_PLL_CLK,            // System PLL output
            NONE = 3                // No clock input
        };

        // CapTouch clock source selection
        enum class CaptClockSource
        {
            FRO = 0,                // Internal oscillator
            MAIN_CLK,               // Main system clock
            SYS_PLL_CLK,            // System PLL output
            FRO_DIV,                // FRO clock divider
            WDT_OSC_CLK,            // Watchdog oscillator
            NONE = 7                // No clock input
        };

        // ADC clock source selection
        enum class AdcClockSource
        {
            FRO = 0,                // Internal oscillator
            SYS_PLL_CLK,            // System PLL output
            NONE = 3                // No clock input
        };

        // Fractional Rate Generator selection
        enum class FrgClockSelect
        {
            FRG0 = 0,
            FRG1
        };

        // Fractional Rate Generator clock source selection
        enum class FrgClockSource
        {
            FRO = 0,                // Internal oscillator
            MAIN_CLK,               // Main system clock
            SYS_PLL_CLK,            // System PLL output
            NONE = 3                // No clock input
        };

        // Peripheral to select respective clock source
        enum class PeripheralClockSelect
        {
            UART0 = 0,
            UART1,
            UART2,
            UART3,
            UART4,
            I2C0,
            I2C1,
            I2C2,
            I2C3,
            SPI0,
            SPI1
        };

        // Peripheral clock source selection
        enum class PeripheralClockSource
        {
            FRO = 0,                // Internal oscillator
            MAIN_CLK,               // Main system clock
            FRG0_CLK,               // FRG0 output
            FRG1_CLK,               // FRG1 output
            FRO_DIV,                // FRO clock divider
            NONE = 7                // No clock input
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
            if(static_cast<const uint32_t>(peripheral) < 32)
            {
                LPC_SYSCON->SYSAHBCLKCTRL0 |= (1 << static_cast<const uint32_t>(peripheral));
            }
            else
            {
                LPC_SYSCON->SYSAHBCLKCTRL1 |= (1 << (static_cast<const uint32_t>(peripheral) - 32));
            }
        }

        // Disable system or peripheral clock
        static void disable(const Peripheral peripheral)
        {
            if(static_cast<const uint32_t>(peripheral) < 32)
            {
                LPC_SYSCON->SYSAHBCLKCTRL0 &= ~(1 << static_cast<const uint32_t>(peripheral));
            }
            else
            {
                LPC_SYSCON->SYSAHBCLKCTRL1 &= ~(1 << (static_cast<const uint32_t>(peripheral) - 32));
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
            LPC_SYSCON->SYSAHBCLKDIV = static_cast<const uint32_t>(div);
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
                case FroFrequency::FREQ_18MHZ: freq_khz = 18000; /*kHz*/ break;
                case FroFrequency::FREQ_30MHZ: freq_khz = 30000; /*kHz*/ break;
                case FroFrequency::FREQ_24MHZ:
                default:                       freq_khz = 24000; /*kHz*/ break;
            }

            LPC_ROM_PWR_API->set_fro_frequency(freq_khz);

            if(fro_direct)
            {
                LPC_SYSCON->FROOSCCTRL |= FROOSCCTRL_FRO_DIRECT;
            }
            else
            {
                LPC_SYSCON->FROOSCCTRL &= ~FROOSCCTRL_FRO_DIRECT;
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
                case FroFrequency::FREQ_18MHZ: freq_hz = 18000000; /*Hz*/ break;
                case FroFrequency::FREQ_30MHZ: freq_hz = 30000000; /*Hz*/ break;
                case FroFrequency::FREQ_24MHZ:
                default:                       freq_hz = 24000000; /*Hz*/ break;
            }

            if((LPC_SYSCON->FROOSCCTRL & FROOSCCTRL_FRO_DIRECT) == 0)
            {
                const uint32_t faim_word0 = *reinterpret_cast<volatile uint32_t*>(LPC_FAIM_BASE);

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
            LPC_SYSCON->EXTCLKSEL = static_cast<const uint32_t>(source);
        }

        // Get clock source
        static ExternalClockSource get_external_clock_source()
        {
            return static_cast<ExternalClockSource>(LPC_SYSCON->EXTCLKSEL & 0x01);
        }

        // Get clock input frequency
        static int32_t get_external_clock_frequency()
        {
            // The rates are user defined in 'xarmlib_config.h'
            switch(get_external_clock_source())
            {
                case ExternalClockSource::SYS_OSC_CLK: return OSC_CLK; break; // System oscillator (crystal)
                case ExternalClockSource::CLK_IN:      return EXT_CLK; break; // CLK_IN (clock input pin)
                default:                               return 0;       break;
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
                case SystemPllSource::FRO:          return get_fro_frequency();            break;
                case SystemPllSource::EXTERNAL_CLK: return get_external_clock_frequency(); break;
                case SystemPllSource::WDT_OSC_CLK:  return get_watchdog_osc_frequency();   break;
                case SystemPllSource::FRO_DIV:      return get_fro_div_frequency();        break;
                default:                            return 0;                              break;
            }
        }

        // Get clock output frequency
        static int32_t get_system_pll_out_frequency()
        {
            const uint32_t msel = (LPC_SYSCON->SYSPLLCTRL & 0x1F) + 1;

            return get_system_pll_in_frequency() * static_cast<int32_t>(msel);
        }

        // ------------ MAIN CLOCK --------------------------------------------
        // Set clock source
        static void set_main_clock_source(const MainClockSource source)
        {
            LPC_SYSCON->MAINCLKSEL = static_cast<const uint32_t>(source);

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
                case MainClockSource::FRO:          return get_fro_frequency();            break;
                case MainClockSource::EXTERNAL_CLK: return get_external_clock_frequency(); break;
                case MainClockSource::WDT_OSC_CLK:  return get_watchdog_osc_frequency();   break;
                case MainClockSource::FRO_DIV:      return get_fro_div_frequency();        break;
                default:                            return 0;                              break;
            }
        }

        // ------------ MAIN CLOCK PLL ----------------------------------------
        // Set clock source
        static void set_main_clock_pll_source(const MainClockPllSource source)
        {
            LPC_SYSCON->MAINCLKPLLSEL = static_cast<const uint32_t>(source);

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
                 case MainClockPllSource::MAIN_CLK_PRE_PLL: return get_main_clock_frequency();     break;
                 case MainClockPllSource::SYS_PLL_CLK:      return get_system_pll_out_frequency(); break;
                 default:                                   return 0;                              break;
            }
        }

        // ------------ CLOCKOUT ---------------------------------------------
        // Set clock source
        static void set_clockout_source(const ClockoutSource source)
        {
            LPC_SYSCON->CLKOUTSEL = static_cast<const uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The CLKOUT clock rate is the clock source divided by the divider.
        static void set_clockout_divider(const uint8_t div)
        {
            LPC_SYSCON->CLKOUTDIV = static_cast<const uint32_t>(div);
        }

        // ------------ SCT CLOCK --------------------------------------------
        // Set clock source
        static void set_sct_clock_source(const SctClockSource source)
        {
            LPC_SYSCON->SCTCLKSEL = static_cast<const uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The SCT clock rate is the clock source divided by the divider.
        static void set_sct_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->SCTCLKDIV = static_cast<const uint32_t>(div);
        }

        // ------------ CAPTOUCH CLOCK ---------------------------------------
        // Set clock source
        static void set_capt_clock_source(const CaptClockSource source)
        {
            LPC_SYSCON->CAPTCLKSEL = static_cast<const uint32_t>(source);
        }

        // ------------ ADC CLOCK --------------------------------------------
        // Set clock source
        static void set_adc_clock_source(const AdcClockSource source)
        {
            LPC_SYSCON->ADCCLKSEL = static_cast<const uint32_t>(source);
        }

        // Set clock divider
        // NOTES: Use 0 to disable, or a divider value of 1 to 255.
        //        The ADC clock rate is the clock source divided by the divider.
        static void set_adc_clock_divider(const uint8_t div)
        {
            LPC_SYSCON->ADCCLKDIV = static_cast<const uint32_t>(div);
        }

        // ------------ FRACTIONAL RATE GENERATOR (FRG) CLOCK -----------------
        // Set clock source
        static void set_frg_clock_source(const FrgClockSelect frg, const FrgClockSource source)
        {
            if(frg == FrgClockSelect::FRG0)
            {
                LPC_SYSCON->FRG0CLKSEL = static_cast<const uint32_t>(source);
            }
            else
            {
                LPC_SYSCON->FRG1CLKSEL = static_cast<const uint32_t>(source);
            }
        }

        // Get clock source
        static FrgClockSource get_frg_clock_source(const FrgClockSelect frg)
        {
            if(frg == FrgClockSelect::FRG0)
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
            if(frg == FrgClockSelect::FRG0)
            {
                LPC_SYSCON->FRG0MULT = static_cast<const uint32_t>(mult);
                LPC_SYSCON->FRG0DIV  = static_cast<const uint32_t>(div);
            }
            else
            {
                LPC_SYSCON->FRG1MULT = static_cast<const uint32_t>(mult);
                LPC_SYSCON->FRG1DIV  = static_cast<const uint32_t>(div);
            }
        }

        // Get clock input frequency
        static int32_t get_frg_clock_in_frequency(const FrgClockSelect frg)
        {
            switch(get_frg_clock_source(frg))
            {
                case FrgClockSource::FRO:         return get_fro_frequency();            break;
                case FrgClockSource::MAIN_CLK:    return get_main_clock_pll_frequency(); break;
                case FrgClockSource::SYS_PLL_CLK: return get_system_pll_out_frequency(); break;
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

                if(frg == FrgClockSelect::FRG0)
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
            const int32_t index = static_cast<const int32_t>(peripheral);

            LPC_SYSCON->FCLKSEL[index] = static_cast<const uint32_t>(source);
        }

        // Get clock source
        static PeripheralClockSource get_peripheral_clock_source(const PeripheralClockSelect peripheral)
        {
            const int32_t index = static_cast<const int32_t>(peripheral);

            return static_cast<PeripheralClockSource>(LPC_SYSCON->FCLKSEL[index] & 0x07);
        }

        // Get clock output frequency
        static int32_t get_peripheral_clock_frequency(const PeripheralClockSelect peripheral)
        {
            switch(get_peripheral_clock_source(peripheral))
            {
                case PeripheralClockSource::FRO:      return get_fro_frequency();                               break;
                case PeripheralClockSource::MAIN_CLK: return get_main_clock_pll_frequency();                    break;
                case PeripheralClockSource::FRG0_CLK: return get_frg_clock_out_frequency(FrgClockSelect::FRG0); break;
                case PeripheralClockSource::FRG1_CLK: return get_frg_clock_out_frequency(FrgClockSelect::FRG1); break;
                case PeripheralClockSource::FRO_DIV:  return get_fro_div_frequency();                           break;
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
            const WatchdogFrequency frequency = static_cast<const WatchdogFrequency>((LPC_SYSCON->WDTOSCCTRL >> 5) & 0x0F);

            const uint32_t div = LPC_SYSCON->WDTOSCCTRL & 0x1F;

            // Compute clock rate and divided by divide value
            return m_watchdog_osc_frequency[static_cast<int32_t>(frequency)] / ((div + 1) << 1);
        }

        // Calculate watchdog oscillator configuration based on
        // the // specified duration value of timeout interval.
        template<typename Duration = std::chrono::microseconds, int64_t duration_value>
        static constexpr WatchdogConfig get_watchdog_osc_config()
        {
            static_assert(is_chrono_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

            constexpr int64_t duration_us = std::chrono::duration_cast<std::chrono::microseconds>(Duration(duration_value)).count();;

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
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // FRO Oscillator Control Register (FROOSCCTRL) bits
        enum FROOSCCTRL : uint32_t
        {
            FROOSCCTRL_FRO_DIRECT = (1 << 17)
        };

        // Imprecise clock rates for the watchdog oscillator
        static constexpr std::array<int32_t, 16> m_watchdog_osc_frequency =
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
            const int32_t min_freq = m_watchdog_osc_frequency.at(1) / max_freq_div / 4;

            return max_timer_count * 1000000 / min_freq;
        }
};




} // namespace lpc84x
} // namespace xarmlib

#endif  // __XARMLIB_TARGETS_LPC84X_SYSCON_CLOCK_HPP
