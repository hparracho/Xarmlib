// ----------------------------------------------------------------------------
// @file    kv5x_xbara.hpp
// @brief   Kinetis KV5x Inter-Peripheral Crossbar Switch A (XBARA) class.
// @date    16 January 2020
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

#ifndef __XARMLIB_TARGETS_KV5X_XBARA_HPP
#define __XARMLIB_TARGETS_KV5X_XBARA_HPP

#include "fsl_xbara.h"
#include "targets/KV5x/kv5x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




class XbaraDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        enum class InputSignal
        {
            vss                     = kXBARA_InputVss,              // Logic zero output assigned to XBARA_IN0 input
            vdd                     = kXBARA_InputVdd,              // Logic one output assigned to XBARA_IN1 input
            xbar_in2                = kXBARA_InputXbarIn2,          // XBARIN2 input pin output assigned to XBARA_IN2 input
            xbar_in3                = kXBARA_InputXbarIn3,          // XBARIN3 input pin output assigned to XBARA_IN3 input
            xbar_in4                = kXBARA_InputXbarIn4,          // XBARIN4 input pin output assigned to XBARA_IN4 input
            xbar_in5                = kXBARA_InputXbarIn5,          // XBARIN5 input pin output assigned to XBARA_IN5 input
            xbar_in6                = kXBARA_InputXbarIn6,          // XBARIN6 input pin output assigned to XBARA_IN6 input
            xbar_in7                = kXBARA_InputXbarIn7,          // XBARIN7 input pin output assigned to XBARA_IN7 input
            xbar_in8                = kXBARA_InputXbarIn8,          // XBARIN8 input pin output assigned to XBARA_IN8 input
            xbar_in9                = kXBARA_InputXbarIn9,          // XBARIN9 input pin output assigned to XBARA_IN9 input
            xbar_in10               = kXBARA_InputXbarIn10,         // XBARIN10 input pin output assigned to XBARA_IN10 input
            xbar_in11               = kXBARA_InputXbarIn11,         // XBARIN11 input pin output assigned to XBARA_IN11 input
            cmp0_out                = kXBARA_InputCmp0Output,       // CMP0 Output output assigned to XBARA_IN12 input
            cmp1_out                = kXBARA_InputCmp1Output,       // CMP1 Output output assigned to XBARA_IN13 input
            cmp2_out                = kXBARA_InputCmp2Output,       // CMP2 Output output assigned to XBARA_IN14 input
            cmp3_out                = kXBARA_InputCmp3Output,       // CMP3 Output output assigned to XBARA_IN15 input
            ftm0_ch_alltrig         = kXBARA_InputFtm0Match,        // FTM0 all channels match trigger ORed together output assigned to XBARA_IN16 input
            ftm0_init               = kXBARA_InputFtm0Extrig,       // FTM0 counter init trigger output assigned to XBARA_IN17 input
            ftm3_ch_alltrig         = kXBARA_InputFtm3Match,        // FTM3 all channels match trigger ORed together output assigned to XBARA_IN18 input
            ftm3_init               = kXBARA_InputFtm3Extrig,       // FTM3 counter init trigger output assigned to XBARA_IN19 input
            pwm0_ch0_trig0          = kXBARA_InputPwm0Ch0Trg0,      // PWMA channel 0 trigger 0 output assigned to XBARA_IN20 input
            pwm0_ch0_trig1          = kXBARA_InputPwm0Ch0Trg1,      // PWMA channel 0 trigger 1 output assigned to XBARA_IN21 input
            pwm0_ch1_trig0          = kXBARA_InputPwm0Ch1Trg0,      // PWMA channel 1 trigger 0 output assigned to XBARA_IN22 input
            pwm0_ch1_trig1          = kXBARA_InputPwm0Ch1Trg1,      // PWMA channel 1 trigger 1 output assigned to XBARA_IN23 input
            pwm0_ch2_trig0          = kXBARA_InputPwm0Ch2Trg0,      // PWMA channel 2 trigger 0 output assigned to XBARA_IN24 input
            pwm0_ch2_trig1          = kXBARA_InputPwm0Ch2Trg1,      // PWMA channel 2 trigger 1 output assigned to XBARA_IN25 input
            pwm0_ch3_trig0          = kXBARA_InputPwm0Ch3Trg0,      // PWMA channel 3 trigger 0 output assigned to XBARA_IN26 input
            pwm0_ch3_trig1          = kXBARA_InputPwm0Ch3Trg1,      // PWMA channel 3 trigger 1 output assigned to XBARA_IN27 input
            pdb0_ch1_out            = kXBARA_InputPdb0Ch1Output,    // PDB0 channel 1 output trigger output assigned to XBARA_IN28 input
            pdb0_ch0_out            = kXBARA_InputPdb0Ch0Output,    // PDB0 channel 0 output trigger output assigned to XBARA_IN29 input
            pdb1_ch1_out            = kXBARA_InputPdb1Ch1Output,    // PDB1 channel 1 output trigger output assigned to XBARA_IN30 input
            pdb1_ch0_out            = kXBARA_InputPdb1Ch0Output,    // PDB1 channel 0 output trigger output assigned to XBARA_IN31 input
            hsadc1a_scan_complete   = kXBARA_InputHsadc1Cca,        // High Speed Analog-to-Digital Converter 1 conversion A complete output assigned to XBARA_IN32 input
            hsadc0a_scan_complete   = kXBARA_InputHsadc0Cca,        // High Speed Analog-to-Digital Converter 0 conversion A complete output assigned to XBARA_IN33 input
            hsadc1b_scan_complete   = kXBARA_InputHsadc1Ccb,        // High Speed Analog-to-Digital Converter 1 conversion B complete output assigned to XBARA_IN34 input
            hsadc0b_scan_complete   = kXBARA_InputHsadc0Ccb,        // High Speed Analog-to-Digital Converter 0 conversion B complete output assigned to XBARA_IN35 input
            ftm1_ch_alltrig         = kXBARA_InputFtm1Match,        // FTM1 all channels match trigger ORed together output assigned to XBARA_IN36 input
            ftm1_init               = kXBARA_InputFtm1Extrig,       // FTM1 counter init trigger output assigned to XBARA_IN37 input
            dma_ch0_done            = kXBARA_InputDmaCh0Done,       // DMA channel 0 done output assigned to XBARA_IN38 input
            dma_ch1_done            = kXBARA_InputDmaCh1Done,       // DMA channel 1 done output assigned to XBARA_IN39 input
            dma_ch6_done            = kXBARA_InputDmaCh6Done,       // DMA channel 6 done output assigned to XBARA_IN40 input
            dma_ch7_done            = kXBARA_InputDmaCh7Done,       // DMA channel 7 done output assigned to XBARA_IN41 input
            pit_ch0                 = kXBARA_InputPitTrigger0,      // PIT trigger 0 output assigned to XBARA_IN42 input
            pit_ch1                 = kXBARA_InputPitTrigger1,      // PIT trigger 1 output assigned to XBARA_IN43 input
            adc0_coco               = kXBARA_InputAdc0Coco,         // Analog-to-Digital Converter 0 conversion complete output assigned to XBARA_IN44 input
            enc_posmatch            = kXBARA_InputEnc0CmpPosMatch,  // ENC compare trigger and position match output assigned to XBARA_IN45 input
            and_or_invert_0         = kXBARA_InputAndOrInvert0,     // AOI output 0 output assigned to XBARA_IN46 input
            and_or_invert_1         = kXBARA_InputAndOrInvert1,     // AOI output 1 output assigned to XBARA_IN47 input
            and_or_invert_2         = kXBARA_InputAndOrInvert2,     // AOI output 2 output assigned to XBARA_IN48 input
            and_or_invert_3         = kXBARA_InputAndOrInvert3,     // AOI output 3 output assigned to XBARA_IN49 input
            pit_ch2                 = kXBARA_InputPitTrigger2,      // PIT trigger 2 output assigned to XBARA_IN50 input
            pit_ch3                 = kXBARA_InputPitTrigger3,      // PIT trigger 3 output assigned to XBARA_IN51 input
            pwm1_ch0_trig0_or_trig1 = kXBARA_InputPwm1Ch0Trg0OrTrg1,// PWMB channel 0 trigger 0 or trigger 1 output assigned to XBARA_IN52 input
            pwm1_ch1_trig0_or_trig1 = kXBARA_InputPwm1Ch1Trg0OrTrg1,// PWMB channel 1 trigger 0 or trigger 1 output assigned to XBARA_IN53 input
            pwm1_ch2_trig0_or_trig1 = kXBARA_InputPwm1Ch2Trg0OrTrg1,// PWMB channel 2 trigger 0 or trigger 1 output assigned to XBARA_IN54 input
            pwm1_ch3_trig0_or_trig1 = kXBARA_InputPwm1Ch3Trg0OrTrg1,// PWMB channel 3 trigger 0 or trigger 1 output assigned to XBARA_IN55 input
            ftm2_ch_alltrig         = kXBARA_InputFtm2Match,        // FTM2 all channels match trigger ORed together output assigned to XBARA_IN56 input
            ftm2_init               = kXBARA_InputFtm2Extrig        // FTM2 counter init trigger output assigned to XBARA_IN57 input
        };

        enum class OutputSignal
        {
            dmamux18          = kXBARA_OutputDmamux18,              // XBARA_OUT0 output assigned to DMAMUX slot 18
            dmamux19          = kXBARA_OutputDmamux19,              // XBARA_OUT1 output assigned to DMAMUX slot 19
            dmamux20          = kXBARA_OutputDmamux20,              // XBARA_OUT2 output assigned to DMAMUX slot 20
            dmamux21          = kXBARA_OutputDmamux21,              // XBARA_OUT3 output assigned to DMAMUX slot 21
            xbar_out4         = kXBARA_OutputXbOut4,                // XBARA_OUT4 output assigned to XBAROUT4 output pin
            xbar_out5         = kXBARA_OutputXbOut5,                // XBARA_OUT5 output assigned to XBAROUT5 output pin
            xbar_out6         = kXBARA_OutputXbOut6,                // XBARA_OUT6 output assigned to XBAROUT6 output pin
            xbar_out7         = kXBARA_OutputXbOut7,                // XBARA_OUT7 output assigned to XBAROUT7 output pin
            xbar_out8         = kXBARA_OutputXbOut8,                // XBARA_OUT8 output assigned to XBAROUT8 output pin
            xbar_out9         = kXBARA_OutputXbOut9,                // XBARA_OUT9 output assigned to XBAROUT9 output pin
            xbar_out10        = kXBARA_OutputXbOut10,               // XBARA_OUT10 output assigned to XBAROUT10 output pin
            xbar_out11        = kXBARA_OutputXbOut11,               // XBARA_OUT11 output assigned to XBAROUT11 output pin
            HSADC0A_SYNC      = kXBARA_OutputHsadc0ATrig,           // XBARA_OUT12 output assigned to HSADC0 converter A trigger
            HSADC0B_SYNC      = kXBARA_OutputHsadc0BTrig,           // XBARA_OUT13 output assigned to HSADC0 converter B trigger
            //reserved14        = kXBARA_OutputRESERVED14,            // XBARA_OUT14 output is reserved
            DAC0_12B_SYNC     = kXBARA_OutputDac12bSync,            // XBARA_OUT15 output assigned to DAC synchronisation trigger
            cmp0              = kXBARA_OutputCmp0,                  // XBARA_OUT16 output assigned to CMP0 window/sample
            cmp1              = kXBARA_OutputCmp1,                  // XBARA_OUT17 output assigned to CMP1 window/sample
            cmp2              = kXBARA_OutputCmp2,                  // XBARA_OUT18 output assigned to CMP2 window/sample
            cmp3              = kXBARA_OutputCmp3,                  // XBARA_OUT19 output assigned to CMP3 window/sample
            pwm_ch0_exta      = kXBARA_OutputPwmCh0ExtA,            // XBARA_OUT20 output assigned to PWM0 and PWM1 channel 0 external control A
            pwm_ch1_exta      = kXBARA_OutputPwmCh1ExtA,            // XBARA_OUT21 output assigned to PWM0 and PWM1 channel 1 external control A
            pwm_ch2_exta      = kXBARA_OutputPwmCh2ExtA,            // XBARA_OUT22 output assigned to PWM0 and PWM1 channel 2 external control A
            pwm_ch3_exta      = kXBARA_OutputPwmCh3ExtA,            // XBARA_OUT23 output assigned to PWM0 and PWM1 channel 3 external control A
            pwm0_ch0_ext_sync = kXBARA_OutputPwm0Ch0ExtSync,        // XBARA_OUT24 output assigned to PWM0 channel 0 external synchronization
            pwm0_ch1_ext_sync = kXBARA_OutputPwm0Ch1ExtSync,        // XBARA_OUT25 output assigned to PWM0 channel 1 external synchronization
            pwm0_ch2_ext_sync = kXBARA_OutputPwm0Ch2ExtSync,        // XBARA_OUT26 output assigned to PWM0 channel 2 external synchronization
            pwm0_ch3_ext_sync = kXBARA_OutputPwm0Ch3ExtSync,        // XBARA_OUT27 output assigned to PWM0 channel 3 external synchronization
            pwm_ext_clk       = kXBARA_OutputPwmExtClk,             // XBARA_OUT28 output assigned to PWM0 and PWM1 external clock
            pwm_fault0        = kXBARA_OutputPwm0Fault0,            // XBARA_OUT29 output assigned to PWM0 and PWM1 fault 0
            pwm_fault1        = kXBARA_OutputPwm0Fault1,            // XBARA_OUT30 output assigned to PWM0 and PWM1 fault 1
            pwm_fault2        = kXBARA_OutputPwm0Fault2,            // XBARA_OUT31 output assigned to PWM0 and PWM1 fault 2
            pwm_fault3        = kXBARA_OutputPwm0Fault3,            // XBARA_OUT32 output assigned to PWM0 and PWM1 fault 3
            pwm0_force        = kXBARA_OutputPwm0Force,             // XBARA_OUT33 output assigned to PWM0 external output force
            ftm0_trig2        = kXBARA_OutputFtm0Trig2,             // XBARA_OUT34 output assigned to FTM0 hardware trigger 2
            ftm1_trig2        = kXBARA_OutputFtm1Trig2,             // XBARA_OUT35 output assigned to FTM1 hardware trigger 2
            ftm2_trig2        = kXBARA_OutputFtm2Trig2,             // XBARA_OUT36 output assigned to FTM2 hardware trigger 2
            ftm3_trig2        = kXBARA_OutputFtm3Trig2,             // XBARA_OUT37 output assigned to FTM3 hardware trigger 2
            pdb0_in_ch12      = kXBARA_OutputPdb0InCh12,            // XBARA_OUT38 output assigned to PDB0 trigger option 12
            adc0_hdwt         = kXBARA_OutputAdc0Hdwt,              // XBARA_OUT39 output assigned to ADC0 hardware trigger
            //reserved40        = kXBARA_OutputRESERVED40,            // XBARA_OUT40 output is reserved
            pdb1_in_ch12      = kXBARA_OutputPdb1InCh12,            // XBARA_OUT41 output assigned to PDB1 trigger option 12
            HSADC1A_SYNC      = kXBARA_OutputHsadc1ATrig,           // XBARA_OUT42 output assigned to HSADC1 converter A trigger and FTM1 channel 1 signal XOR input
            HSADC1B_SYNC      = kXBARA_OutputHsadc1BTrig,           // XBARA_OUT43 output assigned to HSADC1 converter B trigger
            enc_pha           = kXBARA_OutputEncPhA,                // XBARA_OUT44 output assigned to ENC quadrature waveform phase A
            enc_phb           = kXBARA_OutputEncPhB,                // XBARA_OUT45 output assigned to ENC quadrature waveform phase B
            enc_index         = kXBARA_OutputEncIndex,              // XBARA_OUT46 output assigned to ENC refresh/reload
            enc_home          = kXBARA_OutputEncHome,               // XBARA_OUT47 output assigned to ENC home position
            enc_trig          = kXBARA_OutputEncCapTrigger,         // XBARA_OUT48 output assigned to ENC clear/snapshot
            ftm0_fault3       = kXBARA_OutputFtm0Fault3,            // XBARA_OUT49 output assigned to FTM0 fault 3
            ftm1_fault1       = kXBARA_OutputFtm1Fault1,            // XBARA_OUT50 output assigned to FTM1 fault 1
            ftm2_fault1       = kXBARA_OutputFtm2Fault1,            // XBARA_OUT51 output assigned to FTM2 fault 1
            ftm3_fault3       = kXBARA_OutputFtm3Fault3,            // XBARA_OUT52 output assigned to FTM3 fault 3
            pwm1_ch0_ext_sync = kXBARA_OutputPwm1Ch0ExtSync,        // XBARA_OUT53 output assigned to PWM0 and PWM1 channel 0 external synchronization
            pwm1_ch1_ext_sync = kXBARA_OutputPwm1Ch1ExtSync,        // XBARA_OUT54 output assigned to PWM0 and PWM1 channel 1 external synchronization
            pwm1_ch2_ext_sync = kXBARA_OutputPwm1Ch2ExtSync,        // XBARA_OUT55 output assigned to PWM0 and PWM1 channel 2 external synchronization
            pwm1_ch3_ext_sync = kXBARA_OutputPwm1Ch3ExtSync,        // XBARA_OUT56 output assigned to PWM0 and PWM1 channel 3 external synchronization
            pwm1_force        = kXBARA_OutputPwm1Force,             // XBARA_OUT57 output assigned to PWM1 external output force
            ewm_in            = kXBARA_OutputEwmIn,                 // XBARA_OUT58 output assigned to EWM input
        };

        struct InputPinConfig
        {
            PinDriver::PinMux pin_mux;
            InputSignal       input_signal;
        };

        struct OutputPinConfig
        {
            PinDriver::PinMux pin_mux;
            OutputSignal      output_signal;
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Connect the XBARA input to the selected XBARA output
        static void set_signals_connection(const InputSignal input, const OutputSignal output)
        {
            if(m_initialized == false)
            {
                m_initialized = true;

                XBARA_Init(XBARA);
            }

            XBARA_SetSignalsConnection(XBARA, static_cast<xbar_input_signal_t>(input), static_cast<xbar_output_signal_t>(output));
        }

        // Get the input pin config struct if the specified pin name has XBARA mux
        static constexpr InputPinConfig get_input_pin_config(const PinDriver::Name pin_name)
        {
            std::size_t index = 0;

            for(; index < m_input_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_input_pin_map_array[index];

                if(pin_map.first == pin_name)
                {
                    return pin_map.second;
                }
            }

            // Assert pin name has XBARA mux
            assert(index < m_input_pin_map_array.size());

            return { PinDriver::PinMux::pin_disabled_or_analog, InputSignal::xbar_in2 };
        }

        // Get the output pin config struct if the specified pin name has XBARA mux
        static constexpr OutputPinConfig get_output_pin_config(const PinDriver::Name pin_name)
        {
            std::size_t index = 0;

            for(; index < m_output_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_output_pin_map_array[index];

                if(pin_map.first == pin_name)
                {
                    return pin_map.second;
                }
            }

            // Assert pin name has XBARA mux
            assert(index < m_output_pin_map_array.size());

            return { PinDriver::PinMux::pin_disabled_or_analog, OutputSignal::xbar_out4 };
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Input pin map type
        using InputPinMap = std::pair<PinDriver::Name, InputPinConfig>;

        // Input pin map array type
        template <std::size_t Size>
        using InputPinMapArray = std::array<InputPinMap, Size>;

        // Output pin map type
        using OutputPinMap = std::pair<PinDriver::Name, OutputPinConfig>;

        // Output pin map array type
        template <std::size_t Size>
        using OutputPinMapArray = std::array<OutputPinMap, Size>;

        // -------- AVAILABLE INPUT PINS --------------------------------------

        static constexpr std::size_t m_input_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 144) ? 23 : 16 };

        static constexpr InputPinMapArray<m_input_pin_map_array_size> m_input_pin_map_array
        { {
              { PinDriver::Name::pe_0,  { PinDriver::PinMux::alt5, InputSignal::xbar_in11 } },
              { PinDriver::Name::pe_1,  { PinDriver::PinMux::alt5, InputSignal::xbar_in7  } },
              { PinDriver::Name::pe_21, { PinDriver::PinMux::alt2, InputSignal::xbar_in9  } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pe_22, { PinDriver::PinMux::alt4, InputSignal::xbar_in2  } },
              { PinDriver::Name::pe_23, { PinDriver::PinMux::alt4, InputSignal::xbar_in3  } },
#endif
              { PinDriver::Name::pe_24, { PinDriver::PinMux::alt4, InputSignal::xbar_in2  } },
              { PinDriver::Name::pe_25, { PinDriver::PinMux::alt4, InputSignal::xbar_in3  } },
              { PinDriver::Name::pa_0,  { PinDriver::PinMux::alt4, InputSignal::xbar_in4  } },
              { PinDriver::Name::pa_3,  { PinDriver::PinMux::alt4, InputSignal::xbar_in9  } },
              { PinDriver::Name::pa_4,  { PinDriver::PinMux::alt4, InputSignal::xbar_in10 } },
              { PinDriver::Name::pa_18, { PinDriver::PinMux::alt2, InputSignal::xbar_in7  } },
              { PinDriver::Name::pa_19, { PinDriver::PinMux::alt2, InputSignal::xbar_in8  } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pa_24, { PinDriver::PinMux::alt2, InputSignal::xbar_in4  } },
              { PinDriver::Name::pa_25, { PinDriver::PinMux::alt2, InputSignal::xbar_in5  } },
#endif
              { PinDriver::Name::pb_16, { PinDriver::PinMux::alt7, InputSignal::xbar_in5  } },
              { PinDriver::Name::pc_1,  { PinDriver::PinMux::alt6, InputSignal::xbar_in11 } },
              { PinDriver::Name::pc_2,  { PinDriver::PinMux::alt6, InputSignal::xbar_in6  } },
              { PinDriver::Name::pc_5,  { PinDriver::PinMux::alt4, InputSignal::xbar_in2  } },
              { PinDriver::Name::pc_6,  { PinDriver::PinMux::alt4, InputSignal::xbar_in3  } },
              { PinDriver::Name::pc_7,  { PinDriver::PinMux::alt4, InputSignal::xbar_in4  } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pd_12, { PinDriver::PinMux::alt4, InputSignal::xbar_in5  } },
              { PinDriver::Name::pd_13, { PinDriver::PinMux::alt4, InputSignal::xbar_in7  } },
              { PinDriver::Name::pd_14, { PinDriver::PinMux::alt4, InputSignal::xbar_in11 } }
#endif
        } };

        // -------- AVAILABLE OUTPUT PINS -------------------------------------

        static constexpr std::size_t m_output_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 144) ? 11 : 8 };

        static constexpr OutputPinMapArray<m_output_pin_map_array_size> m_output_pin_map_array
        { {
              { PinDriver::Name::pe_0,  { PinDriver::PinMux::alt4, OutputSignal::xbar_out10 } },
              { PinDriver::Name::pe_1,  { PinDriver::PinMux::alt4, OutputSignal::xbar_out11 } },
              { PinDriver::Name::pe_24, { PinDriver::PinMux::alt7, OutputSignal::xbar_out4  } },
              { PinDriver::Name::pe_25, { PinDriver::PinMux::alt7, OutputSignal::xbar_out5  } },
              { PinDriver::Name::pa_18, { PinDriver::PinMux::alt5, OutputSignal::xbar_out8  } },
              { PinDriver::Name::pa_19, { PinDriver::PinMux::alt5, OutputSignal::xbar_out9  } },
              { PinDriver::Name::pc_6,  { PinDriver::PinMux::alt6, OutputSignal::xbar_out6  } },
              { PinDriver::Name::pc_7,  { PinDriver::PinMux::alt6, OutputSignal::xbar_out7  } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pd_12, { PinDriver::PinMux::alt5, OutputSignal::xbar_out5  } },
              { PinDriver::Name::pd_13, { PinDriver::PinMux::alt5, OutputSignal::xbar_out7  } },
              { PinDriver::Name::pd_14, { PinDriver::PinMux::alt5, OutputSignal::xbar_out11 } }
#endif
        } };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        inline static bool m_initialized { false };
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV5X_XBARA_HPP
