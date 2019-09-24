// ----------------------------------------------------------------------------
// @file    er_tft032_3.hpp
// @brief   TFT LCD ER-TFT032-3 class.
// @date    19 September 2019
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

#ifndef __XARMLIB_DEVICES_ER_TFT032_3_HPP
#define __XARMLIB_DEVICES_ER_TFT032_3_HPP

#include "ili9341.hpp"

namespace xarmlib
{




class Er_tft032_3 : public Ili9341
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Er_tft032_3(const Pin::Name   dc,
                    const Pin::Name   wr,
                    const Pin::Name   rd          = Pin::Name::nc,
                    const Pin::Name   reset       = Pin::Name::nc,
                    const Orientation orientation = Orientation::portrait) : Ili9341(dc, wr, rd, reset, orientation)
        {}

        void initialize(const Color565 background_color565 = Color565::white)
        {
            reset();

            for(const auto init_cmd : er_tft032_3_init)
            {
                write_command(init_cmd.cmd);

                for(uint8_t i = 0; i < init_cmd.arg_array_size; ++i)
                {
                    write_8bits(init_cmd.arg_array[i]);
                }

                if(init_cmd.wait_ms > 0)
                {
                    UsTicker::wait(std::chrono::milliseconds(init_cmd.wait_ms));
                }
            }

            set_orientation(m_orientation);

            fill_screen(background_color565);

            /*write_command(0xCB);
            write_8bits(0x39);
            write_8bits(0x2C);
            write_8bits(0x00);
            write_8bits(0x34);
            write_8bits(0x02);

            write_command(0xCF);
            write_8bits(0x00);
            write_8bits(0XC1);
            write_8bits(0X30);

            write_command(0xE8);
            write_8bits(0x85);
            write_8bits(0x00);
            write_8bits(0x78);

            write_command(0xEA);
            write_8bits(0x00);
            write_8bits(0x00);

            write_command(0xED);
            write_8bits(0x64);
            write_8bits(0x03);
            write_8bits(0X12);
            write_8bits(0X81);

            write_command(0xF7);
            write_8bits(0x20);

            write_command(0xC0);    //Power control
            write_8bits(0x19);   //VRH[5:0]

            write_command(0xC1);    //Power control
            write_8bits(0x11);   //SAP[2:0];BT[3:0]

            write_command(0xC5);    //VCM control
            write_8bits(0x3d);   //Contrast
            write_8bits(0x2B);

            write_command(0xC7);    //VCM control2
            write_8bits(0xC1);   //--

            write_command(0x36);    // Memory Access Control
            write_8bits(0x48);

            write_command(0x3A);
            write_8bits(0x55);

            write_command(0xB1);
            write_8bits(0x00);
            write_8bits(0x18);

            write_command(0xB6);    // Display Function Control
            write_8bits(0x08);
            write_8bits(0x82);
            write_8bits(0x27);

            write_command(0x11);    //Exit Sleep
            UsTicker::wait(std::chrono::milliseconds(120));

            write_command(0x29);    //Display on
            write_command(0x2c);

            fill_screen(0xFFFF);*/
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        // Initialization commands for TFT032-3 (also compatible for TFT028-4, TFT024-3 and TFT026-1)
        // NOTE: taken from the "Tutorial - Arduino Due (MEGA 2560,Uno)Libraries,Examples" from
        // https://www.buydisplay.com/default/arduino-3-2-inch-tft-lcd-touch-shield-ili9341-library-for-mega-due-uno
        static constexpr InitCmdArray<17, 5> er_tft032_3_init
        { {
              { CMD_PWCTRLA,  5, { 0x39, 0x2C, 0x00, 0x34, 0x02 } },
              { CMD_PWCTRLB,  3, { 0x00, 0xC1, 0x30             } },
              { CMD_DTCTRLA,  3, { 0x85, 0x00, 0x78             } },
              { CMD_DTCTRLB,  2, { 0x00, 0x00                   } },
              { CMD_PWOSCTRL, 4, { 0x64, 0x03, 0x12, 0x81       } },
              { CMD_PRCTRL,   1, { 0x20                         } },
              { CMD_PWCTRL1,  1, { 0x19                         } },
              { CMD_PWCTRL2,  1, { 0x11                         } },
              { CMD_VMCTRL1,  2, { 0x3D, 0x2B                   } },
              { CMD_VMCTRL2,  1, { 0xC1                         } },
              { CMD_MADCTL,   1, { 0x48                         } },
              { CMD_PIXSET,   1, { 0x55                         } },
              { CMD_FRMCTR1,  2, { 0x00, 0x18                   } },
              { CMD_DISCTRL,  3, { 0x08, 0x82, 0x27             } },
              { CMD_SLPOUT,   0, {                              }, 120 },
              { CMD_DISPON,   0, {                              } },
              { CMD_RAMWR,    0, {                              } }
        } };
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_ER_TFT032_3_HPP
