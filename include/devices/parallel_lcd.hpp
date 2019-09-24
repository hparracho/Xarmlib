// ----------------------------------------------------------------------------
// @file    parallel_lcd.hpp
// @brief   LCD base class using 8080 MPU 8-bit Parallel Interface II for KV4x.
// @note    8-bit data bus must be connected to the PORT D:
//          PD_0 -> DB8 ... PD_7 -> DB15
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

#ifndef __XARMLIB_DEVICES_PARALLEL_LCD_HPP
#define __XARMLIB_DEVICES_PARALLEL_LCD_HPP

#include "font.hpp"
#include "api/api_digital_out.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{




// Get a packed 16-bit color value in '565' RGB format
// red, green, blue: 8-bit brightnesss (0 = off, 255 = max)
constexpr uint16_t get_color565(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    return (static_cast<uint16_t>(red & 0xF8) << 8) | (static_cast<uint16_t>(green & 0xFC) << 3) | static_cast<uint16_t>(blue >> 3);
}




class ParallelLcd
{
	public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // LCD viewing position
        enum class Orientation
        {
            portrait = 0,
            landscape,
            portrait_rotated,
            landscape_rotated
        };

        // Color basic enumeration (16-bit color value in '565' RGB format)
        enum class Color565 : uint16_t
        {
            black        = get_color565(  0,   0,   0),
            blue         = get_color565(  0,   0, 255),
            cyan         = get_color565(  0, 255, 255),
            dark_cyan    = get_color565(  0, 125, 123),
            dark_green   = get_color565(  0, 125,   0),
            dark_grey    = get_color565(123, 125, 123),
            green        = get_color565(  0, 255,   0),
            green_yellow = get_color565(173, 255,  41),
            light_grey   = get_color565(198, 195, 198),
            magenta      = get_color565(255,   0, 255),
            maroon       = get_color565(123,   0,   0),
            navy         = get_color565(  0,   0, 123),
            olive        = get_color565(123, 125,   0),
            orange       = get_color565(255, 165,   0),
            pink         = get_color565(255, 130, 198),
            purple       = get_color565(123,   0, 123),
            red          = get_color565(255,   0,   0),
            white        = get_color565(255, 255, 255),
            yellow       = get_color565(255, 255,   0)
        };

        /*enum class Color565 : uint16_t
        {
            black        = 0x0000,  //   0,   0,   0
            blue         = 0x001F,  //   0,   0, 255
            cyan         = 0x07FF,  //   0, 255, 255
            dark_cyan    = 0x03EF,  //   0, 125, 123
            dark_green   = 0x03E0,  //   0, 125,   0
            dark_grey    = 0x7BEF,  // 123, 125, 123
            green        = 0x07E0,  //   0, 255,   0
            green_yellow = 0xAFE5,  // 173, 255,  41
            light_grey   = 0xC618,  // 198, 195, 198
            magenta      = 0xF81F,  // 255,   0, 255
            maroon       = 0x7800,  // 123,   0,   0
            navy         = 0x000F,  //   0,   0, 123
            olive        = 0x7BE0,  // 123, 125,   0
            orange       = 0xFD20,  // 255, 165,   0
            pink         = 0xFC18,  // 255, 130, 198
            purple       = 0x780F,  // 123,   0, 123
            red          = 0xF800,  // 255,   0,   0
            white        = 0xFFFF,  // 255, 255, 255
            yellow       = 0xFFE0   // 255, 255,   0
        };*/

		// --------------------------------------------------------------------
		// PUBLIC MEMBER FUNCTIONS
		// --------------------------------------------------------------------

        ParallelLcd(const Pin::Name   dc,
                    const Pin::Name   wr,
                    const Pin::Name   rd,
                    const Pin::Name   reset,
                    const uint16_t    width,
                    const uint16_t    height,
                    const Orientation orientation) : m_width { width },
                                                     m_height { height },
                                                     m_orientation { orientation },
                                                     m_dc_port_name { get_port_name(dc) },
                                                     m_dc_port_mask { get_port_mask(dc) },
                                                     m_wr_port_name { get_port_name(wr) },
                                                     m_wr_port_mask { get_port_mask(wr) },
                                                     m_rd_port_name { get_port_name(rd) },
                                                     m_rd_port_mask { get_port_mask(rd) },
                                                     m_reset(reset, { Gpio::OutputMode::push_pull_high })
        {
            constexpr PinNameBus data_name_bus({ Pin::Name::pd_0,    // DB8 ...
                                                 Pin::Name::pd_1,
                                                 Pin::Name::pd_2,
                                                 Pin::Name::pd_3,
                                                 Pin::Name::pd_4,
                                                 Pin::Name::pd_5,
                                                 Pin::Name::pd_6,
                                                 Pin::Name::pd_7 }); // ... DB15

            for(const auto pin_name : data_name_bus)
            {
                Gpio gpio_data(pin_name, { Gpio::OutputMode::push_pull_low });
            }

            Gpio gpio_dc(dc, { Gpio::OutputMode::push_pull_high });
            Gpio gpio_wr(wr, { Gpio::OutputMode::push_pull_high });
            Gpio gpio_rd(rd, { Gpio::OutputMode::push_pull_high });
        }

        // -------- CONFIGURATION ---------------------------------------------

        virtual void initialize(const Color565 background_color565) = 0;

        // Get the width of the display, accounting for current orientation
        uint16_t get_width() const { return m_width; };

        // Get the height of the display, accounting for current orientation
        uint16_t get_height() const { return m_height; }

        // Set the orientation setting for display
        virtual void set_orientation(const Orientation orientation) = 0;

        // Get the orientation setting for display
        Orientation get_orientation() const { return m_orientation; }

        // -------- BASIC DRAW ------------------------------------------------

        void draw_horizontal_line(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t lcd_x_end, const uint16_t color565)
        {
            const uint16_t width = lcd_x_end - lcd_x + 1;

            fill_rectangle(lcd_x, lcd_y, width, 1, color565);
        }
        void draw_horizontal_line(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t lcd_x_end, const Color565 color565)
        {
            draw_horizontal_line(lcd_x, lcd_y, lcd_x_end, static_cast<uint16_t>(color565));
        }

        void draw_vertical_line(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t lcd_y_end, const uint16_t color565)
        {
            const uint16_t height = lcd_y_end - lcd_y + 1;

            fill_rectangle(lcd_x, lcd_y, 1, height, color565);
        }
        void draw_vertical_line(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t lcd_y_end, const Color565 color565)
        {
            draw_vertical_line(lcd_x, lcd_y, lcd_y_end, static_cast<uint16_t>(color565));
        }

        void fill_rectangle(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t width, const uint16_t height, const uint16_t color565)
        {
            if((lcd_x + width - 1) < m_width && (lcd_y + height - 1) < m_height)
            {
                set_address_window(lcd_x, lcd_y, width, height);
                write_16bits(color565, width * height);
            }
        }
        void fill_rectangle(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t width, const uint16_t height, const Color565 color565)
        {
            fill_rectangle(lcd_x, lcd_y, width, height, static_cast<uint16_t>(color565));
        }

        void fill_screen(const uint16_t color565)
        {
            fill_rectangle(0, 0, m_width, m_height, color565);
        }
        void fill_screen(const Color565 color565)
        {
            fill_screen(static_cast<uint16_t>(color565));
        }

        // -------- IMAGE DRAW ------------------------------------------------

        // Draw a 16-bit image ('565' RGB format)
        // How to use: https://forum.pjrc.com/threads/35575-Export-for-ILI9341_t3-with-GIMP
        void draw_image(const uint16_t lcd_x, const uint16_t lcd_y, const uint16_t width, const uint16_t height, const uint16_t *pixel_data)
        {
            if((lcd_x + width - 1) < m_width && (lcd_y + height - 1) < m_height)
            {
                set_address_window(lcd_x, lcd_y, width, height);

                uint32_t pixel_count = width * height;

                while(pixel_count-- > 0)
                {
                    write_16bits(*pixel_data++);
                }
            }
        }

        // -------- PRINT -----------------------------------------------------

        void print(uint16_t lcd_x, const uint16_t lcd_y, Font& font, const uint16_t font_color565, const uint16_t background_color565, const char *str)
        {
            const uint8_t char_width  = font.get_char_width();
            const uint8_t char_height = font.get_char_height();

            const char *str_ptr = str;

            uint16_t str_width = 0;

            while(*str_ptr++ != '\0')
            {
                str_width += char_width;
            }

            if((lcd_x + str_width - 1) < m_width && (lcd_y + char_height - 1) < m_height)
            {
                str_ptr = str;

                while(*str_ptr != '\0')
                {
                    const uint8_t* char_data = font.get_data(*str_ptr++);

                    uint8_t char_size = font.get_char_size();

                    set_address_window(lcd_x, lcd_y, char_width, char_height);

                    while(char_size-- > 0)
                    {
                        for(uint8_t bit = 0; bit < 8; ++bit)
                        {
                            const uint8_t char_pixel = *char_data & (1 << bit);

                            write_16bits((char_pixel == 0) ? background_color565 : font_color565);
                        }

                        char_data++;
                    }

                    lcd_x += char_width;
                }
            }
        }

        void print(const uint16_t lcd_x, const uint16_t lcd_y, Font& font, const Color565 font_color565, const Color565 background_color565, const char *str)
        {
            print(lcd_x, lcd_y, font, static_cast<uint16_t>(font_color565), static_cast<uint16_t>(background_color565), str);
        }

	protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        using Pin      = hal::Pin;
        using Gpio     = hal::Gpio;
        using Port     = hal::Port;
        using UsTicker = hal::UsTicker;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        bool is_hardware_reset() const
        {
            return m_reset.get_pin_name() != Pin::Name::nc;
        }

        void do_hardware_reset()
        {
            // Reset pulse duration >= 10 us
            m_reset = 0;
            UsTicker::wait(std::chrono::microseconds(10));

            // Sleep Out command cannot be sent for 120 ms after releasing reset
            m_reset = 1;
            UsTicker::wait(std::chrono::milliseconds(120));
        }

        // Set up the specific display hardware's "address window" for
        // subsequent pixel-pushing operations
        /*
         * x1:     leftmost pixel of area to be drawn (MUST be within display
         *         bounds at current orientation setting);
         * y1:     topmost pixel of area to be drawn (MUST be within display
         *         bounds at current orientation setting);
         * width:  width of area to be drawn, in pixels (MUST be > 0 and, added
         *         to x, within display bounds at current orientation);
         * height: height of area to be drawn, in pixels (MUST be > 0 and,
                   added to x, within display bounds at current orientation).
        */
        virtual void set_address_window(const uint16_t x1, const uint16_t y1, const uint16_t width, const uint16_t height) = 0;

        // Write a single command byte to the display
        // NOTE: this sets the device to COMMAND mode, issues the byte and then
        //       restores DATA mode.
        void write_command(const uint8_t cmd)
        {
            clear_dc();
            write_8bits(cmd);
            set_dc();
        }

        // Write a single 8-bit value to the display
        void write_8bits(const uint8_t value)
        {
            Port::write(m_data_port_name, value);
            strobe_wr();
        }

        // Write a single 16-bit value to the display
        void write_16bits(const uint16_t value)
        {
            const uint32_t value_msb = value >> 8;
            const uint32_t value_lsb = value & 0xFF;

            Port::write(m_data_port_name, value_msb);
            strobe_wr();
            Port::write(m_data_port_name, value_lsb);
            strobe_wr();
        }

        // --------------------------------------------------------------------
        // PROTECTED MEMBER VARIABLES
        // --------------------------------------------------------------------

        uint16_t    m_width;        // Display width as modified by current orientation
        uint16_t    m_height;       // Display height as modified by current orientation
        Orientation m_orientation;  // Display orientation

	private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void clear_dc() { Port::clear(m_dc_port_name, m_dc_port_mask); }
        void set_dc()   { Port::set  (m_dc_port_name, m_dc_port_mask); }

        void strobe_wr()
        {
            Port::clear(m_wr_port_name, m_wr_port_mask);
            Port::set  (m_wr_port_name, m_wr_port_mask);
        }

        void clear_rd() { Port::clear(m_rd_port_name, m_rd_port_mask); }
        void set_rd()   { Port::set  (m_rd_port_name, m_rd_port_mask); }

        // Write a series of same 16-bit value to the display
        void write_16bits(const uint16_t value, uint32_t count)
        {
            const uint32_t value_msb = value >> 8;
            const uint32_t value_lsb = value & 0xFF;

            while(count-- > 0)
            {
                Port::write(m_data_port_name, value_msb);
                strobe_wr();
                Port::write(m_data_port_name, value_lsb);
                strobe_wr();
            }
        }

        static constexpr Port::Name get_port_name(const Pin::Name pin_name)
        {
            assert(pin_name != Pin::Name::nc);
            //                                                   pin_name  / 32
            return static_cast<Port::Name>(static_cast<uint32_t>(pin_name) >> 5);
        }

        static constexpr uint32_t get_port_mask(const Pin::Name pin_name)
        {
            assert(pin_name != Pin::Name::nc);
            //                                 pin_name  % 32
            return 1 << (static_cast<uint32_t>(pin_name) & 0x1F);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        static constexpr Port::Name m_data_port_name { Port::Name::portd }; // 8-bit data port name
        //static constexpr uint32_t   m_data_port_mask { 0xFF };              // 8-bit data port mask
        const Port::Name            m_dc_port_name;                         // Data (1) / Command (0) port name
        const uint32_t              m_dc_port_mask;                         // Data (1) / Command (0) port mask
        const Port::Name            m_wr_port_name;                         // Data write port name
        const uint32_t              m_wr_port_mask;                         // Data write port mask
        const Port::Name            m_rd_port_name;                         // Data read port name
        const uint32_t              m_rd_port_mask;                         // Data read port mask
        DigitalOut                  m_reset;                                // Display reset
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_PARALLEL_LCD_HPP
