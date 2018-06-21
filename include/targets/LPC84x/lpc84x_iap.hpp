// ----------------------------------------------------------------------------
// @file    lpc84x_iap.hpp
// @brief   NXP LPC84x In-Application Programming (IAP) class.
// @date    19 June 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_IAP_HPP
#define __XARMLIB_TARGETS_LPC84X_IAP_HPP

#include "system/gsl"
#include "targets/LPC84x/lpc84x_cmsis.h"
#include "targets/LPC84x/lpc84x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class Iap
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // IAP status codes
        enum class StatusCode
        {
            CMD_SUCCESS = 0,                            // Command is executed successfully
            INVALID_COMMAND,                            // Invalid command
            SRC_ADDR_ERROR,                             // Source address not on word boundary
            DST_ADDR_ERROR,                             // Destination address is not on a correct boundary
            SRC_ADDR_NOT_MAPPED,                        // Source address is not mapped in the memory map
            DST_ADDR_NOT_MAPPED,                        // Destination address is not mapped in the memory map
            COUNT_ERROR,                                // Byte count is not multiple of 4 or is not a permitted value
            INVALID_SECTOR_INVALID_PAGE,                // Sector/page number is invalid or end sector number is greater than start sector number
            SECTOR_NOT_BLANK,                           // Sector is not blank
            SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,    // Command to prepare sector for write operation was not executed
            COMPARE_ERROR,                              // Source and destination data not equal
            BUSY,                                       // Flash programming hardware interface is busy
            PARAM_ERROR,                                // Insufficient number of parameters or invalid parameter
            ADDR_ERROR,                                 // Address is not on word boundary
            ADDR_NOT_MAPPED,                            // Address is not mapped in the memory map
            CMD_LOCKED,                                 // Command is locked
            INVALID_CODE,                               // Unlock code is invalid
            INVALID_BAUD_RATE,                          // Invalid baud rate setting
            INVALID_STOP_BIT,                           // Invalid stop bit setting
            CODE_READ_PROTECTION_ENABLED,               // Code read protection enabled
            INVALID_FLASH_UNIT,                         // Reserved
            USER_CODE_CHECKSUM,                         // User code checksum is invalid
            SETTING_ACTIVE_PARTITION,                   // Reserved
            FRO_NO_POWER,                               // FRO not turned on in the PDRUNCFG register
            FLASH_NO_POWER,                             // Flash not turned on in the PDRUNCFG register
            EEPROM_NO_POWER,                            // Reserved
            EEPROM_NO_CLOCK,                            // Reserved
            FLASH_NO_CLOCK,                             // Flash clock disabled in the AHBCLKCTRL register
            REINVOKE_ISP_CONFIG,                        // Reinvoke ISP not successful
            NO_VALID_IMAGE,                             // Invalid image
            FAIM_NO_POWER,                              // FAIM not turned on in the PDRUNCFG register
            FAIM_NO_CLOCK                               // FAIM clock disabled in the AHBCLKCTRL register
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a flash page
        // @flash_page: Destination flash page where data is to be written.
        // @ buffer:    Buffer span containing the data to be written.
        // NOTE:        Buffer size must be 64 bytes (page size).
        static bool write_flash_page(const int32_t flash_page, const gsl::span<const uint8_t> buffer)
        {
            if(buffer.size() != m_page_size)
            {
                return false;
            }

            if(flash_page < 0 || flash_page >= m_page_count)
            {
                return false;
            }

            // Get flash page address
            const int32_t flash_address = flash_page * m_page_size;
            // Get the sector of the flash page
            const uint32_t flash_sector = get_sector(flash_address);

            // Prepare sector for erasing
            if(prepare_sectors(flash_sector, flash_sector) != StatusCode::CMD_SUCCESS)
            {
                return false;
            }

            // Erase page
            if(erase_pages(flash_page, flash_page) != StatusCode::CMD_SUCCESS)
            {
                return false;
            }

            // Prepare sector for writing
            if(prepare_sectors(flash_sector, flash_sector) != StatusCode::CMD_SUCCESS)
            {
                return false;
            }

            // Write to flash
            return (copy_ram_to_flash(flash_address, buffer) == StatusCode::CMD_SUCCESS);
        }

        // Write to flash
        // @flash_address: Destination flash address where data is to be written.
        //                 This address should be a 64 byte boundary.
        // @ buffer:       Buffer span containing the data to be written.
        // NOTE:           Buffer size must be multiple of 64 bytes (page size).
        static bool write_flash(const int32_t flash_address, const gsl::span<const uint8_t> buffer)
        {
            // Check for valid buffer size (multiple of page size)
            if(buffer.size() <= 0 || (buffer.size() % m_page_size) != 0)
            {
                return false;
            }

            // Check for valid address boundary
            if(flash_address < 0 || (flash_address % m_page_size) != 0 ||
              (flash_address + buffer.size()) > (m_page_count * m_page_size))
            {
                return false;
            }

            // Get the page of the flash address
            int32_t flash_page = get_page(flash_address);

            for(int32_t p = 0; p < buffer.size(); p += m_page_size)
            {
                // Write buffer one page at a time -> {0,63}, {64,127}, ...
                if(write_flash_page(flash_page++, buffer.subspan(p, m_page_size)) == false)
                {
                    return false;
                }
            }

            return true;
        }

        // Read a FAIM word (the manual calls it FAIIM page)
        static bool read_faim_word(const int32_t faim_word, const uint32_t& faim_value)
        {
            if(faim_word < 0 || faim_word > 7)
            {
                return false;
            }

            uint32_t result[5];
            uint32_t command[5] {      static_cast<uint32_t>(CommandCode::READ_FAIM_WORD),
                                       static_cast<uint32_t>(faim_word),
                                  reinterpret_cast<uint32_t>(&faim_value)
                                };

            iap_entry(command, result);

            return (static_cast<StatusCode>(result[0]) == StatusCode::CMD_SUCCESS);
        }

        // Write a FAIM word (the manual calls it FAIIM page)
        // NOTE: This function is followed by the 'read_faim_word()' to update
        //       the output of the FAIM. For the pull-up, pull-down, and HI-Z
        //       IOCON pin configuration settings, a reset is needed to transfer
        //       the newly programmed FAIM values into the IOCON pin configuration
        //       registers. Similarly, for SWD disable, Low Power Start configuration,
        //       ISP interface and pin select, a reset is needed to update the
        //       actual startup configuration.
        static bool write_faim_word(const int32_t faim_word, const uint32_t faim_value)
        {
            if(faim_word < 0 || faim_word > 7)
            {
                return false;
            }

            uint32_t result[5];
            uint32_t command[5] {      static_cast<uint32_t>(CommandCode::WRITE_FAIM_WORD),
                                       static_cast<uint32_t>(faim_word),
                                  reinterpret_cast<uint32_t>(&faim_value)
                                };

            iap_entry(command, result);

            if(static_cast<StatusCode>(result[0]) != StatusCode::CMD_SUCCESS)
            {
                return false;
            }

            // After a Write FAIM, a Read FAIM is required to update the output of the FAIM
            uint32_t faim_read_value = 0;

            if(read_faim_word(faim_word, faim_read_value) == false)
            {
                return false;
            }

            return (faim_value == faim_read_value);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Get the sector of the specified address
        static constexpr int32_t get_sector(const int32_t flash_address)
        {
            return (flash_address >> 10);       // 1kB sector (0x400 each)
        }

        // Get the page of the specified address
        static constexpr int32_t get_page(const int32_t flash_address)
        {
            return (flash_address >> 6);        // 64B page (0x40 each)
        }

        // Prepare flash sector(s) for erase / writing
        static StatusCode prepare_sectors(const int32_t sector_start, const int32_t sector_end)
        {
            uint32_t result[5];
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::PREPARE_SECTOR),
                                  static_cast<uint32_t>(sector_start),
                                  static_cast<uint32_t>(sector_end)
                                };

            iap_entry(command, result);

            return static_cast<StatusCode>(result[0]);
        }

        // Erase flash sector(s)
        static StatusCode erase_sectors(const int32_t sector_start, const int32_t sector_end)
        {
            uint32_t result[5];
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::ERASE_SECTOR),
                                  static_cast<uint32_t>(sector_start),
                                  static_cast<uint32_t>(sector_end),
                                  SystemCoreClock / 1000    // CPU Clock Frequency in kHz
                                };

            iap_entry(command, result);

            return static_cast<StatusCode>(result[0]);
        }

        // Erase flash page(s)
        static StatusCode erase_pages(const int32_t page_start, const int32_t page_end)
        {
            uint32_t result[5];
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::ERASE_PAGE),
                                  static_cast<uint32_t>(page_start),
                                  static_cast<uint32_t>(page_end),
                                  SystemCoreClock / 1000    // CPU Clock Frequency in kHz
                                };

            iap_entry(command, result);

            return static_cast<StatusCode>(result[0]);
        }

        // Copy RAM contents into flash
        // NOTE: Buffer size must be 64 | 128 | 256 | 512 | 1024 bytes.
        static StatusCode copy_ram_to_flash(const int32_t flash_address, const gsl::span<const uint8_t> buffer)
        {
            uint32_t result[5];
            uint32_t command[5] {      static_cast<uint32_t>(CommandCode::COPY_RAM_TO_FLASH),
                                       static_cast<uint32_t>(flash_address),
                                  reinterpret_cast<uint32_t>(&buffer[0]),
                                       static_cast<uint32_t>(buffer.size()),
                                  SystemCoreClock / 1000    // CPU Clock Frequency in kHz
                                };

            iap_entry(command, result);

            return static_cast<StatusCode>(result[0]);
        }

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Number of flash pages
        // Helder Parracho @ 23 March 2018
        // NOTE: The manual and datasheet states that all MCUs have 64Kb of flash
        static constexpr int32_t m_page_count = 1024; //1024 pages * 64 bytes
        // Flash page size is 64 bytes
        static constexpr int32_t m_page_size = 64;

        // Command codes for IAP
        enum class CommandCode
        {
            PREPARE_SECTOR     = 50,
            COPY_RAM_TO_FLASH  = 51,
            ERASE_SECTOR       = 52,
            BLANK_CHECK_SECTOR = 53,
            READ_PART_ID       = 54,
            READ_BOOT_CODE_VER = 55,
            COMPARE            = 56,
            REINVOKE_ISP       = 57,
            READ_UID           = 58,
            ERASE_PAGE         = 59,
            READ_SIGNATURE     = 73,
            READ_FAIM_WORD     = 80,
            WRITE_FAIM_WORD    = 81
        };
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_IAP_HPP
