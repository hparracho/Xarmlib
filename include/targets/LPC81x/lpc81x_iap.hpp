// ----------------------------------------------------------------------------
// @file    lpc81x_iap.hpp
// @brief   NXP LPC81x In-Application Programming (IAP) class.
// @date    9 April 2019
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

#ifndef __XARMLIB_TARGETS_LPC81X_IAP_HPP
#define __XARMLIB_TARGETS_LPC81X_IAP_HPP

#include "external/span.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class IapDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // IAP status codes
        enum class StatusCode
        {
            cmd_success = 0,                            // Command is executed successfully
            invalid_command,                            // Invalid command
            src_addr_error,                             // Source address not on word boundary
            dst_addr_error,                             // Destination address is not on a correct boundary
            src_addr_not_mapped,                        // Source address is not mapped in the memory map
            dst_addr_not_mapped,                        // Destination address is not mapped in the memory map
            count_error,                                // Byte count is not multiple of 4 or is not a permitted value
            invalid_sector_invalid_page,                // Sector/page number is invalid or end sector number is greater than start sector number
            sector_not_blank,                           // Sector is not blank
            sector_not_prepared_for_write_operation,    // Command to prepare sector for write operation was not executed
            compare_error,                              // Source and destination data not equal
            busy,                                       // Flash programming hardware interface is busy
            param_error,                                // Insufficient number of parameters or invalid parameter
            addr_error,                                 // Address is not on word boundary
            addr_not_mapped,                            // Address is not mapped in the memory map
            cmd_locked,                                 // Command is locked
            invalid_code,                               // Unlock code is invalid
            invalid_baud_rate,                          // Invalid baud rate setting
            invalid_stop_bit,                           // Invalid stop bit setting
            code_read_protection_enabled                // Code read protection enabled
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a flash page
        // @flash_page: Destination flash page where data is to be written.
        // @ buffer:    Buffer span containing the data to be written.
        // NOTE:        Buffer size must be 64 bytes (page size).
        static bool write_flash_page(const int32_t flash_page, const std::span<const uint8_t> buffer)
        {
            if(buffer.size() != TARGET_FLASH_PAGE_SIZE)
            {
                return false;
            }

            if(flash_page < 0 || flash_page >= TARGET_FLASH_PAGE_COUNT)
            {
                return false;
            }

            // Get flash page address
            const int32_t flash_address = flash_page * TARGET_FLASH_PAGE_SIZE;
            // Get the sector of the flash page
            const uint32_t flash_sector = get_sector(flash_address);

            // Prepare sector for erasing
            if(prepare_sectors(flash_sector, flash_sector) != StatusCode::cmd_success)
            {
                return false;
            }

            // Erase page
            if(erase_pages(flash_page, flash_page) != StatusCode::cmd_success)
            {
                return false;
            }

            // Prepare sector for writing
            if(prepare_sectors(flash_sector, flash_sector) != StatusCode::cmd_success)
            {
                return false;
            }

            // Write to flash
            return (copy_ram_to_flash(flash_address, buffer) == StatusCode::cmd_success);
        }

        // Write to flash
        // @flash_address: Destination flash address where data is to be written.
        //                 This address should be a 64 byte boundary.
        // @ buffer:       Buffer span containing the data to be written.
        // NOTE:           Buffer size must be multiple of 64 bytes (page size).
        static bool write_flash(const int32_t flash_address, const std::span<const uint8_t> buffer)
        {
            // Check for valid buffer size (multiple of page size)
            if(buffer.size() <= 0 || (buffer.size() % TARGET_FLASH_PAGE_SIZE) != 0)
            {
                return false;
            }

            // Check for valid address boundary
            if(flash_address < 0 || (flash_address % TARGET_FLASH_PAGE_SIZE) != 0 ||
              (flash_address + buffer.size()) > (TARGET_FLASH_SIZE))
            {
                return false;
            }

            // Get the page of the flash address
            int32_t flash_page = get_page(flash_address);

            for(int32_t p = 0; p < buffer.size(); p += TARGET_FLASH_PAGE_SIZE)
            {
                // Write buffer one page at a time -> {0,63}, {64,127}, ...
                if(write_flash_page(flash_page++, buffer.subspan(p, TARGET_FLASH_PAGE_SIZE)) == false)
                {
                    return false;
                }
            }

            return true;
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
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::prepare_sector),
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
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::erase_sector),
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
            uint32_t command[5] { static_cast<uint32_t>(CommandCode::erase_page),
                                  static_cast<uint32_t>(page_start),
                                  static_cast<uint32_t>(page_end),
                                  SystemCoreClock / 1000    // CPU Clock Frequency in kHz
                                };

            iap_entry(command, result);

            return static_cast<StatusCode>(result[0]);
        }

        // Copy RAM contents into flash
        // NOTE: Buffer size must be 64 | 128 | 256 | 512 | 1024 bytes.
        static StatusCode copy_ram_to_flash(const int32_t flash_address, const std::span<const uint8_t> buffer)
        {
            uint32_t result[5];
            uint32_t command[5] {      static_cast<uint32_t>(CommandCode::copy_ram_to_flash),
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

        // Command codes for IAP
        enum class CommandCode
        {
            prepare_sector     = 50,
            copy_ram_to_flash  = 51,
            erase_sector       = 52,
            blank_check_sector = 53,
            read_part_id       = 54,
            read_boot_code_ver = 55,
            compare            = 56,
            reinvoke_isp       = 57,
            read_uid           = 58,
            erase_page         = 59
        };
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_IAP_HPP
