// ----------------------------------------------------------------------------
// @file    kv4x_flash_iap.hpp
// @brief   Kinetis KV4x flash In-Application Programming (IAP) class.
// @date    13 June 2019
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

#ifndef __XARMLIB_TARGETS_KV4X_FLASH_IAP_HPP
#define __XARMLIB_TARGETS_KV4X_FLASH_IAP_HPP

#include "external/span.hpp"
#include "fsl_flash.h"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class FlashIapDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write to flash
        // @flash_address: Destination flash address where data is to be written.
        //                 This address should be a 16 bytes (Double-Phrase)
        //                 boundary if it is the start of a sector, otherwise
        //                 could be a 4 bytes (Longword) boundary.
        // @ buffer:       Buffer span containing the data to be written.
        //                 The buffer size must be multiple of 16 bytes (Double-Phrase)
        //                 if it is the start of a sector, otherwise could be
        //                 multiple of 4 bytes (Longword).
        // NOTES:          - sector(s) will only be erased if flash_address is the start of a sector;
        //                 - only allowed in Normal Run mode (System::clock::*_run).
        static bool write_flash(const int32_t flash_address, const std::span<const uint8_t> buffer)
        {
            // Check to make sure in RUN mode
            assert((SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK) == 1);

            if(flash_address < 0 || buffer.size() <= 0)
            {
                return false;
            }

            // NOTE: address boundary and buffer size will be checked by FSL drivers

            if(m_initialized == false)
            {
                // Setup flash driver structure for device and initialize variables
                status_t result = FLASH_Init(&m_flash_config);
                if(result != kStatus_FTFx_Success)
                {
                    return false;
                }

                // Setup flash cache driver structure for device and initialize variables
                result = FTFx_CACHE_Init(&m_cache_config);
                if(result != kStatus_FTFx_Success)
                {
                    return false;
                }

                m_initialized = true;
            }

            // Pre-preparation work about flash Cache/Prefetch/Speculation
            FTFx_CACHE_ClearCachePrefetchSpeculation(&m_cache_config, true);

            // Get the sector of the flash address
            const int32_t sector = get_sector_of_address(flash_address);

            if(get_address_of_sector_start(sector) == flash_address)
            {
                // The specified flash address is the start of a sector

                // Erase the sector(s) from flash_address
                status_t result = FLASH_Erase(&m_flash_config, flash_address, buffer.size(), kFTFx_ApiEraseKey);
                if(result != kStatus_FTFx_Success)
                {
                    return false;
                }

                // Verify sector(s) if it's been erased with user margin levels
                // NOTE: 'User' margin levels can be employed to check that flash memory contents have adequate
                //       margin for normal level read operations. If unexpected read results are encountered
                //       when checking flash memory contents at the 'user' margin levels, loss of information
                //       might soon occur during 'normal' readout.
                result = FLASH_VerifyErase(&m_flash_config, flash_address, buffer.size(), kFTFx_MarginValueUser);
                if(result != kStatus_FTFx_Success)
                {
                    return false;
                }
            }

            // Program user buffer into flash
            status_t result = FLASH_Program(&m_flash_config, flash_address, const_cast<uint8_t*>(buffer.data()), buffer.size());
            if(result != kStatus_FTFx_Success)
            {
                return false;
            }

            uint32_t failed_address = 0;
            uint32_t failed_data = 0;

            // Verify programming by program check command with user margin levels
            // NOTE: 'User' margin levels can be employed to check that flash memory contents have adequate
            //       margin for normal level read operations. If unexpected read results are encountered
            //       when checking flash memory contents at the 'user' margin levels, loss of information
            //       might soon occur during 'normal' readout.
            result = FLASH_VerifyProgram(&m_flash_config, flash_address, buffer.size(), buffer.data(), kFTFx_MarginValueUser, &failed_address, &failed_data);
            if(result != kStatus_FTFx_Success)
            {
                return false;
            }

            // Post-preparation work about flash Cache/Prefetch/Speculation
            FTFx_CACHE_ClearCachePrefetchSpeculation(&m_cache_config, false);

            return true;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Status codes
        enum class StatusCode
        {
            success                           = kStatus_FTFx_Success,                       // API is executed successfully
            invalid_argument                  = kStatus_FTFx_InvalidArgument,               // Invalid argument
            size_error                        = kStatus_FTFx_SizeError,                     // Error size
            alignment_error                   = kStatus_FTFx_AlignmentError,                // Parameter is not aligned with the specified baseline
            address_error                     = kStatus_FTFx_AddressError,                  // Address is out of range
            access_error                      = kStatus_FTFx_AccessError,                   // Invalid instruction codes and out-of bound addresses
            protection_violation              = kStatus_FTFx_ProtectionViolation,           // The program/erase operation is requested to execute on protected areas
            command_failure                   = kStatus_FTFx_CommandFailure,                // Run-time error during command execution
            unknown_property                  = kStatus_FTFx_UnknownProperty,               // Unknown property
            erase_key_error                   = kStatus_FTFx_EraseKeyError,                 // API erase key is invalid
            region_execute_only               = kStatus_FTFx_RegionExecuteOnly,             // The current region is execute-only
            execute_in_ram_function_not_ready = kStatus_FTFx_ExecuteInRamFunctionNotReady,  // Execute-in-RAM function is not available
            partition_status_update_failure   = kStatus_FTFx_PartitionStatusUpdateFailure,  // Failed to update partition status
            set_flexram_as_eeprom_error       = kStatus_FTFx_SetFlexramAsEepromError,       // Failed to set FlexRAM as EEPROM
            recover_flexram_as_ram_error      = kStatus_FTFx_RecoverFlexramAsRamError,      // Failed to recover FlexRAM as RAM
            set_flexram_as_ram_error          = kStatus_FTFx_SetFlexramAsRamError,          // Failed to set FlexRAM as RAM
            recover_flexram_as_eeprom_error   = kStatus_FTFx_RecoverFlexramAsEepromError,   // Failed to recover FlexRAM as EEPROM
            command_not_supported             = kStatus_FTFx_CommandNotSupported,           // Flash API is not supported
            swap_system_not_in_uninitialized  = kStatus_FTFx_SwapSystemNotInUninitialized,  // Swap system is not in an uninitialzed state
            swap_indicator_address_error      = kStatus_FTFx_SwapIndicatorAddressError,     // The swap indicator address is invalid
            read_only_property                = kStatus_FTFx_ReadOnlyProperty,              // The flash property is read-only
            invalid_property_value            = kStatus_FTFx_InvalidPropertyValue,          // The flash property value is out of range
            invalid_speculation_option        = kStatus_FTFx_InvalidSpeculationOption       // The option of flash prefetch speculation is invalid
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr int32_t get_sector_of_address(const int32_t flash_address)
        {
            return (flash_address >> 12);   // 4 kB sector
        }

        static constexpr int32_t get_address_of_sector_start(const int32_t sector)
        {
            return (sector * 0x1000);       // 4 kB sector
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        inline static bool                m_initialized { false };
        inline static flash_config_t      m_flash_config {};
        inline static ftfx_cache_config_t m_cache_config {};
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_FLASH_IAP_HPP
