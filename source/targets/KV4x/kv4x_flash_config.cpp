// ----------------------------------------------------------------------------
// @file    kv4x_flash_config.cpp
// @brief   Flash Configuration block: 16-byte flash configuration field that
//          stores default protection settings (loaded on reset) and security
//          information that allows the MCU to restrict access to the Flash
//          Memory module.
//          Placed at address 0x400 by the linker script.
// @notes   Flash is not secure, flash backdoor is enabled/disabled are the
//          only options implemented for now.
//          All program flash region is not protected, otherwise the application
//          will not be able to erase any of the protected flash regions, even
//          the configuration field!
//          For more details:
//          https://www.nxp.com/docs/en/application-note/AN4507.pdf
// @date    17 June 2019
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

#include "core/target_specs.hpp"

#ifdef __KV4X__

#include "xarmlib_config.hpp"

extern "C"
{




// Unsecured state: all flash commands are available to the programming
// interfaces (JTAG), as well as user code execution of Flash Controller
// commands.
#define FLASH_CONFIG_FLASH_NOT_SECURE           0xFFFFFFFE

// Secured without backdoor key state: programmer interfaces are only allowed
// to launch mass erase operations and have no access to memory locations.
#define FLASH_CONFIG_FLASH_BACKDOOR_DISABLED    0xFFFFFFFF

// Secured with backdoor key state: the backdoor key option allows for a means
// of temporarily disabling flash security if the correct 64-bit key value is
// provided when executing a flash verify backdoor access key command.
#define FLASH_CONFIG_FLASH_BACKDOOR_ENABLED     0xFFFFFFBF


#if (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_NOT_SECURE)
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_NOT_SECURE
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#elif (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_BACKDOOR_DISABLED)
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_BACKDOOR_DISABLED
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#elif (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_BACKDOOR_ENABLED)
#ifdef XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY
#if ((XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY != 0ULL) \
  && (XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY != 0xFFFFFFFFFFFFFFFFULL))
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_BACKDOOR_ENABLED
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       (uint64_t)XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY
#else
#   error "Backdoor Key invalid!"
#endif
#else
#   error "Backdoor Key not defined!"
#endif
#endif

#ifndef CURRENT_FLASH_CONFIG_SETTING
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_NOT_SECURE
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#endif




// ----------------------------------------------------------------------------
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
// NOTE: the flash was originally designed for big-endian architectures, but
//       Kinetis is little-endian.
// ----------------------------------------------------------------------------
__attribute__ ((used, section(".FlashConfig")))
const struct
{
    uint8_t  key_byte3;
    uint8_t  key_byte2;
    uint8_t  key_byte1;
    uint8_t  key_byte0;
    uint8_t  key_byte7;
    uint8_t  key_byte6;
    uint8_t  key_byte5;
    uint8_t  key_byte4;
    uint32_t fprot;
    uint32_t reserved_fopt_fsec;
} FlashConfig = { (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 24),   // Swap LSB Backdoor comparison key
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 16),
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 8),
                  (uint8_t) CURRENT_FLASH_CONFIG_BACKDOOR_KEY,
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 56),   // Swap MSB Backdoor comparison key
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 48),
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 40),
                  (uint8_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 32),
                  0xFFFFFFFF,                       // All program flash region is not protected (FPROT[0-3])
                  CURRENT_FLASH_CONFIG_SETTING };   // 0xFF (reserved) | 0xFF (reserved) | 0xFF (FOPT) | FSEC -> mass erase are always enabled
                                                    //                                                        -> factory security level access are always granted




} // extern "C"

#endif // __KV4X__
