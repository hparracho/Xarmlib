// ----------------------------------------------------------------------------
// @file    UHS_macros.h
// @brief   UHS macros.
// @notes   UHS30 UHS_macros.h file suitable for Xarmlib
// @date    5 May 2020
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

#if !defined(MACROS_H)
#define MACROS_H
#include "macro_logic.h"
#include "hal/hal_us_ticker.hpp"

#define delay(ms) xarmlib::hal::UsTicker::wait(std::chrono::milliseconds(ms))
#define micros() std::chrono::duration_cast<std::chrono::microseconds>(xarmlib::hal::UsTicker::now()).count()
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(xarmlib::hal::UsTicker::now()).count()

//#define Init_dyn_SWI() (void(0))
#if !defined(DDSB)
#define DDSB() (void(0))
#endif

// Just in case...
#ifndef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR xarmlib::hal::Uart
#endif

#ifndef INT16_MIN
#define INT16_MIN -32768
#endif

//// Nuke screwed up macro junk from the IDE.
//#ifdef __cplusplus
//#if defined(true)
//#undef true
//#endif
//#if defined(false)
//#undef false
//#endif
//#endif


//#if !defined(UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE)
//
//#if !defined(UHS_BIG_FLASH)
//
//#if defined(FLASHEND) && defined(FLASHSTART)
//#if (FLASHEND - FLASHSTART) > 0x0FFFFU
//#define UHS_BIG_FLASH 1
//#else
//#define UHS_BIG_FLASH 0
//#endif
//
//#elif defined(__PIC32_FLASH_SIZE)
//#if __PIC32_FLASH_SIZE > 511
//#define UHS_BIG_FLASH 1
//#else
//#define UHS_BIG_FLASH 0
//#endif
//
//#elif defined(FLASHEND) && !defined(FLASHSTART)
//// Assumes flash starts at 0x00000, is this a safe assumption?
//// 192K + should be OK
//#if FLASHEND > 0x02FFFFU
//#define UHS_BIG_FLASH 1
//#else
//#define UHS_BIG_FLASH 0
//#endif
//
//#elif defined(IFLASH_SIZE)
//#if IFLASH_SIZE > 0x0FFFFU
//#define UHS_BIG_FLASH 1
//#else
//#define UHS_BIG_FLASH 0
//#endif
//
//#elif defined(ESP8266)
//#define UHS_BIG_FLASH 1
//#define SYSTEM_OR_SPECIAL_YIELD(...) yield()
//
//#elif defined(__arm__) && defined(CORE_TEENSY)
//#define UHS_BIG_FLASH 1
//
//#elif defined(ARDUINO_spresense_ast)
//#define UHS_BIG_FLASH 1
//#else
//// safe default
//#warning Small flash?
//#define UHS_BIG_FLASH 0
//#endif
//#endif
//
//#if UHS_BIG_FLASH
//#define UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE 1
//#else
//#define UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE 0
//#endif
//#endif

//#if defined(__arm__) && defined(CORE_TEENSY)
//#define UHS_PIN_WRITE(p, v) digitalWriteFast(p, v)
//#define UHS_PIN_READ(p) digitalReadFast(p)
//#endif
//// TODO: Fast inline code for AVR and SAM based microcontrollers
////       This can be done pretty easily.
////       For now, this will just work out-of-the-box.
//#if !defined(UHS_PIN_WRITE)
//#define UHS_PIN_WRITE(p, v) digitalWrite(p, v)
//#endif
//#if !defined(UHS_PIN_READ)
//#define UHS_PIN_READ(p) digitalRead(p)
//#endif


#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()


//#if !defined(ARDUINO_SAMD_ZERO)
//#if defined(ARDUINO_AVR_ADK)
//#define UHS_GET_DPI(x) (x == 54 ? 6 : digitalPinToInterrupt(x))
//#else
//#define UHS_GET_DPI(x) digitalPinToInterrupt(x)
//#endif
//#else
//#define UHS_GET_DPI(x) (x)
//#endif

#ifndef __AVR__
#ifndef __PGMSPACE_H_
// This define should prevent reading the system pgmspace.h if included elsewhere
// This is not normally needed.
#define __PGMSPACE_H_ 1
#endif

#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef PGM_P
#define PGM_P  const char *
#endif
#ifndef PSTR
#define PSTR(str) (str)
#endif
#ifndef F
#define F(str) (str)
#endif
#ifndef _SFR_BYTE
#define _SFR_BYTE(n) (n)
#endif
#ifndef memchr_P
#define memchr_P(str, c, len) memchr((str), (c), (len))
#endif
#ifndef memcmp_P
#define memcmp_P(a, b, n) memcmp((a), (b), (n))
#endif
#ifndef memcpy_P
#define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#endif
#ifndef memmem_P
#define memmem_P(a, alen, b, blen) memmem((a), (alen), (b), (blen))
#endif
#ifndef memrchr_P
#define memrchr_P(str, val, len) memrchr((str), (val), (len))
#endif
#ifndef strcat_P
#define strcat_P(dest, src) strcat((dest), (src))
#endif
#ifndef strchr_P
#define strchr_P(str, c) strchr((str), (c))
#endif
#ifndef strchrnul_P
#define strchrnul_P(str, c) strchrnul((str), (c))
#endif
#ifndef strcmp_P
#define strcmp_P(a, b) strcmp((a), (b))
#endif
#ifndef strcpy_P
#define strcpy_P(dest, src) strcpy((dest), (src))
#endif
#ifndef strcasecmp_P
#define strcasecmp_P(a, b) strcasecmp((a), (b))
#endif
#ifndef strcasestr_P
#define strcasestr_P(a, b) strcasestr((a), (b))
#endif
#ifndef strlcat_P
#define strlcat_P(dest, src, len) strlcat((dest), (src), (len))
#endif
#ifndef strlcpy_P
#define strlcpy_P(dest, src, len) strlcpy((dest), (src), (len))
#endif
#ifndef strlen_P
#define strlen_P(s) strlen((const char *)(s))
#endif
#ifndef strnlen_P
#define strnlen_P(str, len) strnlen((str), (len))
#endif
#ifndef strncmp_P
#define strncmp_P(a, b, n) strncmp((a), (b), (n))
#endif
#ifndef strncasecmp_P
#define strncasecmp_P(a, b, n) strncasecmp((a), (b), (n))
#endif
#ifndef strncat_P
#define strncat_P(a, b, n) strncat((a), (b), (n))
#endif
#ifndef strncpy_P
#define strncpy_P(a, b, n) strncmp((a), (b), (n))
#endif
#ifndef strpbrk_P
#define strpbrk_P(str, chrs) strpbrk((str), (chrs))
#endif
#ifndef strrchr_P
#define strrchr_P(str, c) strrchr((str), (c))
#endif
#ifndef strsep_P
#define strsep_P(strp, delim) strsep((strp), (delim))
#endif
#ifndef strspn_P
#define strspn_P(str, chrs) strspn((str), (chrs))
#endif
#ifndef strstr_P
#define strstr_P(a, b) strstr((a), (b))
#endif
#ifndef sprintf_P
#define sprintf_P(s, ...) sprintf((s), __VA_ARGS__)
#endif
#ifndef vfprintf_P
#define vfprintf_P(s, ...) vfprintf((s), __VA_ARGS__)
#endif
#ifndef printf_P
#define printf_P(...) printf(__VA_ARGS__)
#endif
#ifndef snprintf_P
#define snprintf_P(s, n, ...) ((s), (n), __VA_ARGS__)
#endif
#ifndef vsprintf_P
#define vsprintf_P(s, ...) ((s),__VA_ARGS__)
#endif
#ifndef vsnprintf_P
#define vsnprintf_P(s, n, ...) ((s), (n),__VA_ARGS__)
#endif
#ifndef fprintf_P
#define fprintf_P(s, ...) ((s), __VA_ARGS__)
#endif

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif
#ifndef pgm_read_float
#define pgm_read_float(addr) (*(const float *)(addr))
#endif

#ifndef pgm_read_byte_near
#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#endif
#ifndef pgm_read_word_near
#define pgm_read_word_near(addr) pgm_read_word(addr)
#endif
#ifndef pgm_read_dword_near
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#endif
#ifndef pgm_read_float_near
#define pgm_read_float_near(addr) pgm_read_float(addr)
#endif
#ifndef pgm_read_byte_far
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#endif
#ifndef pgm_read_word_far
#define pgm_read_word_far(addr) pgm_read_word(addr)
#endif
#ifndef pgm_read_dword_far
#define pgm_read_dword_far(addr) pgm_read_dword(addr)
#endif
#ifndef pgm_read_float_far
#define pgm_read_float_far(addr) pgm_read_float(addr)
#endif

#ifndef pgm_read_pointer
#define pgm_read_pointer
#endif

#endif


////////////////////////////////////////////////////////////////////////////////
// HANDY MACROS
////////////////////////////////////////////////////////////////////////////////

// Atmoically set/clear single bits using bitbands.
// Believe it or not, this boils down to a constant,
// and is less code than using |= &= operators.
// Bonus, it makes code easier to read too.
// Bitbanding is a wonderful thing.
//#define BITNR(i) (i&0x1?0:i&0x2?1:i&0x4?2:i&0x8?3:i&0x10?4:i&0x20?5:i&0x40?6:i&0x80?7:i&0x100?8:i&0x200?9:i&0x400?10:i&0x800?11:i&0x1000?12:i&0x2000?13:i&0x4000?14:i&0x8000?15:i&0x10000?16:i&0x20000?17:i&0x40000?18:i&0x80000?19:i&0x100000?20:i&0x200000?21:i&0x400000?22:i&0x800000?23:i&0x1000000?24:i&0x2000000?25:i&0x4000000?26:i&0x8000000?27:i&0x10000000?28:i&0x20000000?29:i&0x40000000?30:i&0x80000000?31:32)
//#define UHS_KIO_BITBAND_ADDR(r, i) (((uint32_t)&(r) - 0x40000000) * 32 + (i) * 4 + 0x42000000)
//#define UHS_KIO_SETBIT_ATOMIC(r, m) (*(uint32_t *)UHS_KIO_BITBAND_ADDR((r), BITNR((m)))) = 1
//#define UHS_KIO_CLRBIT_ATOMIC(r, m) (*(uint32_t *)UHS_KIO_BITBAND_ADDR((r), BITNR((m)))) = 0
//
//
//#define VALUE_BETWEEN(v,l,h) (((v)>(l)) && ((v)<(h)))
//#define VALUE_WITHIN(v,l,h) (((v)>=(l)) && ((v)<=(h)))
//#define output_pgm_message(wa,fp,mp,el) wa = &mp, fp((char *)pgm_read_pointer(wa), el)
//#define output_if_between(v,l,h,wa,fp,mp,el) if(VALUE_BETWEEN(v,l,h)) output_pgm_message(wa,fp,mp[v-(l+1)],el);
//
//#define UHS_SWAP_VALUES(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))
//#ifndef __BYTE_GRABBING_DEFINED__
//#define __BYTE_GRABBING_DEFINED__ 1
//#ifdef BROKEN_OPTIMIZER_LITTLE_ENDIAN
//// Note: Use this if your compiler generates horrible assembler!
//#define UHS_UINT8_BYTE0(__usi__)  (((uint8_t *)&(__usi__))[0])
//#define UHS_UINT8_BYTE1(__usi__)  (((uint8_t *)&(__usi__))[1])
//#define UHS_UINT8_BYTE2(__usi__)  (((uint8_t *)&(__usi__))[2])
//#define UHS_UINT8_BYTE3(__usi__)  (((uint8_t *)&(__usi__))[3])
//#define UHS_UINT8_BYTE4(__usi__)  (((uint8_t *)&(__usi__))[4])
//#define UHS_UINT8_BYTE5(__usi__)  (((uint8_t *)&(__usi__))[5])
//#define UHS_UINT8_BYTE6(__usi__)  (((uint8_t *)&(__usi__))[6])
//#define UHS_UINT8_BYTE7(__usi__)  (((uint8_t *)&(__usi__))[7])
//#else
//// Note: The cast alone to uint8_t is actually enough.
//// GCC throws out the "& 0xff", and the size is no different.
//// Some compilers need it.
//#define UHS_UINT8_BYTE0(__usi__)  ((uint8_t)((__usi__) & 0xff ))
//#define UHS_UINT8_BYTE1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xff))
//#define UHS_UINT8_BYTE2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xff))
//#define UHS_UINT8_BYTE3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xff))
//#define UHS_UINT8_BYTE4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xff))
//#define UHS_UINT8_BYTE5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xff))
//#define UHS_UINT8_BYTE6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xff))
//#define UHS_UINT8_BYTE7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xff))
//#endif
//#define UHS_UINT16_SET_BYTE1(__usi__)  ((uint16_t)(__usi__) << 8)
//#define UHS_UINT32_SET_BYTE1(__usi__)  ((uint32_t)(__usi__) << 8)
//#define UHS_UINT64_SET_BYTE1(__usi__)  ((uint64_t)(__usi__) << 8)
//#define UHS_UINT32_SET_BYTE2(__usi__)  ((uint32_t)(__usi__) << 16)
//#define UHS_UINT64_SET_BYTE2(__usi__)  ((uint64_t)(__usi__) << 16)
//#define UHS_UINT32_SET_BYTE3(__usi__)  ((uint32_t)(__usi__) << 24)
//#define UHS_UINT64_SET_BYTE3(__usi__)  ((uint64_t)(__usi__) << 24)
//#define UHS_UINT64_SET_BYTE4(__usi__)  ((uint64_t)(__usi__) << 32)
//#define UHS_UINT64_SET_BYTE5(__usi__)  ((uint64_t)(__usi__) << 40)
//#define UHS_UINT64_SET_BYTE6(__usi__)  ((uint64_t)(__usi__) << 48)
//#define UHS_UINT64_SET_BYTE7(__usi__)  ((uint64_t)(__usi__) << 56)
//
//// These are the smallest and fastest ways I have found so far in pure C/C++.
//#define UHS_BYTES_TO_UINT16(__usc1__,__usc0__) ((uint16_t)((uint16_t)(__usc0__) | (uint16_t)UHS_UINT16_SET_BYTE1(__usc1__)))
//#define UHS_BYTES_TO_UINT32(__usc3__,__usc2__,__usc1__,__usc0__) ((uint32_t)((uint32_t)(__usc0__) | UHS_UINT32_SET_BYTE1(__usc1__) | UHS_UINT32_SET_BYTE2(__usc2__) | UHS_UINT32_SET_BYTE3(__usc3__)))
//#define UHS_BYTES_TO_UINT64(__usc7__,__usc6__,__usc5__,__usc4__,__usc3__,__usc2__,__usc1__,__usc0__) ((uint64_t)((uint64_t)__usc0__ | UHS_UINT64_SET_BYTE1(__usc1__) | UHS_UINT64_SET_BYTE2(__usc2__) | UHS_UINT64_SET_BYTE3(__usc3__) | UHS_UINT64_SET_BYTE4(__usc4__) | UHS_UINT64_SET_BYTE5(__usc5__) | UHS_UINT64_SET_BYTE6(__usc6__) | UHS_UINT64_SET_BYTE7(__usc7__)))
//#endif


#define VOID0 ((void)0)
#if !defined(NOTUSED)
#define NOTUSED(...)  __VA_ARGS__ __attribute__((unused))
#endif
#endif /* MACROS_H */

