// ----------------------------------------------------------------------------
// @file    UHS_macros.h
// @brief   UHS macros.
// @notes   UHS30 UHS_macros.h file suitable for Xarmlib
// @date    5 June 2020
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


#define delay(ms) xarmlib::hal::UsTicker::wait(std::chrono::milliseconds(ms))
#define delayMicroseconds(us) xarmlib::hal::UsTicker::wait(std::chrono::microseconds(us))
#define micros() std::chrono::duration_cast<std::chrono::microseconds>(xarmlib::hal::UsTicker::now()).count()
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(xarmlib::hal::UsTicker::now()).count()


////////////////////////////////////////////////////////////////////////////////
// HANDY MACROS
////////////////////////////////////////////////////////////////////////////////

#define UHS_SWAP_VALUES(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))
#ifndef __BYTE_GRABBING_DEFINED__
#define __BYTE_GRABBING_DEFINED__ 1
#ifdef BROKEN_OPTIMIZER_LITTLE_ENDIAN
// Note: Use this if your compiler generates horrible assembler!
#define UHS_UINT8_BYTE0(__usi__)  (((uint8_t *)&(__usi__))[0])
#define UHS_UINT8_BYTE1(__usi__)  (((uint8_t *)&(__usi__))[1])
#define UHS_UINT8_BYTE2(__usi__)  (((uint8_t *)&(__usi__))[2])
#define UHS_UINT8_BYTE3(__usi__)  (((uint8_t *)&(__usi__))[3])
#define UHS_UINT8_BYTE4(__usi__)  (((uint8_t *)&(__usi__))[4])
#define UHS_UINT8_BYTE5(__usi__)  (((uint8_t *)&(__usi__))[5])
#define UHS_UINT8_BYTE6(__usi__)  (((uint8_t *)&(__usi__))[6])
#define UHS_UINT8_BYTE7(__usi__)  (((uint8_t *)&(__usi__))[7])
#else
// Note: The cast alone to uint8_t is actually enough.
// GCC throws out the "& 0xff", and the size is no different.
// Some compilers need it.
#define UHS_UINT8_BYTE0(__usi__)  ((uint8_t)((__usi__) & 0xff ))
#define UHS_UINT8_BYTE1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xff))
#define UHS_UINT8_BYTE2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xff))
#define UHS_UINT8_BYTE3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xff))
#define UHS_UINT8_BYTE4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xff))
#define UHS_UINT8_BYTE5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xff))
#define UHS_UINT8_BYTE6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xff))
#define UHS_UINT8_BYTE7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xff))
#endif
#define UHS_UINT16_SET_BYTE1(__usi__)  ((uint16_t)(__usi__) << 8)
#define UHS_UINT32_SET_BYTE1(__usi__)  ((uint32_t)(__usi__) << 8)
#define UHS_UINT64_SET_BYTE1(__usi__)  ((uint64_t)(__usi__) << 8)
#define UHS_UINT32_SET_BYTE2(__usi__)  ((uint32_t)(__usi__) << 16)
#define UHS_UINT64_SET_BYTE2(__usi__)  ((uint64_t)(__usi__) << 16)
#define UHS_UINT32_SET_BYTE3(__usi__)  ((uint32_t)(__usi__) << 24)
#define UHS_UINT64_SET_BYTE3(__usi__)  ((uint64_t)(__usi__) << 24)
#define UHS_UINT64_SET_BYTE4(__usi__)  ((uint64_t)(__usi__) << 32)
#define UHS_UINT64_SET_BYTE5(__usi__)  ((uint64_t)(__usi__) << 40)
#define UHS_UINT64_SET_BYTE6(__usi__)  ((uint64_t)(__usi__) << 48)
#define UHS_UINT64_SET_BYTE7(__usi__)  ((uint64_t)(__usi__) << 56)

// These are the smallest and fastest ways I have found so far in pure C/C++.
#define UHS_BYTES_TO_UINT16(__usc1__,__usc0__) ((uint16_t)((uint16_t)(__usc0__) | (uint16_t)UHS_UINT16_SET_BYTE1(__usc1__)))
#define UHS_BYTES_TO_UINT32(__usc3__,__usc2__,__usc1__,__usc0__) ((uint32_t)((uint32_t)(__usc0__) | UHS_UINT32_SET_BYTE1(__usc1__) | UHS_UINT32_SET_BYTE2(__usc2__) | UHS_UINT32_SET_BYTE3(__usc3__)))
#define UHS_BYTES_TO_UINT64(__usc7__,__usc6__,__usc5__,__usc4__,__usc3__,__usc2__,__usc1__,__usc0__) ((uint64_t)((uint64_t)__usc0__ | UHS_UINT64_SET_BYTE1(__usc1__) | UHS_UINT64_SET_BYTE2(__usc2__) | UHS_UINT64_SET_BYTE3(__usc3__) | UHS_UINT64_SET_BYTE4(__usc4__) | UHS_UINT64_SET_BYTE5(__usc5__) | UHS_UINT64_SET_BYTE6(__usc6__) | UHS_UINT64_SET_BYTE7(__usc7__)))
#endif


#define VOID0 ((void)0)

#if !defined(NOTUSED)
#define NOTUSED(...)  __VA_ARGS__ __attribute__((unused))
#endif

#define UHS_SLEEP_MS(v) pUsb->sof_delay(v)

#ifndef UHS_NI
#define UHS_NI __attribute__((noinline))
#endif

#endif /* MACROS_H */

