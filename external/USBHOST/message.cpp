// ----------------------------------------------------------------------------
// @file    message.cpp
// @brief   message functions.
// @notes   USB_Host_Shield_2.0 message.cpp file suitable for Xarmlib
// @date    8 June 2020
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

#include "Usb.h"
// 0x80 is the default (i.e. trace) to turn off set this global to something lower.
// this allows for 126 other debugging levels.
// TO-DO: Allow assignment to a different serial port by software
int UsbDEBUGlvl = 0x80;

void E_Notifyc(char c, int lvl)
{
        if(UsbDEBUGlvl < lvl) return;
//#if defined(ARDUINO) && ARDUINO >=100
//        USB_HOST_SERIAL.print(c);
//#else
//        USB_HOST_SERIAL.print(c, BYTE);
//#endif
//        //USB_HOST_SERIAL.flush();

#ifdef DEBUG_USB_HOST
        DBG("{}", c);
#endif
}

void E_Notify(char const * msg, int lvl) {
        if(UsbDEBUGlvl < lvl) return;
        if(!msg) return;
        char c;

        while((c = pgm_read_byte(msg++))) E_Notifyc(c, lvl);
}

void E_NotifyStr(char const * msg, int lvl) {
        if(UsbDEBUGlvl < lvl) return;
        if(!msg) return;
        char c;

        while((c = *msg++)) E_Notifyc(c, lvl);
}

void E_Notify(uint8_t b, int lvl) {
        if(UsbDEBUGlvl < lvl) return;
//#if defined(ARDUINO) && ARDUINO >=100
//        USB_HOST_SERIAL.print(b);
//#else
//        USB_HOST_SERIAL.print(b, DEC);
//#endif
//        //USB_HOST_SERIAL.flush();

#ifdef DEBUG_USB_HOST
        DBG("{}", b);
#endif
}

void E_Notify(double d, int lvl) {
        if(UsbDEBUGlvl < lvl) return;
//        USB_HOST_SERIAL.print(d);
//        //USB_HOST_SERIAL.flush();

#ifdef DEBUG_USB_HOST
        DBG("{}", d);
#endif
}

#ifdef DEBUG_USB_HOST

void NotifyFailGetDevDescr(void) {
        Notify(PSTR("\r\ngetDevDescr "), 0x80);
}

void NotifyFailSetDevTblEntry(void) {
        Notify(PSTR("\r\nsetDevTblEn "), 0x80);
}

void NotifyFailGetConfDescr(void) {
        Notify(PSTR("\r\ngetConf "), 0x80);
}

void NotifyFailSetConfDescr(void) {
        Notify(PSTR("\r\nsetConf "), 0x80);
}

void NotifyFailGetDevDescr(uint8_t reason) {
        NotifyFailGetDevDescr();
        NotifyFail(reason);
}

void NotifyFailSetDevTblEntry(uint8_t reason) {
        NotifyFailSetDevTblEntry();
        NotifyFail(reason);

}

void NotifyFailGetConfDescr(uint8_t reason) {
        NotifyFailGetConfDescr();
        NotifyFail(reason);
}

void NotifyFailSetConfDescr(uint8_t reason) {
        NotifyFailSetConfDescr();
        NotifyFail(reason);
}

void NotifyFailUnknownDevice(uint16_t VID, uint16_t PID) {
        Notify(PSTR("\r\nUnknown Device Connected - VID: "), 0x80);
        D_PrintHex<uint16_t > (VID, 0x80);
        Notify(PSTR(" PID: "), 0x80);
        D_PrintHex<uint16_t > (PID, 0x80);
}

void NotifyFail(uint8_t rcode) {
        D_PrintHex<uint8_t > (rcode, 0x80);
        Notify(PSTR("\r\n"), 0x80);
}
#endif
