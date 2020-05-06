// ----------------------------------------------------------------------------
// @file    spi_max3421e_INLINE.h
// @brief   SPI MAX3421E driver class implementation.
// @notes   Based on UHS30 USB_HOST_SHIELD_INLINE.h file suitable for Xarmlib
// @date    6 May 2020
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

#if defined(SPI_MAX3421E_H) && !defined(USB_HOST_SHIELD_LOADED)
#define USB_HOST_SHIELD_LOADED

//#if !defined(digitalPinToInterrupt)
//#error digitalPinToInterrupt not defined, complain to your board maintainer.
//#endif


// allow two slots. this makes the maximum allowed shield count TWO
// for AVRs this is limited to pins 2 and 3 ONLY
// for all other boards, one odd and one even pin number is allowed.
static MAX3421E_HOST *ISReven;
static MAX3421E_HOST *ISRodd;

static void UHS_NI call_ISReven(void) {
        ISReven->ISRTask();
}

static void UHS_NI call_ISRodd(void) {
        UHS_PIN_WRITE(LED_BUILTIN, HIGH);
        ISRodd->ISRTask();
}


// Write single byte into MAX3421e register
void UHS_NI MAX3421E_HOST::regWr(uint8_t reg, uint8_t data)
{
    pSpi->mutex_take();
    ss_pin = 0;

    pSpi->transfer(reg | 0x02);
    pSpi->transfer(data);

    ss_pin = 1;
    pSpi->mutex_give();
}


// Multiple-byte write
// returns a pointer to memory position after last written
uint8_t* UHS_NI MAX3421E_HOST::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t* data_p)
{
    pSpi->mutex_take();
    ss_pin = 0;

    pSpi->transfer(reg | 0x02);

    while(nbytes)
    {
        pSpi->transfer(*data_p);
        nbytes--;
        data_p++; // advance data pointer
    }

    ss_pin = 1;
    pSpi->mutex_give();

    return data_p;
}


// GPIO Write
// GPIO byte is split between 2 registers, so two writes are needed to write one byte
// GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2
void UHS_NI MAX3421E_HOST::gpioWr(uint8_t data)
{
    regWr(rIOPINS1, data);
    data >>= 4;
    regWr(rIOPINS2, data);
}


// Single host register read
uint8_t UHS_NI MAX3421E_HOST::regRd(uint8_t reg)
{
    pSpi->mutex_take();
    ss_pin = 0;

    pSpi->transfer(reg);
    uint8_t rv = pSpi->transfer(0); // send empty byte

    ss_pin = 1;
    pSpi->mutex_give();

    return rv;
}


// Multiple-byte register read
// returns a pointer to a memory position after last read
uint8_t* UHS_NI MAX3421E_HOST::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p)
{
    pSpi->mutex_take();
    ss_pin = 0;

    pSpi->transfer(reg);

    while(nbytes)
    {
        *data_p++ = pSpi->transfer(0);
        nbytes--;
    }

    ss_pin = 1;
    pSpi->mutex_give();

    return data_p;
}


// GPIO read
// (see gpioWr for explanation)
// GPIN pins are in high nibbles of IOPINS1, IOPINS2
uint8_t UHS_NI MAX3421E_HOST::gpioRd(void)
{
    uint8_t gpin = 0;

    gpin  = regRd(rIOPINS2);        // pins 4-7
    gpin &= 0xf0;                   // clean lower nibble
    gpin |= (regRd(rIOPINS1) >> 4); // shift low bits and OR with upper from previous operation.

    return gpin;
}


// Reset MAX3421E
// returns number of microseconds it took for PLL to stabilize after reset
// or zero if PLL haven't stabilized in 65535 cycles
uint16_t UHS_NI MAX3421E_HOST::reset(void)
{
    // Initiate chip reset
    regWr(rUSBCTL, bmCHIPRES);
    regWr(rUSBCTL, 0x00);

    uint32_t expires = micros() + 65535;

    // Enable full-duplex SPI so we can read rUSBIRQ
    regWr(rPINCTL, bmFDUPSPI);

    while((int32_t)(micros() - expires) < 0L)
    {
        if((regRd(rUSBIRQ) & bmOSCOKIRQ))
        {
            break;
        }
    }

    uint16_t i = 0;

    int32_t now = (int32_t)(micros() - expires);

    if(now < 0L)
    {
        i = 65535 + now; // note this subtracts, as now is negative
    }

    return i;
}


void UHS_NI MAX3421E_HOST::VBUS_changed(void)
{
    // Modify USB task state because Vbus changed or unknown

    uint8_t speed = 1;

    switch(vbusState)
    {
        case LSHOST: // low speed
            speed = 0;
            // Intentional fall-through
        case FSHOST: // full speed
            // Start device initialization if we are not initializing
            // Resets to the device cause an IRQ
            // usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
            //if((usb_task_state & UHS_USB_HOST_STATE_MASK) != UHS_USB_HOST_STATE_DETACHED) {
            ReleaseChildren();
            if(!doingreset)
            {
                if(usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE)
                {
                    usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
                }
                else if(usb_task_state != UHS_USB_HOST_STATE_WAIT_BUS_READY)
                {
                    usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE;
                }
            }
            sof_countdown = 0;
            break;
        case SE1: // illegal state
            sof_countdown = 0;
            doingreset = false;
            ReleaseChildren();
            usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
            break;
        case SE0: // disconnected
        default:
            sof_countdown = 0;
            doingreset = false;
            ReleaseChildren();
            usb_task_state = UHS_USB_HOST_STATE_IDLE;
            break;
    }

    usb_host_speed = speed;
}


// Probe bus to determine device presence and speed,
// then switch host to detected speed
void UHS_NI MAX3421E_HOST::busprobe(void)
{
    uint8_t bus_sample = regRd(rHRSL);      // get J,K status
    bus_sample &= (bmJSTATUS | bmKSTATUS);  // zero the rest of the byte

    uint8_t tmpdata;

    switch(bus_sample) // start full-speed or low-speed host
    {
        case(bmJSTATUS):
            if((regRd(rMODE) & bmLOWSPEED) == 0)
            {
                regWr(rMODE, MODE_FS_HOST); // start full-speed host
                vbusState = FSHOST;
            }
            else
            {
                regWr(rMODE, MODE_LS_HOST); // start low-speed host
                vbusState = LSHOST;
            }
            tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation
            regWr(rHIRQ, bmFRAMEIRQ); // see data sheet
            regWr(rMODE, tmpdata);
            break;
        case(bmKSTATUS):
            if((regRd(rMODE) & bmLOWSPEED) == 0)
            {
                regWr(rMODE, MODE_LS_HOST); // start low-speed host
                vbusState = LSHOST;
            }
            else
            {
                regWr(rMODE, MODE_FS_HOST); // start full-speed host
                vbusState = FSHOST;
            }
            tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation
            regWr(rHIRQ, bmFRAMEIRQ); // see data sheet
            regWr(rMODE, tmpdata);
            break;
        case(bmSE1): // illegal state
            regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
            vbusState = SE1;
            // sofevent = false;
            break;
        case(bmSE0): // disconnected state
            regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
            vbusState = SE0;
            // sofevent = false;
            break;
    }
}


// Initialize USB hardware, turn on VBUS
// @param mseconds Delay energizing VBUS after mseconds, A value of INT16_MIN means no delay.
// returns 0 on success, -1 on error
int16_t UHS_NI MAX3421E_HOST::Init(int16_t mseconds)
{
    usb_task_state = UHS_USB_HOST_STATE_INITIALIZE; // set up state machine

    UHS_printf_HELPER_init();

    if(reset() == 0) // OSCOKIRQ hasn't asserted in time
    {
        MAX_HOST_DEBUG(PSTR("OSCOKIRQ hasn't asserted in time"));
        return -1;
    }

    // Set full-duplex SPI, INT level-active and VBUS OFF
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | GPX_VBDET));

    // Delay a minimum of 1 second to ensure any capacitors are drained
    // 1 second is required to make sure we do not smoke a Microdrive!
    if(mseconds != INT16_MIN)
    {
        if(mseconds < 1000) mseconds = 1000;

        delay(mseconds); // we can't depend on SOF timer here
    }

    // Set pull-downs and Host mode
    regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);

    // Enable interrupts
    regWr(rHIEN, IRQ_CHECK_MASK);

    // Enable INT pin
    regWr(rCPUCTL, bmIE);

    // Check if device is connected
    regWr(rHCTL, bmSAMPLEBUS);            // sample USB bus
    while(!(regRd(rHCTL) & bmSAMPLEBUS)); // wait for sample operation to finish

    busprobe(); // check if anything is connected
    VBUS_changed();

    // !?? GPX pin on. This is done here so that a change is detected if we have a switch connected.
    // Set full-duplex SPI, INT level-active and VBUS ON
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));
    regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet
    regWr(rHCTL, bmBUSRST);      // issue bus reset to force generate yet another possible IRQ

//@TODO: ADD attach interrupt handler (possibly in constructor)
#if USB_HOST_SHIELD_USE_ISR
    // Attach ISR to service IRQ from MAX3421e
    noInterrupts();
    if(irq_pin & 1) {
            ISRodd = this;
            attachInterrupt(UHS_GET_DPI(irq_pin), call_ISRodd, IRQ_SENSE);
    } else {
            ISReven = this;
            attachInterrupt(UHS_GET_DPI(irq_pin), call_ISReven, IRQ_SENSE);
    }
    interrupts();
#endif

    return 0;
}


// Setup UHS_EpInfo structure
/* @param addr USB device address
 * @param ep Endpoint
 * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @return 0 on success */
uint8_t UHS_NI MAX3421E_HOST::SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit)
{
    UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

    if(!p)
        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

    if(!p->epinfo)
        return UHS_HOST_ERROR_NULL_EPINFO;

    *ppep = getEpInfoEntry(addr, ep);

    if(!*ppep)
        return UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE;

    nak_limit = (0x0001UL << (((*ppep)->bmNakPower > UHS_USB_NAK_MAX_POWER) ? UHS_USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
    nak_limit--;

    regWr(rPERADDR, addr); // set peripheral address

    uint8_t mode = regRd(rMODE);

    // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
    regWr(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED | hub_present);

    return 0;
}


// Receive a packet
/* @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytesptr pointer to maximum number of bytes of data to receive
 * @param data pointer to data buffer
 * @return 0 on success */
uint8_t UHS_NI MAX3421E_HOST::InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t* data)
{
    uint8_t rcode = 0;
    uint8_t pktsize;

    uint16_t nbytes = *nbytesptr;
    MAX_HOST_DEBUG(PSTR("Requesting %i bytes "), nbytes);
    uint8_t maxpktsize = pep->maxPktSize;

    *nbytesptr = 0;
    regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); // set toggle value

    // Use a 'break' to exit this loop
    while(1)
    {
        rcode = dispatchPkt(MAX3421E_tokIN, pep->epAddr, nak_limit); // IN packet to EP-'endpoint'. Function takes care of NAKS.

        if(rcode)
        {
            //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! dispatchPkt %2.2x\r\n"), rcode);
            break; // should be 0, indicating ACK. Else return error code.
        }

        // Check for RCVDAVIRQ and generate error if not present
        // the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that
        if((regRd(rHIRQ) & bmRCVDAVIRQ) == 0)
        {
            //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n"));
            rcode = 0xf0; // receive error
            break;
        }

        pktsize = regRd(rRCVBC); // number of received bytes
        MAX_HOST_DEBUG(PSTR("Got %i bytes \r\n"), pktsize);

        if(pktsize > nbytes) // certain devices send more than asked
        {
            //MAX_HOST_DEBUG(PSTR(">>>>>>>> Warning: wanted %i bytes but got %i.\r\n"), nbytes, pktsize);
            pktsize = nbytes;
        }

        int16_t mem_left = (int16_t)nbytes - *((int16_t*)nbytesptr);

        if(mem_left < 0)
            mem_left = 0;

        data = bytesRd(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

        // Clear the IRQ & free the buffer
        regWr(rHIRQ, bmRCVDAVIRQ);
        *nbytesptr += pktsize; // add this packet's byte count to total transfer length

        // The transfer is complete under two conditions:
        // 1. The device sent a short packet (L.T. maxPacketSize)
        // 2. 'nbytes' have been transferred
        if((pktsize < maxpktsize) || (*nbytesptr >= nbytes)) // have we transferred 'nbytes' bytes?
        {
            // Save toggle value
            pep->bmRcvToggle = ((regRd(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
            //MAX_HOST_DEBUG(PSTR("\r\n"));
            rcode = 0;
            break;
        }
    }

    return rcode;
}


// Transmit a packet
/* @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytes number of bytes of data to send
 * @param data pointer to data buffer
 * @return 0 on success */
uint8_t UHS_NI MAX3421E_HOST::OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data)
{
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count;
    uint8_t *data_p = data; // local copy of the data pointer
    uint16_t bytes_tosend;
    uint16_t nak_count;
    uint16_t bytes_left = nbytes;

    uint8_t maxpktsize = pep->maxPktSize;

    if(maxpktsize < 1 || maxpktsize > 64)
        return UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE;

    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;

    // Set toggle value
    regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0);

    while(bytes_left)
    {
        SYSTEM_OR_SPECIAL_YIELD();

        retry_count = 0;
        nak_count = 0;

        bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;

        bytesWr(rSNDFIFO, bytes_tosend, data_p);       // filling output FIFO
        regWr(rSNDBC, bytes_tosend);                   // set number of bytes
        regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); // dispatch packet

        // Wait for the completion IRQ
        while(!(regRd(rHIRQ) & bmHXFRDNIRQ));

        regWr(rHIRQ, bmHXFRDNIRQ); // clear IRQ

        rcode = (regRd(rHRSL) & 0x0f);

        while(rcode && ((long)(millis() - timeout) < 0L))
        {
            switch(rcode)
            {
                case UHS_HOST_ERROR_NAK:
                    nak_count++;
                    if(nak_limit && (nak_count == nak_limit))
                        goto breakout; //@TODO: to change this shit!
                    break;
                case UHS_HOST_ERROR_TIMEOUT:
                    retry_count++;
                    if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                        goto breakout; //@TODO: to change this shit!
                    break;
                case UHS_HOST_ERROR_TOGERR:
                    // Yes, we flip it wrong here so that next time it is actually correct!
                    pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                    regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); // set toggle value
                    break;
                default:
                    goto breakout; //@TODO: to change this shit!
            }

            // Process NAK according to Host out NAK bug
            regWr(rSNDBC, 0);
            regWr(rSNDFIFO, *data_p);
            regWr(rSNDBC, bytes_tosend);
            regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); // dispatch packet

            // Wait for the completion IRQ
            while(!(regRd(rHIRQ) & bmHXFRDNIRQ));

            regWr(rHIRQ, bmHXFRDNIRQ); // clear IRQ

            rcode = (regRd(rHRSL) & 0x0f);

            SYSTEM_OR_SPECIAL_YIELD();
        }

        bytes_left -= bytes_tosend;
        data_p += bytes_tosend;
    }

breakout: //@TODO: to change this shit!

    pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 1 : 0; // bmSNDTOG1 : bmSNDTOG0;  // update toggle

    return rcode; // should be 0 in all cases
}


// Send the actual packet
/* @param token
 * @param ep Endpoint
 * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
 * @return 0 on success, 0xFF indicates NAK timeout. @see */
/* NOTES
 * Assumes peripheral address is set and relevant buffer is loaded/empty
 * If NAK, tries to re-send up to nak_limit times
 * If nak_limit == 0, do not count NAKs, exit after timeout
 * If bus timeout, re-sends up to USB_RETRY_LIMIT times
 * return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout */
uint8_t UHS_NI MAX3421E_HOST::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit)
{
    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
    uint8_t tmpdata;
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count = 0;
    uint16_t nak_count = 0;

    for(;;)
    {
        regWr(rHXFR, (token | ep)); // launch the transfer

        // Wait for transfer completion
        while((long)(millis() - timeout) < 0L)
        {
            SYSTEM_OR_SPECIAL_YIELD();

            tmpdata = regRd(rHIRQ);

            if(tmpdata & bmHXFRDNIRQ)
            {
                regWr(rHIRQ, bmHXFRDNIRQ); // clear the interrupt
                break;
            }
        }

        rcode = (regRd(rHRSL) & 0x0f); // analyze transfer result

        switch(rcode)
        {
            case UHS_HOST_ERROR_NAK:
                nak_count++;
                if(nak_limit && (nak_count == nak_limit))
                    return rcode;
                UsTicker::wait(std::chrono::microseconds(200));
                break;
            case UHS_HOST_ERROR_TIMEOUT:
                retry_count++;
                if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                    return rcode;
                break;
            default:
                return rcode;
        }
    }
}


// NULL is error, we don't need to know the reason

UHS_EpInfo * UHS_NI MAX3421E_HOST::ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr)
{
    uint8_t rcode;
    UHS_EpInfo *pep = NULL;
    uint16_t nak_limit = 0;
    rcode = SetAddress(addr, 0, &pep, nak_limit);

    if(!rcode)
    {
        bytesWr(rSUDFIFO, 8, (uint8_t*)(&Request)); // transfer to setup packet FIFO

        rcode = dispatchPkt(MAX3421E_tokSETUP, 0, nak_limit); // dispatch packet

        if(!rcode)
        {
            if(dataptr != NULL)
            {
                if(((Request)/* bmReqType*/ & 0x80) == 0x80)
                {
                    pep->bmRcvToggle = 1; // bmRCVTOG1;
                }
                else
                {
                    pep->bmSndToggle = 1; // bmSNDTOG1;
                }
            }
        }
        else
        {
            pep = NULL;
        }
    }

    return pep;
}


uint8_t UHS_NI MAX3421E_HOST::ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr)
{
    *read = 0;
    uint16_t nak_limit = 0;
    MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i\r\n"), *left);

    if(*left)
    {
again: //@TODO: to change this shit!

        *read = nbytes;
        uint8_t rcode = InTransfer(pep, nak_limit, read, dataptr);

        if(rcode == UHS_HOST_ERROR_TOGERR)
        {
            // Yes, we flip it wrong here so that next time it is actually correct!
            pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
            goto again; //@TODO: to change this shit!
        }

        if(rcode)
        {
            MAX_HOST_DEBUG(PSTR("ctrlReqRead ERROR: %2.2x, left: %i, read %i\r\n"), rcode, *left, *read);
            return rcode;
        }

        *left -= *read;
        MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i, read %i\r\n"), *left, *read);
    }

    return 0;
}


uint8_t UHS_NI MAX3421E_HOST::ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr)
{
    uint8_t rcode = 0;

    //MAX_HOST_DEBUG(PSTR("Closing"));
    if(((bmReqType & 0x80) == 0x80) && pep && left && dataptr)
    {
        MAX_HOST_DEBUG(PSTR("ctrlReqRead Sinking %i\r\n"), left);
        // If reading, sink the rest of the data
        while(left)
        {
            uint16_t read = nbytes;
            rcode = InTransfer(pep, 0, &read, dataptr);

            if(rcode == UHS_HOST_ERROR_TOGERR)
            {
                // Yes, we flip it wrong here so that next time it is actually correct!
                pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                continue;
            }

            if(rcode) break;
            left -= read;
            if(read < nbytes) break;
        }
    }

    if(!rcode)
    {
        rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? MAX3421E_tokOUTHS : MAX3421E_tokINHS, 0, 0); // GET if direction
    }

    return rcode;
}


// Bottom half of the ISR task
void UHS_NI MAX3421E_HOST::ISRbottom(void)
{
        uint8_t x;
        //        Serial.print("Enter ");
        //        Serial.print((uint32_t)this,HEX);
        //        Serial.print(" ");
        //        Serial.println(usb_task_state, HEX);

        DDSB();
        if(condet) {
                VBUS_changed();
#if USB_HOST_SHIELD_USE_ISR
                noInterrupts();
#endif
                condet = false;
#if USB_HOST_SHIELD_USE_ISR
                interrupts();
#endif
        }
        switch(usb_task_state) {
                case UHS_USB_HOST_STATE_INITIALIZE:
                        // should never happen...
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_INITIALIZE\r\n"));
                        busprobe();
                        VBUS_changed();
                        break;
                case UHS_USB_HOST_STATE_DEBOUNCE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE\r\n"));
                        // This seems to not be needed. The host controller has debounce built in.
                        sof_countdown = UHS_HOST_DEBOUNCE_DELAY_MS;
                        usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE;
                        break;
                case UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE\r\n"));
                        if(!sof_countdown) usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
                        break;
                case UHS_USB_HOST_STATE_RESET_DEVICE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_DEVICE\r\n"));
                        busevent = true;
                        usb_task_state = UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
                        regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.
                        regWr(rHCTL, bmBUSRST); // issue bus reset
                        break;
                case UHS_USB_HOST_STATE_RESET_NOT_COMPLETE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_NOT_COMPLETE\r\n"));
                        if(!busevent) usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
                        break;
                case UHS_USB_HOST_STATE_WAIT_BUS_READY:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_WAIT_BUS_READY\r\n"));
                        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
                        break; // don't fall through

                case UHS_USB_HOST_STATE_CONFIGURING:
                        usb_task_state = UHS_USB_HOST_STATE_CHECK;
                        x = Configuring(0, 1, usb_host_speed);
                        usb_error = x;
                        if(usb_task_state == UHS_USB_HOST_STATE_CHECK) {
                                if(x) {
                                        MAX_HOST_DEBUG(PSTR("Error 0x%2.2x"), x);
                                        if(x == UHS_HOST_ERROR_JERR) {
                                                usb_task_state = UHS_USB_HOST_STATE_IDLE;
                                        } else if(x != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE) {
                                                usb_error = x;
                                                usb_task_state = UHS_USB_HOST_STATE_ERROR;
                                        }
                                } else
                                        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING_DONE;
                        }
                        break;

                case UHS_USB_HOST_STATE_CHECK:
                        // Serial.println((uint32_t)__builtin_return_address(0), HEX);
                        break;
                case UHS_USB_HOST_STATE_CONFIGURING_DONE:
                        usb_task_state = UHS_USB_HOST_STATE_RUNNING;
                        break;
                case UHS_USB_HOST_STATE_RUNNING:
                        Poll_Others();
                        for(x = 0; (usb_task_state == UHS_USB_HOST_STATE_RUNNING) && (x < UHS_HOST_MAX_INTERFACE_DRIVERS); x++) {
                                if(devConfig[x]) {
                                        if(devConfig[x]->bPollEnable) devConfig[x]->Poll();
                                }
                        }
                        // fall thru
                default:
                        // Do nothing
                        break;
        } // switch( usb_task_state )
        DDSB();
#if USB_HOST_SHIELD_USE_ISR
        if(condet) {
                VBUS_changed();
                noInterrupts();
                condet = false;
                interrupts();
        }
#endif

        //usb_task_polling_disabled--;
        EnablePoll();
        DDSB();
}


// USB main task. Services the MAX3421e
void UHS_NI MAX3421E_HOST::ISRTask(void)
{
    DDSB();

    //suspend_host();

    interrupts(); // ??

        counted = false;

        if(!irq_pin)
        {
            uint8_t HIRQALL = regRd(rHIRQ); //determine interrupt source
            uint8_t HIRQ = HIRQALL & IRQ_CHECK_MASK;
            uint8_t HIRQ_sendback = 0x00;

            if((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
                    MAX_HOST_DEBUG
                            (PSTR("\r\nBEFORE CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
                            (HIRQ & bmCONDETIRQ) ? "T" : "F",
                            (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
                            doingreset ? "T" : "F",
                            usb_task_state
                            );
            }
            // ALWAYS happens BEFORE or WITH CONDETIRQ
            if(HIRQ & bmBUSEVENTIRQ) {
                    HIRQ_sendback |= bmBUSEVENTIRQ;
                    if(!doingreset) condet = true;
                    busprobe();
                    busevent = false;
            }

            if(HIRQ & bmCONDETIRQ) {
                    HIRQ_sendback |= bmCONDETIRQ;
                    if(!doingreset) condet = true;
                    busprobe();
            }

#if 1
                if((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
                        MAX_HOST_DEBUG
                                (PSTR("\r\nAFTER CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
                                (HIRQ & bmCONDETIRQ) ? "T" : "F",
                                (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
                                doingreset ? "T" : "F",
                                usb_task_state
                                );
                }
#endif

                if(HIRQ & bmFRAMEIRQ) {
                        HIRQ_sendback |= bmFRAMEIRQ;
                        if(sof_countdown) {
                                sof_countdown--;
                                counted = true;
                        }
                        sofevent = false;
                }

                //MAX_HOST_DEBUG(PSTR("\r\n%s%s%s\r\n"),
                //        sof_countdown ? "T" : "F",
                //        counted ? "T" : "F",
                //        usb_task_polling_disabled? "T" : "F");
                DDSB();
                regWr(rHIRQ, HIRQ_sendback);

        //resume_host();

        noInterrupts(); // ??

                if(!sof_countdown && !counted && !usb_task_polling_disabled) {
                        DisablePoll();
                        //usb_task_polling_disabled++;

                        interrupts(); // ??

                        ISRbottom(); //@TODO: signal semaphore to call after this function...
                }
        }
}


#else
#error "Never include spi_max3421e_INLINE.h, include UHS_host.h instead"
#endif
