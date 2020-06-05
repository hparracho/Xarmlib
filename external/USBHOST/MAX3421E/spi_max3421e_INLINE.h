// ----------------------------------------------------------------------------
// @file    spi_max3421e_INLINE.h
// @brief   SPI MAX3421E driver class implementation.
// @notes   Based on UHS30 USB_HOST_SHIELD_INLINE.h file suitable for Xarmlib
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

#if defined(SPI_MAX3421E_H) && !defined(USB_HOST_SHIELD_LOADED)
#define USB_HOST_SHIELD_LOADED


// ----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

void UHS_NI MAX3421E_HOST::initialize()
{
    usb_task_state = TaskState::illegal; // set up state machine

    // Set full-duplex mode and negative edge interrupt pin
    write_reg(rPINCTL, bmFDUPSPI);

    reset();

    // Delay a minimum of 1 second to ensure any capacitors are drained
    // 1 second is required to make sure we do not smoke a Microdrive!
    UsTicker::wait(std::chrono::milliseconds(1000)); // we can't depend on SOF timer here

    // Activate HOST mode and turn on the 15K pulldown resistors on D+ and D-
    write_reg(rMODE, (bmDPPULLDN | bmDMPULLDN |bmHOST)); // NOTE: initially set up as a FS host (LOWSPEED=0)

    // Clear the interrupts on the MAX3421e
    write_reg(rHIRQ, (bmFRAMEIRQ | bmCONDETIRQ | bmBUSEVENTIRQ));

    // Create and assign the PinInt IRQ handler to the MAX3421E_HOST::isr() member function
    auto handler = PinInt::IrqHandler::create<MAX3421E_HOST, &MAX3421E_HOST::isr>(this);
    m_max_int.assign_irq_handler(handler);

    m_max_int.enable_interrupt();

    // Enable interrupts on the MAX3421e
    write_reg(rHIEN, (bmFRAMEIE | bmCONDETIE | bmBUSEVENTIE));

    // Enable INT pin
    write_reg(rCPUCTL, bmIE);

    // See if anything is plugged in
    write_reg(rHCTL, bmSAMPLEBUS);           // update the JSTATUS and KSTATUS bits
    while(!(read_reg(rHCTL) & bmSAMPLEBUS)); // wait for sample operation to finish

    vbus_changed();
}


void UHS_NI MAX3421E_HOST::task()
{
    DBG("Processing ");
    DBG_state();

    switch(usb_task_state)
    {
        case TaskState::illegal: break;

        case TaskState::idle: break;

        case TaskState::bus_reset:
            usb_task_state = TaskState::wait_bus_ready;
            break;

        case TaskState::wait_bus_ready:
            usb_task_state = TaskState::configuring;
            DBG("Device reseted\n");
            break;

        case TaskState::configuring:
            usb_error = Configuring(0, 1, usb_host_speed);
            if(usb_error)
            {
                DBG("Error 0x{:X}", usb_error);
                if(usb_error == UHS_HOST_ERROR_JERR)
                {
                    usb_task_state = TaskState::idle;
                }
                else if(usb_error != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE)
                {
                    usb_task_state = TaskState::error;
                }
                else
                {
                    usb_task_state = TaskState::check;
                }
            }
            else
            {
                usb_task_state = TaskState::running;
            }
            break;

        case TaskState::check: break;

        case TaskState::error: break;

        case TaskState::running:
            for(uint8_t x = 0; (usb_task_state == TaskState::running) && (x < UHS_HOST_MAX_INTERFACE_DRIVERS); x++)
            {
                if(devConfig[x])
                {
                    if(devConfig[x]->bPollEnable) devConfig[x]->Poll();
                }
            }
            break;
    }
}


// ----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

int32_t UHS_NI MAX3421E_HOST::isr()
{
    int32_t yield = 0;  // User in FreeRTOS

    const uint8_t hirq_flags = read_reg(rHIRQ) & ICLRALLBITS; // determine interrupt source

    // ALWAYS happens BEFORE or WITH CONDETIRQ
    if(hirq_flags & bmBUSEVENTIRQ)
    {
        m_bus_event = true;
        m_task_pending |= (usb_task_state == TaskState::bus_reset);
    }
    else
    {
        if(hirq_flags & bmCONDETIRQ)
        {
            vbus_changed();
        }
        else if(hirq_flags & bmFRAMEIRQ)
        {
            m_sof_event = true;
            m_task_pending |= (usb_task_state == TaskState::wait_bus_ready);
        }
    }

    write_reg(rHIRQ, hirq_flags); // clear interrupt source

    return yield;
}


void UHS_NI MAX3421E_HOST::reset()
{
    write_reg(rUSBCTL, bmCHIPRES);  // chip reset - this stops the oscillator
    write_reg(rUSBCTL, 0x00);       // remove the reset

    while(!(read_reg(rUSBIRQ) & bmOSCOKIRQ)); // hang until the PLL stabilizes
}


void UHS_NI MAX3421E_HOST::vbus_changed()
{
    uint8_t busstate = read_reg(rHRSL);  // get the JSTATUS and KSTATUS bits
    busstate &= (bmJSTATUS | bmKSTATUS); // check for either of them high

    switch(busstate) // start full-speed or low-speed host
    {
        case(bmJSTATUS):
            if((read_reg(rMODE) & bmLOWSPEED) == 0)
            {
                write_reg(rMODE, MODE_FS_HOST); // start full-speed host
                usb_host_speed = 1;
                DBG("Full-Speed Device Detected\n");
            }
            else
            {
                write_reg(rMODE, MODE_LS_HOST); // start low-speed host
                usb_host_speed = 0;
                DBG("Low-Speed Device Detected\n");
            }
            release_children();
            usb_task_state = TaskState::bus_reset;
            issue_bus_reset();
            break;

        case(bmKSTATUS):
            if((read_reg(rMODE) & bmLOWSPEED) == 0)
            {
                write_reg(rMODE, MODE_LS_HOST); // start low-speed host
                usb_host_speed = 0;
                DBG("Low-Speed Device Detected\n");
            }
            else
            {
                write_reg(rMODE, MODE_FS_HOST); // start full-speed host
                usb_host_speed = 1;
                DBG("Full-Speed Device Detected\n");
            }
            release_children();
            usb_task_state = TaskState::bus_reset;
            issue_bus_reset();
            break;

        case(bmSE1): // illegal state
            write_reg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // turn off frame markers
            usb_host_speed = 1;
            release_children();
            usb_task_state = TaskState::illegal;
            DBG_state();
            break;

        case(bmSE0): // disconnected state
            write_reg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // turn off frame markers
            usb_host_speed = 1;
            release_children();
            usb_task_state = TaskState::idle;
            DBG_state();
            break;
    }
}


void UHS_NI MAX3421E_HOST::issue_bus_reset()
{
    //DBG("Issuing USB bus reset\n");

    write_reg(rHIRQ, bmBUSEVENTIRQ); // see MAX3421E Programming Guide pages 19 and 20
    write_reg(rHCTL, bmBUSRST);      // initiate the 50 msec bus reset
}


void UHS_NI MAX3421E_HOST::release_children()
{
    for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
    {
        if(devConfig[i])
        {
            devConfig[i]->Release();
        }
    }

    hub_present = 0;
}


void UHS_NI MAX3421E_HOST::doHostReset()
{
    m_bus_event = false;

    issue_bus_reset();

    while(m_bus_event == false)
    {
#ifdef XARMLIB_ENABLE_FREERTOS
        taskYIELD();
#endif
    }

    m_sof_event = false;

    // Wait for SOF
    while(m_sof_event == false);
}


/**
 * Setup UHS_EpInfo structure
 *
 * @param addr USB device address
 * @param ep Endpoint
 * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @return 0 on success
 */
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
    /*
      USBTRACE2("\r\nAddress: ", addr);
      USBTRACE2(" EP: ", ep);
      USBTRACE2(" NAK Power: ",(*ppep)->bmNakPower);
      USBTRACE2(" NAK Limit: ", nak_limit);
      USBTRACE("\r\n");
     */
    write_reg(rPERADDR, addr); //set peripheral address

    uint8_t mode = read_reg(rMODE);

    //Serial.print("\r\nMode: ");
    //Serial.println( mode, HEX);
    //Serial.print("\r\nLS: ");
    //Serial.println(p->speed, HEX);

    // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
    write_reg(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED | hub_present);

    return 0;
}


/**
 * Transmit a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytes number of bytes of data to send
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data)
{
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count;
    uint8_t *data_p = data; //local copy of the data pointer
    uint16_t bytes_tosend;
    uint16_t nak_count;
    uint16_t bytes_left = nbytes;

    uint8_t maxpktsize = pep->maxPktSize;

    if(maxpktsize < 1 || maxpktsize > 64)
            return UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE;

    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;

    write_reg(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value

    while(bytes_left) {
            //SYSTEM_OR_SPECIAL_YIELD();
#ifdef XARMLIB_ENABLE_FREERTOS
            taskYIELD();
#endif
            retry_count = 0;
            nak_count = 0;
            bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
            write_bytes(rSNDFIFO, bytes_tosend, data_p); //filling output FIFO
            write_reg(rSNDBC, bytes_tosend); //set number of bytes
            write_reg(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); //dispatch packet
            while(!(read_reg(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ
            write_reg(rHIRQ, bmHXFRDNIRQ); //clear IRQ
            rcode = (read_reg(rHRSL) & 0x0f);

            while(rcode && ((long)(millis() - timeout) < 0L)) {
                    switch(rcode) {
                            case UHS_HOST_ERROR_NAK:
                                    nak_count++;
                                    if(nak_limit && (nak_count == nak_limit))
                                            goto breakout;
                                    break;
                            case UHS_HOST_ERROR_TIMEOUT:
                                    retry_count++;
                                    if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                                            goto breakout;
                                    break;
                            case UHS_HOST_ERROR_TOGERR:
                                    // yes, we flip it wrong here so that next time it is actually correct!
                                    pep->bmSndToggle = (read_reg(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                                    write_reg(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value
                                    break;
                            default:
                                    goto breakout;
                    }//switch( rcode

                    /* process NAK according to Host out NAK bug */
                    write_reg(rSNDBC, 0);
                    write_reg(rSNDFIFO, *data_p);
                    write_reg(rSNDBC, bytes_tosend);
                    write_reg(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); //dispatch packet
                    while(!(read_reg(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ
                    write_reg(rHIRQ, bmHXFRDNIRQ); //clear IRQ
                    rcode = (read_reg(rHRSL) & 0x0f);
                    //SYSTEM_OR_SPECIAL_YIELD();
#ifdef XARMLIB_ENABLE_FREERTOS
                    taskYIELD();
#endif
            }//while( rcode && ....
            bytes_left -= bytes_tosend;
            data_p += bytes_tosend;
    }//while( bytes_left...
breakout:

    pep->bmSndToggle = (read_reg(rHRSL) & bmSNDTOGRD) ? 1 : 0; //bmSNDTOG1 : bmSNDTOG0;  //update toggle
    return ( rcode); //should be 0 in all cases
}


/**
 * Receive a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytesptr pointer to maximum number of bytes of data to receive
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t* data)
{
    uint8_t rcode = 0;
    uint8_t pktsize;

    uint16_t nbytes = *nbytesptr;
    DBG("Requesting {} bytes ", nbytes);
    uint8_t maxpktsize = pep->maxPktSize;

    *nbytesptr = 0;
    write_reg(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); //set toggle value

    // use a 'break' to exit this loop
    while(1) {
            rcode = dispatchPkt(MAX3421E_tokIN, pep->epAddr, nak_limit); //IN packet to EP-'endpoint'. Function takes care of NAKS.

            if(rcode) {
                    //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! dispatchPkt %2.2x\r\n"), rcode);
                    break; //should be 0, indicating ACK. Else return error code.
            }
            /* check for RCVDAVIRQ and generate error if not present */
            /* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
            if((read_reg(rHIRQ) & bmRCVDAVIRQ) == 0) {
                    //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n"));
                    rcode = 0xf0; //receive error
                    break;
            }
            pktsize = read_reg(rRCVBC); //number of received bytes
            DBG("Got {} bytes \n", pktsize);

            if(pktsize > nbytes) { //certain devices send more than asked
                    //MAX_HOST_DEBUG(PSTR(">>>>>>>> Warning: wanted %i bytes but got %i.\r\n"), nbytes, pktsize);
                    pktsize = nbytes;
            }

            int16_t mem_left = (int16_t)nbytes - *((int16_t*)nbytesptr);

            if(mem_left < 0)
                    mem_left = 0;

            data = read_bytes(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

            write_reg(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
            *nbytesptr += pktsize; // add this packet's byte count to total transfer length

            /* The transfer is complete under two conditions:           */
            /* 1. The device sent a short packet (L.T. maxPacketSize)   */
            /* 2. 'nbytes' have been transferred.                       */
            if((pktsize < maxpktsize) || (*nbytesptr >= nbytes)) // have we transferred 'nbytes' bytes?
            {
                    // Save toggle value
                    pep->bmRcvToggle = ((read_reg(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
                    //MAX_HOST_DEBUG(PSTR("\r\n"));
                    rcode = 0;
                    break;
            } // if
    } //while( 1 )
    return ( rcode);
}


//
// NULL is error, we don't need to know the reason.
//

UHS_EpInfo * UHS_NI MAX3421E_HOST::ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr)
{
    uint8_t rcode;
    UHS_EpInfo *pep = NULL;
    uint16_t nak_limit = 0;
    rcode = SetAddress(addr, 0, &pep, nak_limit);

    if(!rcode) {

            write_bytes(rSUDFIFO, 8, (uint8_t*)(&Request)); //transfer to setup packet FIFO

            rcode = dispatchPkt(MAX3421E_tokSETUP, 0, nak_limit); //dispatch packet
            if(!rcode) {
                    if(dataptr != NULL) {
                            if(((Request)/* bmReqType*/ & 0x80) == 0x80) {
                                    pep->bmRcvToggle = 1; //bmRCVTOG1;
                            } else {
                                    pep->bmSndToggle = 1; //bmSNDTOG1;
                            }
                    }
            } else {
                    pep = NULL;
            }
    }
    return pep;
}


uint8_t UHS_NI MAX3421E_HOST::ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr)
{
    uint8_t rcode = 0;

    //MAX_HOST_DEBUG(PSTR("Closing"));
    if(((bmReqType & 0x80) == 0x80) && pep && left && dataptr) {
            DBG("ctrlReqRead Sinking {}\n", left);
            // If reading, sink the rest of the data.
            while(left) {
                    uint16_t read = nbytes;
                    rcode = InTransfer(pep, 0, &read, dataptr);
                    if(rcode == UHS_HOST_ERROR_TOGERR) {
                            // yes, we flip it wrong here so that next time it is actually correct!
                            pep->bmRcvToggle = (read_reg(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                            continue;
                    }
                    if(rcode) break;
                    left -= read;
                    if(read < nbytes) break;
            }
    }
    if(!rcode) {
            //               Serial.println("Dispatching");
            rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? MAX3421E_tokOUTHS : MAX3421E_tokINHS, 0, 0); //GET if direction
            //        } else {
            //                Serial.println("Bypassed Dispatch");
    }
    return rcode;
}


uint8_t UHS_NI MAX3421E_HOST::ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr)
{
    *read = 0;
    uint16_t nak_limit = 0;
    DBG("ctrlReqRead left: {}\n", *left);
    if(*left) {
again:
            *read = nbytes;
            uint8_t rcode = InTransfer(pep, nak_limit, read, dataptr);
            if(rcode == UHS_HOST_ERROR_TOGERR) {
                    // yes, we flip it wrong here so that next time it is actually correct!
                    pep->bmRcvToggle = (read_reg(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                    goto again;
            }

            if(rcode) {
                    DBG("ctrlReqRead ERROR: 0x{:X}, left: {}, read {}\n", rcode, *left, *read);
                    return rcode;
            }
            *left -= *read;
            DBG("ctrlReqRead left: {}, read {}\n", *left, *read);
    }
    return 0;
}


/**
 * Send the actual packet.
 *
 * @param token
 * @param ep Endpoint
 * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
 * @return 0 on success, 0xFF indicates NAK timeout. @see
 */
/* Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */

/* return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout                       */
uint8_t UHS_NI MAX3421E_HOST::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit)
{
    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
    uint8_t tmpdata;
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count = 0;
    uint16_t nak_count = 0;

    for(;;) {
            write_reg(rHXFR, (token | ep)); //launch the transfer
            while((long)(millis() - timeout) < 0L) //wait for transfer completion
            {
                    //SYSTEM_OR_SPECIAL_YIELD();
#ifdef XARMLIB_ENABLE_FREERTOS
                    taskYIELD();
#endif
                    tmpdata = read_reg(rHIRQ);

                    if(tmpdata & bmHXFRDNIRQ) {
                            write_reg(rHIRQ, bmHXFRDNIRQ); //clear the interrupt
                            //rcode = 0x00;
                            break;
                    }//if( tmpdata & bmHXFRDNIRQ

            }//while ( millis() < timeouthttps://www.mouser.com/Search/Refine.aspx?Keyword=

            rcode = (read_reg(rHRSL) & 0x0f); //analyze transfer result

            switch(rcode) {
                    case UHS_HOST_ERROR_NAK:
                            nak_count++;
                            if(nak_limit && (nak_count == nak_limit))
                                    return (rcode);
                            //delayMicroseconds(200);
                            UsTicker::wait(std::chrono::microseconds(200));
                            break;
                    case UHS_HOST_ERROR_TIMEOUT:
                            retry_count++;
                            if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                                    return (rcode);
                            break;
                    default:
                            return (rcode);
            }//switch( rcode
    }
}


// Write single byte into MAX3421e register
void UHS_NI MAX3421E_HOST::write_reg(const uint8_t reg, const uint8_t data)
{
    m_spi->mutex_take();
    m_ss = 0;

    m_spi->transfer(reg | 0x02);
    m_spi->transfer(data);

    m_ss = 1;
    m_spi->mutex_give();
}


// Single host register read
uint8_t UHS_NI MAX3421E_HOST::read_reg(const uint8_t reg)
{
    m_spi->mutex_take();
    m_ss = 0;

    m_spi->transfer(reg);
    uint8_t rv = m_spi->transfer(0x00); // send empty byte

    m_ss = 1;
    m_spi->mutex_give();

    return rv;
}


// Multiple-byte write
// returns a pointer to memory position after last written
uint8_t* UHS_NI MAX3421E_HOST::write_bytes(const uint8_t reg, uint8_t nbytes, uint8_t* data_p)
{
    m_spi->mutex_take();
    m_ss = 0;

    m_spi->transfer(reg | 0x02);

    while(nbytes)
    {
        m_spi->transfer(*data_p);
        nbytes--;
        data_p++; // advance data pointer
    }

    m_ss = 1;
    m_spi->mutex_give();

    return data_p;
}


// Multiple-byte register read
// returns a pointer to a memory position after last read
uint8_t* UHS_NI MAX3421E_HOST::read_bytes(const uint8_t reg, uint8_t nbytes, uint8_t* data_p)
{
    m_spi->mutex_take();
    m_ss = 0;

    m_spi->transfer(reg);

    while(nbytes)
    {
        *data_p++ = m_spi->transfer(0x00);
        nbytes--;
    }

    m_ss = 1;
    m_spi->mutex_give();

    return data_p;
}


// GPIO Write
// GPIO byte is split between 2 registers, so two writes are needed to write one byte
// GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2
void UHS_NI MAX3421E_HOST::write_gpio(uint8_t data)
{
    write_reg(rIOPINS1, data);
    data >>= 4;
    write_reg(rIOPINS2, data);
}


// GPIO read
// (see gpioWr for explanation)
// GPIN pins are in high nibbles of IOPINS1, IOPINS2
uint8_t UHS_NI MAX3421E_HOST::read_gpio()
{
    uint8_t gpin = 0;

    gpin  = read_reg(rIOPINS2);        // pins 4-7
    gpin &= 0xf0;                      // clean lower nibble
    gpin |= (read_reg(rIOPINS1) >> 4); // shift low bits and OR with upper from previous operation.

    return gpin;
}


#ifdef DEBUG_USB_HOST
void UHS_NI MAX3421E_HOST::DBG_state()
{
    switch(usb_task_state)
    {
        case TaskState::illegal: DBG("state: illegal\n"); break;
        case TaskState::idle: DBG("state: idle\n"); break;
        case TaskState::bus_reset: DBG("state: bus reset\n"); break;
        case TaskState::wait_bus_ready: DBG("state: wait bus ready\n"); break;
        case TaskState::configuring: DBG("state: configuring\n"); break;
        case TaskState::check: DBG("state: check\n"); break;
        case TaskState::error: DBG("state: error\n"); break;
        case TaskState::running: DBG("state: running\n"); break;
    }
}
#endif


#else
#error "Never include spi_max3421e_INLINE.h, include UHS_host.h instead"
#endif
