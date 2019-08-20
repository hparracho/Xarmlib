// ----------------------------------------------------------------------------
// @file    spi_sd_card.hpp
// @brief   SPI SD card class.
// @note    For FatFs use should be included "external/fatfs.hpp" header file
//          instead of this one (setting XARMLIB_ENABLE_FATFS == 1).
// @date    25 July 2019
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

#ifndef __XARMLIB_DEVICES_SPI_SD_CARD_HPP
#define __XARMLIB_DEVICES_SPI_SD_CARD_HPP

// Based on Physical Layer Simplified Specification v6.00 from 29 August 2018
// available in the following URL: https://www.sdcard.org/downloads/pls/index.html

#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{




class SpiSdCard
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiSdCard(hal::SpiMaster& spi_master, const hal::Pin::Name cs) : m_spi_master { spi_master },
                                                                         m_cs(cs, { hal::Gpio::OutputMode::push_pull_high })
        {}

        // Second stage card initialization
        bool start()
        {
            m_type = CardType::unknown;

            m_spi_master.mutex_take();

            m_cs = 1;

            // Saves the current SPI frequency
            const int32_t user_frequency = m_spi_master.get_frequency();

            // Before reset, Send at least 74 clocks at low frequency
            // (between 100kHz and 400kHz) with CS high and DI (MISO) high.
            m_spi_master.set_frequency(400000);

            for(int32_t i = 0; i < 10; i++)
            {
                m_spi_master.transfer(0xFF);
            }

            m_spi_master.mutex_give();

            select();

            // Send CMD0 with CS low to enter SPI mode and reset the card. The card will enter
            // SPI mode if CS is low during the reception of CMD0. Since the CMD0 (and CMD8)
            // must be sent as a native command, the CRC field must have a valid value.
            //if(send_command(CMD_GO_IDLE_STATE, 0, nullptr, 0) == R1_IN_IDLE_STATE) // CMD0
            if(enter_idle_state() == R1_IN_IDLE_STATE)
            {
                // Now the card enters IDLE state.
                // Card type identification Start...

                uint8_t buffer[4];

                // Check the card type, needs around 1000ms
                uint8_t r1 = send_command(CMD_SEND_IF_COND, 0x1AA, buffer, 4); // CMD8

                if((r1 & 0x80) == 0)
                {
                    const auto start = UsTicker::now();

                    if(r1 == R1_IN_IDLE_STATE)
                    {
                        // It's V2.0 or later SD card
                        if(buffer[2] == 0x01 && buffer[3] == 0xAA)
                        {
                            // The card is SD V2 and can work at voltage range of 2.7 to 3.6V
                            do
                            {
                                r1 = send_app_command(APP_CMD_SEND_OP_COND, 0x40000000, nullptr, 0); // ACMD41
                                if(r1 > 0x01) break;
                            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(1000)) == false && r1 != R1_NO_ERROR);

                            if(r1 == R1_NO_ERROR && send_command(CMD_READ_OCR, 0, buffer, 4) == R1_NO_ERROR) // CMD58
                            {
                                m_type = (buffer[0] & 0x40) ? CardType::sdv2_hc : CardType::sdv2_sc;
                            }
                        }
                    }
                    else
                    {
                        // It's Ver1.x SD card or MMC card

                        // Check if it is SD card
                        if(send_command(CMD_APP_CMD, 0, nullptr, 0) & R1_ILLEGAL_CMD)
                        {
                            m_type = CardType::mmc;

                            do
                            {
                                r1 = send_command(CMD_SEND_OP_COND, 0, nullptr, 0);
                            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(1000)) == false && r1);
                        }
                        else
                        {
                            m_type = CardType::sdv1;

                            do
                            {
                                r1 = send_app_command(APP_CMD_SEND_OP_COND, 0, nullptr, 0);
                            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(1000)) == false && r1);
                        }

                        if(r1 != R1_NO_ERROR)
                        {
                            m_type = CardType::unknown;
                        }
                    }

                    // For SDHC or SDXC, block length is fixed to 512 bytes.
                    // For others, the block length is set to 512 manually.
                    if(m_type == CardType::mmc || m_type == CardType::sdv1 || m_type == CardType::sdv2_sc)
                    {
                        if(send_command(CMD_SET_BLOCKLEN, SECTOR_SIZE, nullptr, 0) != R1_NO_ERROR)
                        {
                            m_type = CardType::unknown;
                        }
                    }
                }
            }

            m_spi_master.set_frequency(user_frequency);

            deselect();

            if(m_type != CardType::unknown)
            {
                // Initialization OK
                return true;
            }

            return false;
        }

        // Read the card configuration
        bool read_configuration()
        {
            select();

            // Read OCR
            if(send_command(CMD_READ_OCR, 0, m_config.ocr, 4) != R1_NO_ERROR)
            {
                deselect();
                return false;
            }

            // Read CID
            if(send_command(CMD_SEND_CID, 0, nullptr, 0) != R1_NO_ERROR || receive_data_block(m_config.cid, 16) == false)
            {
                deselect();
                return false;
            }

            // Read CSD
            if(send_command(CMD_SEND_CSD, 0, nullptr, 0) != R1_NO_ERROR || receive_data_block(m_config.csd, 16) == false)
            {
                deselect();
                return false;
            }

            // Sector size
            m_config.sector_size = SECTOR_SIZE;

            // Sector count
            if(((m_config.csd[0] >> 6) & 0x03) == 0x01)
            {
                // CSD V2.0 (for High/eXtended Capacity)

                // Read C_SIZE
                const uint32_t c_size = ((static_cast<uint32_t>(m_config.csd[7]) << 16)
                                      +  (static_cast<uint32_t>(m_config.csd[8]) <<  8)
                                      +  (static_cast<uint32_t>(m_config.csd[9]) <<  0)) & 0x3FFFFF;

                // Calculate sector count
                m_config.sector_count = (c_size + 1) * 1024;
            }
            else
            {
                // CSD V1.0 (for Standard Capacity)

                // C_SIZE
                const uint32_t c_size = (((static_cast<uint32_t>(m_config.csd[6]) & 0x03) << 10)
                                      +  ( static_cast<uint32_t>(m_config.csd[7])         <<  2)
                                      +                         (m_config.csd[8]          >>  6)) & 0xFFF;

                // C_SIZE_MULT
                const uint32_t c_size_mult = ((m_config.csd[9] & 0x03) << 1) + ((m_config.csd[10] & 0x80) >> 7);

                // READ_BL_LEN
                const uint32_t read_bl_len = m_config.csd[5] & 0x0F;

                // Sector count = BLOCKNR * BLOCK_LEN / 512, we manually set SECTOR_SIZE to 512
                m_config.sector_count = (c_size + 1) << (read_bl_len + c_size_mult - 7);
            }

            uint8_t buffer[16];

            // Get erase block size in unit of sector
            switch(m_type)
            {
                case CardType::sdv2_sc:
                case CardType::sdv2_hc:
                    // Read partial block
                    if(send_app_command(APP_CMD_SD_STATUS, 0, buffer, 1) !=  R1_NO_ERROR || receive_data_block(buffer, 16) == false)
                    {
                        deselect();
                        return false;
                    }

                    for(int32_t i = 64 - 16; i; i--)
                    {
                        // Purge trailing data
                        m_spi_master.transfer(0xFF);
                    }

                    // Calculate block size based on AU size
                    m_config.block_size = 16UL << (buffer[10] >> 4);
                    break;

                case CardType::mmc:
                    m_config.block_size = (static_cast<uint16_t>((m_config.csd[10] & 124) >> 2) + 1) *
                                                               (((m_config.csd[10] &   3) << 3) +
                                                                ((m_config.csd[11] & 224) >> 5) + 1);
                    break;

                case CardType::sdv1:
                    m_config.block_size = (((m_config.csd[10] &  63) << 1) +
                                          (static_cast<uint16_t>(m_config.csd[11] & 128) >> 7) + 1) << ((m_config.csd[13] >> 6) - 1);
                    break;

                default:
                    deselect();
                    return false;
            }

            deselect();
            return true;
        }

        // Read a single or multiple sectors from memory card
        bool read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count)
        {
            select();

            // Convert sector-based address to byte-based address for non SDHC
            if(m_type != CardType::sdv2_hc)
            {
                starting_sector <<= 9;
            }

            bool flag = false;

            if(sector_count > 1)
            {
                // Read multiple block

                if(send_command(CMD_READ_MULTIPLE_BLOCK, starting_sector, nullptr, 0) == R1_NO_ERROR)
                {
                    do
                    {
                        if(receive_data_block(data_array, SECTOR_SIZE) == false) break;
                        data_array += SECTOR_SIZE;
                    } while(--sector_count);

                    // Stop transmission
                    send_command(CMD_STOP_TRANSMISSION, 0, nullptr, 0);

                    // Wait for complete
                    if(wait_for_ready() && sector_count == 0)
                    {
                        flag = true;
                    }
                }
            }
            else
            {
                // Read single block

                if((send_command(CMD_READ_SINGLE_BLOCK, starting_sector, nullptr, 0) == R1_NO_ERROR) &&
                    receive_data_block(data_array, SECTOR_SIZE) == true)
                {
                    flag = true;
                }
            }

            deselect();

            return flag;
        }

        // Write a single or multiple sectors to memory card
        bool write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count)
        {
            select();

            // Convert sector-based address to byte-based address for non SDHC
            if(m_type != CardType::sdv2_hc)
            {
                starting_sector <<= 9;
            }

            bool flag = false;

            if(sector_count > 1)
            {
                // Write multiple block

                if(send_command(CMD_WRITE_MULTIPLE_BLOCK, starting_sector, nullptr, 0) == R1_NO_ERROR)
                {
                    do
                    {
                        if(send_data_block(0xFC, data_array, SECTOR_SIZE) == false) break;
                        data_array += SECTOR_SIZE;
                    } while(--sector_count);

                    // Send Stop Transmission Token
                    m_spi_master.transfer(0xFD);

                    // Wait for complete
                    if(wait_for_ready() && sector_count == 0)
                    {
                        flag = true;
                    }
                }
            }
            else
            {
                // Write single block

                if((send_command(CMD_WRITE_SINGLE_BLOCK, starting_sector, nullptr, 0) == R1_NO_ERROR) &&
                    send_data_block(0xFE, data_array, SECTOR_SIZE) == true)
                {
                    flag = true;
                }
            }

            deselect();

            return flag;
        }

#if (XARMLIB_ENABLE_FATFS == 1)
        // Control device specific features and miscellaneous functions other than generic read/write
        bool control(const uint8_t code, void* data)
        {
            uint8_t *ptr = (uint8_t *)data;

            bool res = true;

            switch(code)
            {
                // Make sure that no pending write process
                case CTRL_SYNC:
                    select();
                    if(wait_for_ready() == false)
                    {
                        res = false;
                    }
                    deselect();
                    break;

                // Get number of sectors on the disk (DWORD)
                case GET_SECTOR_COUNT:
                    *(uint32_t *)data = m_config.sector_count;
                    break;

                // Get R/W sector size (WORD)
                case GET_SECTOR_SIZE:
                    *(uint32_t *)data = m_config.sector_size; // 512;
                    break;

                // Get erase block size in unit of sector (DWORD)
                case GET_BLOCK_SIZE:
                    *(uint32_t*)data = m_config.block_size;
                    break;

                // Get card type flags (1 byte)
                case MMC_GET_TYPE:
                    *ptr = static_cast<uint8_t>(m_type);
                    break;

                // Receive CSD as a data block (16 bytes)
                case MMC_GET_CSD:
                    for(uint32_t n=0; n < 16; n++)
                    {
                        *(ptr + n) = m_config.csd[n];
                    }
                    break;

                // Receive CID as a data block (16 bytes)
                case MMC_GET_CID:
                    for(uint32_t n=0; n < 16; n++)
                    {
                        *(ptr + n) = m_config.cid[n];
                    }
                    break;

                // Receive OCR as an R3 resp (4 bytes)
                case MMC_GET_OCR:
                    for(uint32_t n=0; n < 4; n++)
                    {
                        *(ptr + n) = m_config.ocr[n];
                    }
                    break;

                // Receive SD status as a data block (64 bytes)
                case MMC_GET_SDSTAT:
                    for(uint32_t n=0; n < 64; n++)
                    {
                        *(ptr + n) = m_config.status[n];
                    }
                    break;
            }

            return res;
        }
#endif // (XARMLIB_ENABLE_FATFS == 1)

    private:

        // --------------------------------------------------------------------
        // PRIVATE TYPE ALIASES
        // --------------------------------------------------------------------

        using UsTicker = hal::UsTicker;

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // SD/MMC card configuration
        struct CardConfig
        {
            uint32_t sector_size;       	// Size (in byte) of each sector - fixed to 512 bytes
            uint32_t sector_count;      	// Total sector number
            uint32_t block_size;        	// Erase block size in unit of sector
            uint8_t  ocr[4];            	// OCR
            uint8_t  cid[16];           	// CID
            uint8_t  csd[16];           	// CSD
            uint8_t  status[64];        	// Status
        };

        // Memory card type enumeration
        enum class CardType : uint8_t
        {
            unknown = 0,
            mmc,              	            // MMC
            sdv1,             	            // V1.x Standard Capacity SD card
            sdv2_sc,          	            // V2.0 or later Standard Capacity SD card
            sdv2_hc           	            // V2.0 or later High/eXtended Capacity SD card
        };

        // Command definitions in SPI bus mode
        enum CMD : uint8_t
        {
            CMD_GO_IDLE_STATE        = 0,   // CMD0
            CMD_SEND_OP_COND         = 1,   // CMD1
            CMD_SEND_IF_COND         = 8,   // CMD8
            CMD_SEND_CSD             = 9,   // CMD9
            CMD_SEND_CID             = 10,  // CMD10
            CMD_STOP_TRANSMISSION    = 12,  // CMD12
            CMD_SET_BLOCKLEN         = 16,  // CMD16
            CMD_READ_SINGLE_BLOCK    = 17,  // CMD17
            CMD_READ_MULTIPLE_BLOCK  = 18,  // CMD18
            CMD_WRITE_SINGLE_BLOCK   = 24,  // CMD24
            CMD_WRITE_MULTIPLE_BLOCK = 25,  // CMD25
            CMD_APP_CMD              = 55,  // CMD55
            CMD_READ_OCR             = 58,  // CMD58
        };

        // Application specific commands
        // NOTE: all these commands shall be preceded with APP_CMD (CMD55)
        enum APP_CMD : uint8_t
        {
            APP_CMD_SD_STATUS    = 13,      // ACMD13
            APP_CMD_SEND_OP_COND = 41       // ACMD41
        };

        // R1 response bit flag definition
        enum R1 : uint8_t
        {
            R1_NO_ERROR        = 0x00,
            R1_IN_IDLE_STATE   = 0x01,
            R1_ERASE_RESET     = 0x02,
            R1_ILLEGAL_CMD     = 0x04,
            R1_COM_CRC_ERROR   = 0x08,
            R1_ERASE_SEQ_ERROR = 0x10,
            R1_ADDRESS_ERROR   = 0x20,
            R1_PARA_ERROR      = 0x40,
            R1_MASK            = 0x7F
        };

        // The sector size is fixed to 512 bytes in most applications
        enum SECTOR : uint32_t
        {
            SECTOR_SIZE = 512
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Assert memory card (chip select)
        void select()
        {
            m_spi_master.mutex_take();
            m_spi_master.transfer(0xFF);	// Dummy clock (force DO enabled)
            m_cs = 0;
        }

        // De-assert memory card (chip select)
        void deselect()
        {
            m_cs = 1;
            m_spi_master.transfer(0xFF);	// Dummy clock (force DO hi-z for multiple slave SPI)
            m_spi_master.mutex_give();
        }

        // Get assert memory card (chip select) state
        bool is_selected() const
        {
            return (m_cs == 0);
        }

        // Wait for the card to become ready
        bool wait_for_ready()
        {
            const auto start = UsTicker::now();

            do
            {
                if(m_spi_master.transfer(0xFF) == 0xFF)
                {
                    return true;
                }
            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(500)) == false);

            return false;
        }

        uint8_t enter_idle_state()
        {
            uint8_t r1;

            for(int n = 0; n < 5; n++)
            {
                r1 = send_command(CMD_GO_IDLE_STATE, 0, nullptr, 0);
                if(r1 == R1_IN_IDLE_STATE)
                {
                    break;
                }
                UsTicker::wait(std::chrono::milliseconds(1));
            }

            return r1;
        }

        // Receive a data block with specified length from memory card
        // NOTE: data_array_size should be a multiple of 4 (in bytes)
        bool receive_data_block(uint8_t* data_array, const std::size_t data_array_size)
        {
            assert(data_array != nullptr);

            uint8_t token;

            const auto start = UsTicker::now();

            // Read data token (0xFE)
            do
            {
                token = m_spi_master.transfer(0xFF);
            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(100)) == false && token != 0xFE);

            if(token != 0xFE)
            {
                return false;   // Data read timeout
            }

            // Read data block
            for(std::size_t i = 0; i < data_array_size; i++)
            {
                data_array[i] = m_spi_master.transfer(0xFF);
            }

            // 2 bytes CRC will be discarded
            m_spi_master.transfer(0xFF);
            m_spi_master.transfer(0xFF);

            return true;
        }

        // Send a data block with specified length to memory card
        // NOTE: data_array_size should be 512 for memory card (in bytes)
        bool send_data_block(const uint8_t token, const uint8_t* data_array, const std::size_t data_array_size)
        {
            assert(data_array != nullptr);

            // Send start block token
            m_spi_master.transfer(token);

            // Write data block
            for(std::size_t i = 0; i < data_array_size; i++)
            {
                m_spi_master.transfer(data_array[i]);
            }

            // Send 2 bytes dummy CRC
            m_spi_master.transfer(0xFF);
            m_spi_master.transfer(0xFF);

            // Read data response to check if the data block has been accepted
            if((m_spi_master.transfer(0xFF) & 0x0F) != 0x05)
            {
                return false;   // Write error
            }

            const auto start = UsTicker::now();

            uint8_t recv;

            // Wait for write complete
            do
            {
                recv = m_spi_master.transfer(0xFF);
            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(200)) == false && recv != 0xFF);

            if(recv == 0xFF)
            {
                // Write complete
                return true;
            }

            // write time out
            return false;
        }

        // Send a command and receives a response with specified format
        // Returned value: below 0x80 is the normal R1 response (0x0 means no error)
        //                 above 0x80 is the additional returned status code
        //                 0x81: card is not ready
        //                 0x82: command response time out error
        uint8_t send_command(const uint8_t cmd, const uint32_t arg, uint8_t* response_array, const std::size_t response_array_size)
        {
            assert(is_selected() == true);

            deselect();
            select();

            // Wait until the card is ready to read (DI signal is High)
            if(wait_for_ready() == false)
            {
                return 0x81;
            }

            uint8_t crc_stop;

            // Prepare CRC7 + stop bit. For cmd GO_IDLE_STATE and SEND_IF_COND,
            // the CRC7 should be valid, otherwise, the CRC7 will be ignored.
            if      (cmd == CMD_GO_IDLE_STATE) crc_stop = 0x95; // Valid CRC7 + stop bit
            else if (cmd == CMD_SEND_IF_COND)  crc_stop = 0x87; // Valid CRC7 + stop bit
            else                               crc_stop = 0x01; // Dummy CRC7 + Stop bit

            // Send 6-byte command with CRC
            m_spi_master.transfer(cmd | 0x40);
            m_spi_master.transfer(arg >> 24);
            m_spi_master.transfer(arg >> 16);
            m_spi_master.transfer(arg >> 8);
            m_spi_master.transfer(arg);
            m_spi_master.transfer(crc_stop);     // Valid or dummy CRC plus stop bit

            uint32_t r1;

            // The command response time (NCR) is 0 to 8 bytes for SDC, 1 to 8 bytes for MMC.
            for(int32_t i = 8; i; i--)
            {
                r1 = m_spi_master.transfer(0xFF);

                if(r1 != 0xFF) break;   // Received valid response
            }

            if(r1 == 0xFF)
            {
                return 0x82;            // Command response time out error
            }

            // Read remaining bytes after R1 response
            if(response_array != nullptr)
            {
                for(std::size_t i = 0; i < response_array_size; i++)
                {
                    response_array[i] = static_cast<uint8_t>(m_spi_master.transfer(0xFF));
                }
            }

            return static_cast<uint8_t>(r1);
        }

        // Send an application specific command for memory card and receives a response with specified format
        // Returned value: below 0x80 is the normal R1 response (0x0 means no error)
        //                 above 0x80 is the additional returned status code
        //                 0x81: card is not ready
        //                 0x82: command response time out error
        // NOTE: all the application specific commands should be preceded with APP_CMD
        uint8_t send_app_command(const uint8_t cmd, const uint32_t arg, uint8_t* response_array, const std::size_t response_array_size)
        {
            // Send APP_CMD (CMD55) first
            const uint8_t r1 = send_command(CMD_APP_CMD, 0, nullptr, 0);

            if(r1 > 0x01)
            {
                return r1;
            }

            return send_command(cmd, arg, response_array, response_array_size);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        hal::SpiMaster& m_spi_master;                   // SPI bus
        DigitalOut      m_cs;       	                // Chip select
        CardConfig      m_config {};   	                // Card configuration structure
        CardType        m_type { CardType::unknown };   // Card type
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_SD_CARD_HPP
