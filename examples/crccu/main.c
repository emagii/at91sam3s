/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2009, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \page crccu CRCCU Example
 *
 * \section Purpose
 *
 * This example demonstrates the Cyclic Redundancy Check Calculation Unit (CRCCU)
 * provided on SAM3S microcontrollers. It shows how to use CRCCU
 * to compute CRC on a memory area.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 * The example work as:
 * a. Initialize a buffer in SRAM with random data;
 *    Initialize a buffer in Flash with the same data in SRAM.
 * b. Initialize the CRCCU to compute the CRC of the buffer in SRAM
 *    with one of the 3 CRC algorithm.
 * c. Start the computation.
 * d. Read the result and display it.
 * e. Calculate and display the CRC result in Flash and compare it with
 *    the result in sram .
 * f. Redo b, c, d, e with 2 other CRC modes.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- CRCCU Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 *
 * \section References
 * - crccu/main.c
 * - crccu.c
 * - crccu.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the crccu example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Flash buffer size (in byte) */
#define FLASH_BUFFER_SIZE    IFLASH_PAGE_SIZE
/** Flash buffer address */
#define FLASH_BUFFER_ADDRESS (IFLASH_ADDR + IFLASH_SIZE - FLASH_BUFFER_SIZE)

/** CRC data buffer size (in byte) */
#define BUFFER_LENGTH     64
#if (BUFFER_LENGTH > FLASH_BUFFER_SIZE)
  #error "Flash buffer size is too small."
#endif

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

#ifdef __ICCARM__          /* IAR */
#pragma data_alignment=512
#define __attribute__(...)
#endif
/** CRC descriptor */
__attribute__ ((aligned(512))) CrcDscr crcDscr;

/** CRC data buffer */
uint8_t gDataBuf[BUFFER_LENGTH];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Compute CRC of a buffer.
 *
 * \param pBuffer         the buffer hold the data.
 * \param length          the buffer length.
 * \param polynomialType  polynomial type(CRCCU_MR_PTYPE_XXX).
 *
 * \return CRC of the buffer.
 */
static uint32_t _ComputeCrc( uint8_t *pBuffer, uint32_t length, uint32_t polynomialType )
{
    uint32_t crc ;

    /* Reset crc initial value */
    CRCCU_ResetCrcValue( CRCCU ) ;

    /* Configure crccu */
    memset((void *)&crcDscr, 0, sizeof(crcDscr));
    crcDscr.TR_ADDR = (uint32_t)pBuffer;
    crcDscr.TR_CTRL = (0 << 24) |    /* TRWIDTH: 0 - byte, 1 - halfword, 2 - word */
                      length;        /* BTSIZE */

    CRCCU_Configure( CRCCU, (uint32_t)&crcDscr, CRCCU_MR_ENABLE | polynomialType ) ;

    /* Compute crc */
    crc = CRCCU_ComputeCrc( CRCCU ) ;

    /* Display crc */
    if ( polynomialType == CRCCU_MR_PTYPE_CCIT16 )
    {  /* 16-bits CRC */
        crc &= 0xFFFF ;
        printf( "  CRC of the buffer is 0x%04x\n\r", (unsigned int)crc ) ;
    }
    else
    { /* 32-bits CRC */
        printf( "  CRC of the buffer is 0x%08x\n\r", (unsigned int)crc ) ;
    }

    return crc ;
}

/**
 * \brief Compute CRC of a buffer and compare it with reference CRC.
 *
 * \param pBuffer         the buffer hold the data.
 * \param length          the buffer length.
 * \param polynomialType  polynomial type(CRCCU_MR_PTYPE_XXX).
 * \param refCrc          reference CRC for the buffer.
 *
 * \return CRC of the buffer.
 */
static uint32_t _ComputeCrcAndCompare( uint8_t *pBuffer, uint32_t length, uint32_t polynomialType, uint32_t refCrc )
{
    uint32_t crc ;

    /* Compute crc */
    crc = _ComputeCrc( pBuffer, length, polynomialType ) ;

    /* Compare crc */
    if ( crc == refCrc )
    {
        printf( "  CRC match the reference value.\n\r" ) ;
    }
    else
    {
        printf( "  CRC does NOT match the reference value.\n\r" ) ;
    }

    return crc ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for CRCCU example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    int32_t i ;
    int32_t crc ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- CRCCU Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Enable CRCCU peripheral clock */
    PMC_EnablePeripheral( ID_CRCCU ) ;

    /* Fill data buffer in SRAM (The data may be random data) */
    for ( i = 0 ; i < BUFFER_LENGTH ; i++ )
    {
        gDataBuf[i] = i ;
    }

    /* Fill data buffer in Flash, the data is same as in SRAM */
    FLASHD_Write( FLASH_BUFFER_ADDRESS, (void *)gDataBuf, BUFFER_LENGTH ) ;

    /*==== Test CRC with CCITT-16 polynomial ====*/
    printf( "\n\r====Test CRC with CCITT 16 (0x1021) ====\n\r" ) ;

    /* Compute CRC in SRAM */
    printf( "Test CRC in SRAM buffer\n\r" ) ;
    crc = _ComputeCrc( gDataBuf, BUFFER_LENGTH, CRCCU_MR_PTYPE_CCIT16 ) ;

    /* Compute CRC in Flash and compare it with the result in SRAM */
    printf( "Test CRC in Flash buffer\n\r" ) ;
    _ComputeCrcAndCompare( (uint8_t*)FLASH_BUFFER_ADDRESS, BUFFER_LENGTH, CRCCU_MR_PTYPE_CCIT16, crc ) ;

    /*==== Test CRC with CASTAGNOLI polynomial ====*/
    printf( "\n\r====Test CRC with CASTAGNOLI (0x1EDC6F41) ====\n\r" ) ;

    /* Compute CRC in SRAM */
    printf( "Test CRC in SRAM buffer\n\r" ) ;
    crc = _ComputeCrc( gDataBuf, BUFFER_LENGTH, CRCCU_MR_PTYPE_CASTAGNOLI ) ;
    /* Compute CRC in Flash and compare it with the result in SRAM */
    printf( "Test CRC in Flash buffer\n\r" ) ;
    _ComputeCrcAndCompare( (uint8_t*)FLASH_BUFFER_ADDRESS, BUFFER_LENGTH, CRCCU_MR_PTYPE_CASTAGNOLI, crc ) ;

    /*==== Test CRC with CCIT 802.3 polynomial ====*/
    printf( "\n\r====Test CRC with CCITT 802.3 (0x04C11DB7) ====\n\r" ) ;

    /* Compute CRC in SRAM */
    printf( "Test CRC in SRAM buffer\n\r" ) ;
    crc = _ComputeCrc( gDataBuf, BUFFER_LENGTH, CRCCU_MR_PTYPE_CCIT8023 ) ;
    /* Compute CRC in Flash and compare it with the result in SRAM */
    printf( "Test CRC in Flash buffer\n\r" ) ;
    _ComputeCrcAndCompare( (uint8_t*)FLASH_BUFFER_ADDRESS, BUFFER_LENGTH, CRCCU_MR_PTYPE_CCIT8023, crc ) ;

    return 0;
}

