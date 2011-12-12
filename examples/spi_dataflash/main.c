/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
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
 * \Page spi_dataflash SPI Dataflash Example
 *
 * \section Purpose
 *
 * The spi_dataflash example will help new users get familiar with SPI interface
 * on sam3s. This example gives you an AT45 Dataflash programming code so
 * that can help develop your own SPI devices applications with maximum
 * efficiency.
 *
 * You can find following information depends on your needs:
 * - A Spi low level driver performs SPI device Initializes, data transfer and
 * receive. It can be used by upper SPI driver such as AT45 %dataflash.
 * - A Dataflash driver is based on top of the corresponding Spi driver.
 * It allow user to do operations with %dataflash in a unified way.
 *
 * \section Requirements
 *
 * This package can be used with sam3s-ek and external Data Flash connected.
 * Please connect the SPI peripheral to external board like following matching
 * pairs:
 *        - <b>SAM3S--DataFlash</b>
 *        - VCC--VCC
 *        - GND--GND
 *        - NPCS0--NSS
 *        - MISO--MISO
 *        - MOSI--MOSI
 *        - SPCK--SPCK
 *
 * \section Description
 *
 * The demonstration program tests the dataflash connected to the evaluation kit by
 * erasing and writing each one of its pages.
 *
 * \section Requirements
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">SAM-BA User Guide</a>,
 *    the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
 *    application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the terminal.
 *    \code
 *    -- SPI Dataflash Example xxx --
 *    -- xxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    -I- Initializing the SPI and AT45 drivers
 *    -I- At45 enabled
 *    -I- Waiting for a dataflash to be connected ...
 *    \endcode
 * -# As soon as a dataflash is connected, the tests will start. Eventually,
 *    the test result (pass or fail) will be output to the hyperterminal.
  * \section References
 * - spi_dataflash/main.c
 * - spi_pdc.c
 * - spi_at45.c
 * - at45d.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi_dataflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Internal definitions
 *----------------------------------------------------------------------------*/
/** SPI clock frequency, in Hz.*/
#define SPCK        1000000

/*----------------------------------------------------------------------------
 *        Internal variables
 *----------------------------------------------------------------------------*/

/** SPI driver instance.*/
static Spid spid;

/** AT45 driver instance.*/
static At45 at45;

/** Pins used by the application.*/
static const Pin pins[]  = {PINS_SPI, PIN_SPI_NPCS0_PA11};

/** Page buffer.*/
static uint8_t pBuffer[2112];


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Tests the At45 connected to the board by performing several command
 * on each of its pages.
 */
extern int main( void )
{
    uint32_t i ;
    uint32_t dwPage ;
    uint8_t testFailed ;
    const At45Desc *pDesc ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- SPI  Dataflash Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure pins */
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    /* SPI and At45 driver initialization. */
    printf("-I- Initializing the SPI and AT45 drivers\n\r" ) ;
    SPID_Configure( &spid, SPI, ID_SPI ) ;
    SPID_ConfigureCS( &spid, 0, AT45_CSR( BOARD_MCK, SPCK ) ) ;
    AT45_Configure( &at45, &spid, 0 ) ;
    printf( "-I- At45 enabled\n\r" ) ;

    /* Identify the At45 device*/
    printf( "-I- Waiting for a dataflash to be connected ...\n\r" ) ;
    pDesc = 0 ;
    while ( !pDesc )
    {
        pDesc = AT45_FindDevice( &at45, AT45D_GetStatus( &at45 ) ) ;
    }
    printf( "-I- %s detected\n\r", at45.pDesc->name ) ;

    /* Test all pages. */
    testFailed = 0 ;
    dwPage = 0 ;

    while ( !testFailed && (dwPage < AT45_PageNumber( &at45 )) )
    {
        printf( "-I- Test in progress on page: %6u\r", (unsigned int)dwPage ) ;

        /* Erase dwPage */
        AT45D_Erase( &at45, dwPage * AT45_PageSize( &at45 ) ) ;

        /* Verify that dwPage has been erased correctly */
        memset( pBuffer, 0, AT45_PageSize( &at45 ) ) ;
        AT45D_Read( &at45, pBuffer, AT45_PageSize( &at45 ), dwPage * AT45_PageSize( &at45 ) ) ;

        for ( i=0 ; i < AT45_PageSize( &at45 ) ; i++ )
        {
            if ( pBuffer[i] != 0xff )
            {
                printf( "-E- Could not erase page %u\n\r", (unsigned int)dwPage ) ;
                testFailed = 1 ;
                break;
            }
        }

        /* Write dwPage */
        for ( i=0; i < AT45_PageSize( &at45 ) ; i++ )
        {
            pBuffer[i] = i & 0xFF ;
        }
        AT45D_Write( &at45, pBuffer, AT45_PageSize( &at45 ), dwPage * AT45_PageSize( &at45 ) ) ;

        /* Check that data has been written correctly.*/
        memset( pBuffer, 0, AT45_PageSize( &at45 ) ) ;
        AT45D_Read( &at45, pBuffer, AT45_PageSize( &at45 ), dwPage * AT45_PageSize( &at45 ) ) ;

        for ( i=0; i < AT45_PageSize( &at45 ) ; i++ )
        {
            if ( pBuffer[i] != (i & 0xFF) )
            {
                printf( "-E- Could not write dwPage %u\n\r", (unsigned int)dwPage ) ;
                testFailed = 1 ;
                break ;
            }
        }

        dwPage++ ;
    }

    /* Display test result */
    if ( testFailed )
    {
        printf( "-E- Test failed.\n\r" ) ;
    }
    else
    {
         printf( "-I- Test passed.\n\r" ) ;
    }

    return 0 ;
}

