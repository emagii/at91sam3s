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
 * \page eefc_uniqueid EEFC unique id example
 *
 * \section Purpose
 *
 * This basic example shows how to use the Enhance Embedded Flach (EEFC) peripheral
 * available on the newest Atmel SAM3S microcontrollers.
 * It read and displays the Unique ID stored in the Flash.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 * To read the Unique Identifier the sequence is:
 * <ul>
 *	<li>Send the Start Read unique Identifier command (STUI) by writing the Flash Command
 * Register with the STUI command.</li>
 *	<li>When the Unique Identifier is ready to be read, the FRDY bit in the Flash Programming
 * Status Register (EEFC_FSR) falls.</li>
 *	<li>The Unique Identifier is located in the first 128 bits of the Flash memory mapping. So, at the
 * address 0x400000-0x400003.</li>
 *	<li>To stop the Unique Identifier mode, the user needs to send the Stop Read unique Identifier
 * command (SPUI) by writing the Flash Command Register with the SPUI command.</li>
 *  <li>When the Stop read Unique Unique Identifier command (SPUI) has been performed, the
 * FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises.</li>
 * </ul>
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
 *     -- EEFC Programming Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Reading 128 bits unique identifier
 *     -I- ID: 0xxxxxxxxx, 0xxxxxxxxx, 0xxxxxxxxx, 0xxxxxxxxx
 *
 * \endcode
 * \section References
 * - eefc_uniqueid/main.c
 * - efc.h
 */
/**
 * \file
 *
 * This file contains all the specific code for the eefc_uniqueid example.
 *
 */
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"

#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Read the unique ID.
 *
 * \param uniqueID pointer on a 4bytes char containing the unique ID value.
 */

static
#if defined(flash)
   #ifdef __ICCARM__
__ramfunc
   #else
__attribute__ ((section (".ramfunc")))
   #endif
#endif
void _EEFC_ReadUniqueID( uint32_t* pdwUniqueID )
{
    uint32_t status ;
    assert( pdwUniqueID != NULL ) ;

    pdwUniqueID[0] = 0 ;
    pdwUniqueID[1] = 0 ;
    pdwUniqueID[2] = 0 ;
    pdwUniqueID[3] = 0 ;

    /* Send the Start Read unique Identifier command (STUI) by writing the Flash Command Register with the STUI command.*/
    EFC->EEFC_FCR = (0x5A << 24) | EFC_FCMD_STUI;

    /* The Unique Identifier is located in the first 128 bits of the Flash memory mapping. So, at the address 0x400000-0x400003. */
    pdwUniqueID[0] = *(uint32_t *)IFLASH_ADDR;
    pdwUniqueID[1] = *(uint32_t *)(IFLASH_ADDR + 4);
    pdwUniqueID[2] = *(uint32_t *)(IFLASH_ADDR + 8);
    pdwUniqueID[3] = *(uint32_t *)(IFLASH_ADDR + 12);

    /* To stop the Unique Identifier mode, the user needs to send the Stop Read unique Identifier
       command (SPUI) by writing the Flash Command Register with the SPUI command. */
    EFC->EEFC_FCR = (0x5A << 24) | EFC_FCMD_SPUI ;

    /* When the Stop read Unique Unique Identifier command (SPUI) has been performed, the
       FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises. */
    do
    {
        status = EFC->EEFC_FSR ;
    } while ( (status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY ) ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for eefc programming example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
    uint32_t adwUniqueID[4] ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "\n\r\n\r\n\r" ) ;
    printf( "EEFC Uniqueid Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "%s\n\r", BOARD_NAME ) ;
    printf( "Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Read the unique ID. */
    printf("-I- Reading 128 bits unique identifier \n\r" ) ;
    _EEFC_ReadUniqueID( adwUniqueID ) ;

    printf ("-I- ID: 0x%08x, 0x%08x, 0x%08x, 0x%08x \n\r",
        (unsigned int)adwUniqueID[0],
        (unsigned int)adwUniqueID[1],
        (unsigned int)adwUniqueID[2],
        (unsigned int)adwUniqueID[3]);

    return 0 ;
}

