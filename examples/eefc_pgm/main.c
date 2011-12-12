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
 * \page eefc_pgm EEFC programming example
 *
 * \section Purpose
 * This basic example shows how to use the Enhance Embedded Flach (EEFC) peripheral
 * available on the newest Atmel SAM3S microcontrollers. It details steps required to
 * program the internal flash, and manage secure and lock bits.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 * The SAM3S ROM code embeds small In Application Programming Procedure.
 * Since this function is executed from ROM, this allows Flash programming (such as sector write)
 * to be done by code running in Flash.\n
 * The IAP function entry point is retrieved by reading the NMI vector in ROM ().
 * This function takes two argument in parameter: bank index (always 0) and the command to be sent to the EEFC.
 *    \code
 *    static uint32_t  (*IAP_PerformCommand)(uint32_t, uint32_t);
 *    IAP_PerformCommand = (uint32_t (*)(uint32_t, uint32_t)) *((uint32_t *) 0x00800008);
 *    IAP_PerformCommand(0, (0x5A << 24) | (argument << 8) | command);
 *     \endcode
 * IAP function returns the value of the MC_FSR register.
 * The required steps are:
 * - Unlock a page.
 * - Program a page of the embedded flash with incremental values (0x0, 0x1, 0x2, 0x3бн.) by using the IAP function.
 * - Check the flash is correctly programmed by reading all the values programmed.
 * - Lock the page.
 * - Set the security bit.
 *
 * The SAM3S features a security bit, based on a specific General Purpose NVM bit 0.
 * When the security is enabled, any access to the Flash, SRAM, Core Registers and Internal
 * Peripherals either through the ICE interface is forbidden.
 * This example will reproduce this scene.
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
 *     -I- Unlocking last page
 *     -I- Writing last page with walking bit pattern
 *     -I- Checking page contents .............................................................. ok
 *     -I- Locking last page
 *     -I- Try to program the locked page...
 *     -I- Please open Segger's JMem program
 *     -I- Read memory at address 0x0043FF00 to check contents
 *     -I- Press any key to continue...
 *     -I- Good job!
 *     -I- Now set the security bit
 *     -I- Press any key to continue to see what happened...
 *     -I- Setting GPNVM #0
 *     -I- All tests done
 * \endcode
 *
 * \section References
 * - eefc_pgm/main.c
 * - efc.c
 * - efc.h
 * - flashd.c
 * - flashd.h
 */
/**
 * \file
 *
 * This file contains all the specific code for the eefc_pgm example.
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
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for eefc programming example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
    uint32_t i;
    uint8_t error;
    uint32_t pBuffer[IFLASH_PAGE_SIZE / 4];
    uint32_t lastPageAddress;
    volatile uint32_t *pLastPageData;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "\n\r\n\r\n\r" ) ;
    printf( "EEFC Programming Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "%s\n\r", BOARD_NAME ) ;
    printf( "Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Performs tests on last page (to avoid overriding existing program).*/
    lastPageAddress = IFLASH_ADDR + IFLASH_SIZE - IFLASH_PAGE_SIZE;
    pLastPageData = (volatile uint32_t *) lastPageAddress;

    /* Unlock page */
    printf("-I- Unlocking last page\n\r");
    error = FLASHD_Unlock(lastPageAddress, lastPageAddress + IFLASH_PAGE_SIZE, 0, 0);
    assert( !error ) ;

    /* Write page with walking bit pattern (0x00000001, 0x00000002, ...) */
    printf("-I- Writing last page with walking bit pattern\n\r");
    for ( i=0 ; i < (IFLASH_PAGE_SIZE / 4); i++ )
    {
        pBuffer[i] = 1 << (i % 32);
    }
    error = FLASHD_Write(lastPageAddress, pBuffer, IFLASH_PAGE_SIZE);
    assert(!error ) ;

    /* Check page contents */
    printf( "-I- Checking page contents " ) ;
    for (i=0; i < (IFLASH_PAGE_SIZE / 4); i++)
    {
        printf(".") ;
        assert( pLastPageData[i] == (uint32_t)(1 << (i % 32)) ) ;
    }
    printf(" ok \n\r") ;

    /* Lock page */
    printf( "-I- Locking last page\n\r" ) ;
    error = FLASHD_Lock( lastPageAddress, lastPageAddress + IFLASH_PAGE_SIZE, 0, 0 ) ;
    assert( !error ) ;

    /* Check that associated region is locked*/
    printf( "-I- Try to program the locked page... \n\r" ) ;
    error = FLASHD_Write( lastPageAddress, pBuffer, IFLASH_PAGE_SIZE ) ;
    if ( error )
    {
        printf( "-I- The page to be programmed belongs to a locked region.\n\r" ) ;
    }

    printf( "-I- Please open Segger's JMem program \n\r" ) ;
    printf( "-I- Read memory at address 0x%08x to check contents\n\r",(unsigned int)lastPageAddress ) ;
    printf( "-I- Press any key to continue...\n\r" ) ;
    while ( !UART_GetChar() ) ;

    printf( "-I- Good job!\n\r" ) ;
    printf( "-I- Now set the security bit \n\r" ) ;
    printf( "-I- Press any key to continue to see what happened...\n\r" ) ;
    while ( !UART_GetChar() ) ;

    /* Set GPNVM bit 0 (security bit) */
    printf( "-I- Setting GPNVM #%d\n\r", 0 ) ;
    error = FLASHD_SetGPNVM( 0 ) ;
    assert( !error ) ;

    printf( "-I- All tests done\n\r" ) ;

    return 0;
}

