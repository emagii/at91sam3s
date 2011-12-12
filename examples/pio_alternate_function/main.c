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
 *  \page pio_alternate_function PIO Alternate Function Example
 *
 *  \section Purpose
 *
 *  The PIO Alternate Function demonstrates how to alternate ERASE pin between system
 *  I/O mode and PIO mode.
 *
 *  \section Requirements
 *
 *  This package can only be used with sam3s-ek.
 *
 *  \section Description
 *
 *  The application shows the alternate function of ERASE pin, which extends PIO
 *  from a dedicated system I/O line. It first programs the internal flash with
 *  a walking pattern and then enable ERASE pin in PIO mode. After the erase
 *  operation, it check the content of flash, which is unchanged due to ERASE in
 *  PIO mode. In the next step, it put ERASE pin in system I/O mode,which will
 *  enable the erase function. The content of flash should be erased to 0xFF, which
 *  indicates the ERASE pin functioning.
 *
 *  The pins which are used by USB(DDM,DDP),JTAG(TCK,TMS,TDO,TDI) and Oscillator(XIN,
 *  XOUT,XIN32,XOUT32) could also be alternated between general purpose I/O and system
 *  I/O in the similar way.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or to the
 *     <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *     IAR EWARM User Guide</a>,
 *     depending on your chosen solution.
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start application.
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- PIO Alternate Function Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# Perform erase operation following the application prompt.
 *
 *  \section References
 *  - pio_alternate_function/main.c
 *  - pio.h, pio_it.h
 *  - flashd.h
 */

/** \file
 *
 *  This file contains all the specific code for the pio_alternate_function.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Pushbutton \#1 pin instance. */
const Pin pinPB1 = PIN_PUSHBUTTON_1;

/** Pushbutton \#1 pin event flag. */
volatile bool button1Evt = false;


/**
 *  \brief Handler for Button 1 rising edge interrupt.
 *
 *  Set button1 event flag (button1Evt).
 */
static void _Button1_Handler( const Pin *pin )
{
    if ( pin->mask == pinPB1.mask && pin->id == pinPB1.id )
    {
        button1Evt = true ;
    }
    else
    {
        button1Evt = false ;
    }
}

/**
 *  \brief Configure the Pushbutton
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void _ConfigureButton( void )
{
    /* Configure pios as inputs. */
    PIO_Configure(&pinPB1, 1);
    /* Adjust pio debounce filter parameters, uses 10 Hz filter. */
    PIO_SetDebounceFilter(&pinPB1, 10);
    /* Initialize pios interrupt handlers, see PIO definition in board.h. */
    PIO_ConfigureIt(&pinPB1, _Button1_Handler); /* Interrupt on rising edge  */
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ(PIOB_IRQn);
    /* Enable PIO line interrupts. */
    PIO_EnableIt(&pinPB1);
}
/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief pio_alternate_function Application entry point.
 *
 *  Program the last page of internal flash with pattern
 *  0x00000001,0x00000002,...,0x80000000.If the ERASE is in
 *  PIO mode, putting jumper on ERASE has no effect.
 *  That is, this pin could be used as general purpose I/O
 *  line. Otherwise,the content of falsh will be erased to
 *  0xFFFFFFFF.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */

int main(void)
{
    uint32_t i;
    uint8_t error;
    volatile uint32_t *pLastPageData;
    uint32_t pBuffer[IFLASH_PAGE_SIZE / 4];
    uint32_t lastPageAddress;

    /*disable watchdog*/
    WDT_Disable( WDT ) ;

    /*configure BP2*/
    _ConfigureButton() ;

    printf("-- PIO Alternate Function Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


    printf("-I- Unlocking the whole flash\n\r");
    /* Unlock whole flash*/
    error = FLASHD_Unlock(IFLASH_ADDR, IFLASH_ADDR + IFLASH_SIZE, 0, 0);

    assert( !error ) ;

    /* Performs tests on last page.*/
    lastPageAddress = IFLASH_ADDR + IFLASH_SIZE - IFLASH_PAGE_SIZE;
    pLastPageData = (volatile uint32_t *) lastPageAddress;

    /* Write page with walking bit pattern (0x00000001, 0x00000002, ...)*/
    printf("-I- Writing last page with walking bit pattern\n\r");
    for (i=0; i < (IFLASH_PAGE_SIZE / 4); i++) {

        pBuffer[i] = 1 << (i % 32);
    }
    error = FLASHD_Write(lastPageAddress, pBuffer, IFLASH_PAGE_SIZE);
    assert( !error ) ;

    /* Check page contents*/
    printf("-I- Checking page contents ");
    for ( i=0 ; i < (IFLASH_PAGE_SIZE / 4) ; i++ )
    {
        printf(".");
        assert( pLastPageData[i] == (uint32_t)(1 << (i % 32)) ) ;
    }
    printf(" OK! \n\r");

    /*  Configure Erase pin NOT in Erase mode,*/
    printf("-I- Configure Erase pin in PIO mode\n\r");
    MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12;
    /*  Ask the user to put the erase jumper and remove it (200ms minimum)*/
    printf("-I- Please put the erase jumper and remove it at least 200ms later.\n\r");
    printf("Then press USRPB1 to go on!\n\r");
    /* Wait until BP2 pressed*/
    while(!button1Evt);
    button1Evt = false;
    /* Read the page again, it should not have changed*/
    printf("-I- Reading the page\n\r");
    for (i=0; i < (IFLASH_PAGE_SIZE / 4); i++)
    {
        printf(".");
        if( pLastPageData[i] != (uint32_t)(1 << (i % 32)))
        {
            printf("-F- Reading Error! \n\r");
            return 0;
        }
    }
    printf("Read OK! Erase is out of  function!\n\r");
    /* Configure Erase pin as Erase function*/
    printf("-I- Configure Erase pin as Erase function\n\r");
    MATRIX->CCFG_SYSIO &= (uint32_t)~(CCFG_SYSIO_SYSIO12);
    /*  Ask the user to put the erase jumper and remove it (200ms minimum)*/
    printf("-I- Please put the erase jumper and remove it at least 200ms later.\n\r");
    printf(" Then press USRPB1 to go on!\n\r");
    /* Wait until use confirmed*/
    while(!button1Evt);
    button1Evt = false;

    /* Check that data is 0xFF.*/
    for (i=0; i < (IFLASH_PAGE_SIZE / 4); i++)
    {
        printf(".");
        if( pLastPageData[i] != 0xFFFFFFFF )
        {
            printf("-F- Erasing is wrong!\n\r");
            return 0;
        }
    }
    printf( "Erase OK!\n\r" ) ;

    while ( 1 ) ;
}

