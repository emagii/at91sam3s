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
 * \page twi_eeprom TWI EEPROM Example
 *
 * \section Purpose
 *
 * This basic example program demonstrates how to use the TWI peripheral
 * to access an external serial EEPROM chip.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 * Connect the TWI1 peripheral to eeprom device:
 *        - <b>SAM3S-- EEPROM</b>
 *        - VCC -- VCC
 *        - GND -- GND
 *        - TWCK1(PB5) -- SCK
 *        - TWD1(PB4)  -- SCL
 *
 *  All these pins can be easily connecting with jumpers.
 *
 * When if works for twi_slave_example as a TWI master.
 * -# Connect TWD1 (SDA) for the 2 boards: PB4(pin 51).
 * -# Connect TWCK1 (SCL) for the 2 boards: PB5(pin 76).
 * -# Connect GND for the 2 boards: GND on extension header.
 * -# Add a pull up of 2,2KOhms on TWD and TWCK
 *
 * By default, PB4 and PB5 are SWJ-DP pins, it can be used as a pin for TWI1
 * peripheral in the end application. Mode selection between SWJ-DP (System IO mode)
 * and general IO mode is performed through the AHB Matrix CCFG_SYSIO.
 *
 * \section Description
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li>Configure TWI pins.</li>
 * <li>Enable TWI peripheral clock.</li>
 * <li>Configure TWI clock.</li>
 * <li>Initialize TWI as twi master.</li>
 * <li>TWI interrupt handler.</li>
 * <li>The main function, which implements the program behavior.</li>
 * <ol>
 * <li>Sets the first and second page of the EEPROM to all zeroes.</li>
 * <li>Writes pattern in page 0. </li>
 * <li>Reads back data in page 0 and compare with original pattern (polling). </li>
 * <li>Writes pattern in page 1. </li>
 * <li>Reads back data in page 1 and compare with original pattern (interrupts). </li>
 * </ol>
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
 *     -- TWI EEPROM Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The following traces detail operations on the EEPROM, displaying success
 *    or error messages depending on the results of the commands.
 *
 * \section References
 * - twi_eeprom/main.c
 * - twi.c
 * - twid.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the twi eeprom example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK            400000

/** Slave address of AT24C chips.*/
#define AT24C_ADDRESS   0x50

/** Page size of an AT24C512 chip (in bytes)*/
#define PAGE_SIZE       64


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pio pins to configure. */
static const Pin pins[] = {BOARD_PINS_TWI_EEPROM};

/** TWI driver instance.*/
static Twid twid;

/** Page buffer.*/
static uint8_t pData[PAGE_SIZE];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWI1_IrqHandler( void )
{
    TWID_Handler( &twid ) ;
}

/**
 * \brief Dummy callback, to test asynchronous transfer modes.
 */
static void TestCallback( void )
{
    printf( "-I- Callback fired !\n\r" ) ;
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for TWI eeprom example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
    volatile uint32_t i;
    Async async;
    uint32_t numErrors;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Select PB4 and PB5 as peripheral function */
    REG_MATRIX_SYSIO = CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5;

    /* Output example information */
    printf("-- TWI EEPROM Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure TWI pins. */
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    /* Enable TWI peripheral clock */
    PMC->PMC_PCER0 = 1 << BOARD_ID_TWI_EEPROM;

    /* Configure TWI */
    TWI_ConfigureMaster(BOARD_BASE_TWI_EEPROM, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, BOARD_BASE_TWI_EEPROM);

    /* Configure TWI interrupts */
    NVIC_DisableIRQ(TWI1_IRQn);
    NVIC_ClearPendingIRQ(TWI1_IRQn);
    NVIC_SetPriority(TWI1_IRQn, 0);
    NVIC_EnableIRQ(TWI1_IRQn);


    /* Erase page #0 and #1 */
    memset(pData, 0, PAGE_SIZE);
    printf("-I- Filling page #0 with zeroes ...\n\r");
    TWID_Write(&twid, AT24C_ADDRESS, 0x0000, 2, pData, PAGE_SIZE, 0);

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);

    printf("-I- Filling page #1 with zeroes ...\n\r");
    TWID_Write(&twid, AT24C_ADDRESS, 0x0100, 2, pData, PAGE_SIZE, 0);

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);

    /* Synchronous operation */
    printf("-I- Read/write on page #0 (polling mode)\n\r");

    /* Write checkerboard pattern in first page */
    for ( i=0 ; i < PAGE_SIZE ; i++ )
    {
        /* Even*/
        if ( (i & 1) == 0 )
        {
            pData[i] = 0xA5;
        }
        /* Odd */
        else
        {
            pData[i] = 0x5A;
        }
    }
    TWID_Write(&twid, AT24C_ADDRESS, 0x0000, 2, pData, PAGE_SIZE, 0);

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);

    /* Read back data */
    memset(pData, 0, PAGE_SIZE);
    TWID_Read(&twid, AT24C_ADDRESS, 0x0000, 2, pData, PAGE_SIZE, 0);

    /* Compare */
    numErrors = 0;
    for (i=0; i < PAGE_SIZE; i++)
    {
        /* Even */
        if ( ((i & 1) == 0) && (pData[i] != 0xA5) )
        {
            printf( "-E- Data mismatch at offset #%d: expected 0xA5, read 0x%02X\n\r", (unsigned int)i, pData[i] ) ;
            numErrors++ ;
        }
        /* Odd */
        else
        {
            if ( ((i & 1) == 1) && (pData[i] != 0x5A) )
            {
                printf( "-E- Data mismatch at offset #%d: expected 0x5A, read 0x%02X\n\r", (unsigned int)i, pData[i] ) ;
                numErrors++;
            }
        }
    }
    printf("-I- %ld comparison error(s) found\n\r", numErrors);

    /* Asynchronous operation */
    printf("-I- Read/write on page #1 (IRQ mode)\n\r");

    /* Write checkerboard pattern in first page */
    for ( i=0 ; i < PAGE_SIZE ; i++ )
    {
        /* Even */
        if ( (i & 1) == 0 )
        {
            pData[i] = 0xA5;
        }
        /* Odd */
        else
        {
            pData[i] = 0x5A;
        }
    }

    memset(&async, 0, sizeof(async));
    async.callback = (void *) TestCallback;
    TWID_Write(&twid, AT24C_ADDRESS, 0x0100, 2, pData, PAGE_SIZE, &async);
    while (!ASYNC_IsFinished(&async));

    /* Wait at least 10 ms */
    for ( i=0 ; i < 1000000 ; i++ ) ;

    /* Read back data */
    memset(pData, 0, PAGE_SIZE);
    memset(&async, 0, sizeof(async));
    async.callback = (void *) TestCallback;
    TWID_Read(&twid, AT24C_ADDRESS, 0x0100, 2, pData, PAGE_SIZE, &async);
    while ( !ASYNC_IsFinished( &async ) ) ;

    /* Compare */
    numErrors = 0 ;
    for ( i=0 ; i < PAGE_SIZE ; i++ )
    {
        /* Even */
        if ( ((i & 1) == 0) && (pData[i] != 0xA5) )
        {
            printf( "-E- Data mismatch at offset #%d : expected 0xA5, read 0x%02X\n\r", (unsigned int)i, pData[i] ) ;
            numErrors++ ;
        }
        /* Odd */
        else
        {
            if ( ((i & 1) == 1) && (pData[i] != 0x5A) )
            {
                printf( "-E- Data mismatch at offset #%d : expected 0x5A, read 0x%02X\n\r", (unsigned int)i, pData[i] ) ;
                numErrors++ ;
            }
        }
    }
    printf( "-I- %ld comparison error(s) found\n\r", numErrors ) ;

    return 0 ;
}
