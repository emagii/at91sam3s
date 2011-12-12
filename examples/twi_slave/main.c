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
 * \page twi_slave TWI Slave Example
 *
 * \section Purpose
 *
 * This project demonstrates the TWI peripheral in slave mode. It mimics the
 * behavior of a serial memory, enabling the TWI master to read and write
 * data in its internal SRAM.
 *
 * \section Requirements
 *
 * In addition, another device will be needed to act as the TWI master. The
 * twi_eeprom example can be used for that, in which case a second kit
 * supported by that project is needed.
 * -# Connect TWD1 (SDA) for the 2 boards: PB4(pin 51) of chipset socket pin.
 * -# Connect TWCK1 (SCL) for the 2 boards: PB5(pin 76) of chipset socket pin.
 * -# Connect GND for the 2 boards: GND on extension header.
 * -# Add a pull up of 2,2KOhms on TWD and TWCK
 *
 * By default, PB4 and PB5 are SWJ-DP pins, it can be used as a pin for TWI1
 * peripheral in the end application. Mode selection between SWJ-DP (System IO mode)
 * and general IO mode is performed through the AHB Matrix CCFG_SYSIO.
 *
 * \section Description
 *
 * After launching the program, the device will act as a simple TWI-enabled
 * serial memory containing 512 bytes. This enables this project to be used
 * with the basic-twi-eeprom-project as the master.
 *
 * To write in the memory, the TWI master must address the device first, then
 * send two bytes containing the memory address to access. Additional bytes are
 * treated as the data to write.
 *
 * Reading is done in the same fashion, except that after receiving the memory
 * address, the device will start outputting data until a STOP condition is
 * sent by the master.
 * The default address for the TWI slave is fixed to Ox50. If the board has a TWI
 * component with this adress, you can change the define AT24C_ADDRESS in main.c
 * of twi_eeprom example, and the define SLAVE_ADDRESS in main.c of twi_slave
 * exmaple.
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
 * -# Upon startup, the application will output the following line on the DBGU:
 *    \code
 *     -- twi slave example xxx --
 *     -- SAM3S-EK
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Configuring the TWI in slave mode
 *    \endcode
 * -# For the TWI Master board, see the description inside his project
 * -# and the "Master" board will output:
 *    \code
 *     -- twi eeprom example xxx --
 *     -- SAM3S-EK
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Filling page #0 with zeroes ...
 *     -I- Filling page #1 with zeroes ...
 *     -I- Read/write on page #0 (polling mode)
 *     -I- 0 comparison error(s) found
 *     -I- Read/write on page #1 (IRQ mode)
 *     -I- Callback fired !
 *     -I- Callback fired !
 *     -I- 0 comparison error(s) found
 *    \endcode
 *
 * \section References
 * - twi_slave/main.c
 * - twi_eeprom/main.c
 * - twi.c
 * - twid.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the twi_slave example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local constants
 *----------------------------------------------------------------------------*/
/** Slave address of the device on the TWI bus. */
#define SLAVE_ADDRESS       0x50
/** Memory size in bytes (example AT24C512)*/
#define MEMORY_SIZE         512
/** Page size in bytes (example AT24C512)*/
#define PAGE_SIZE           128

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pio pins to configure. */
const Pin pins[] = {BOARD_PINS_TWI_SLAVE};

/** The slave device instance*/
typedef struct _SlaveDeviceDriver
{
    /** PageAddress of the slave device*/
    uint16_t pageAddress;
    /** Offset of the memory access*/
    uint16_t offsetMemory;
    /** Read address of the request*/
    uint8_t acquireAddress;
    /** Memory buffer*/
    uint8_t pMemory[MEMORY_SIZE];
} SlaveDeviceDriver ;

SlaveDeviceDriver EmulateDriver;

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/
void TWI1_IrqHandler( void )
{
    uint32_t status ;

    status = TWI_GetStatus( BOARD_BASE_TWI_SLAVE ) ;

    if ( ((status & TWI_SR_SVACC) == TWI_SR_SVACC) && (EmulateDriver.acquireAddress == 0) )
    {
        TWI_DisableIt( BOARD_BASE_TWI_SLAVE, TWI_IDR_SVACC ) ;
        TWI_EnableIt( BOARD_BASE_TWI_SLAVE, TWI_IER_RXRDY
                                   | TWI_IER_GACC
                                   | TWI_IER_NACK
                                   | TWI_IER_EOSACC
                                   | TWI_IER_SCL_WS ) ;
        EmulateDriver.acquireAddress++;
        EmulateDriver.pageAddress = 0;
        EmulateDriver.offsetMemory = 0;
    }

    if ( (status & TWI_SR_GACC) == TWI_SR_GACC )
    {
        printf( "General Call Treatment\n\r" ) ;
        printf( "not treated" ) ;
    }

    if ( ((status & TWI_SR_SVACC) == TWI_SR_SVACC ) && ((status & TWI_SR_GACC) == 0 ) && ((status & TWI_SR_RXRDY) == TWI_SR_RXRDY ) )
    {

        if ( EmulateDriver.acquireAddress == 1 )
        {
            /* Acquire LSB address */
            EmulateDriver.pageAddress = (TWI_ReadByte(BOARD_BASE_TWI_SLAVE) & 0xFF);
            EmulateDriver.acquireAddress++;
        }
        else
        {
            if ( EmulateDriver.acquireAddress == 2 )
            {
                /* Acquire MSB address */
                EmulateDriver.pageAddress |= (TWI_ReadByte(BOARD_BASE_TWI_SLAVE) & 0xFF)<<8;
                EmulateDriver.acquireAddress++;
            }
            else
            {
                /* Read one byte of data from master to slave device */
                EmulateDriver.pMemory[(PAGE_SIZE*EmulateDriver.pageAddress)+EmulateDriver.offsetMemory] = (TWI_ReadByte(BOARD_BASE_TWI_SLAVE) & 0xFF);
                EmulateDriver.offsetMemory++;
            }
        }
    }
    else
    {
        if( ((status & TWI_SR_TXRDY) == TWI_SR_TXRDY ) && ((status & TWI_SR_TXCOMP) == TWI_SR_TXCOMP ) && ((status & TWI_SR_EOSACC) == TWI_SR_EOSACC ) )
        {
            /* End of transfert, end of slave access */
            EmulateDriver.offsetMemory = 0;
            EmulateDriver.acquireAddress = 0;
            EmulateDriver.pageAddress = 0;
            TWI_EnableIt( BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC ) ;
            TWI_DisableIt( BOARD_BASE_TWI_SLAVE, TWI_IDR_RXRDY
                                        | TWI_IDR_GACC
                                        | TWI_IDR_NACK
                                        | TWI_IDR_EOSACC
                                        | TWI_IDR_SCL_WS );
        }
        else
        {
            if ( ((status & TWI_SR_SVACC) == TWI_SR_SVACC ) && ((status & TWI_SR_GACC) == 0 ) && ( EmulateDriver.acquireAddress == 3 )
                && ((status & TWI_SR_SVREAD) == TWI_SR_SVREAD ) && ((status & TWI_SR_NACK) == 0 ) )
            {
                /* Write one byte of data from slave to master device */
                TWI_WriteByte( BOARD_BASE_TWI_SLAVE, EmulateDriver.pMemory[(PAGE_SIZE*EmulateDriver.pageAddress)+EmulateDriver.offsetMemory] ) ;
                EmulateDriver.offsetMemory++;
            }
        }
    }
}

/*----------------------------------------------------------------------------
 * Default main function. Initializes the DBGU and writes a string on the DBGU.
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Select PB4 and PB5 as peripheral function */
    REG_MATRIX_SYSIO = CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5;

    PIO_Configure(pins, PIO_LISTSIZE(pins));
    printf( "-- twi slave exmaple %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Enable TWI peripheral clock */
    REG_PMC_PCER0 = 1 << BOARD_ID_TWI_SLAVE;

    for ( i=0 ; i < MEMORY_SIZE ; i++ )
    {
        EmulateDriver.pMemory[i] = 0;
    }
    EmulateDriver.offsetMemory = 0;
    EmulateDriver.acquireAddress = 0;
    EmulateDriver.pageAddress = 0;

    /* Configure TWI as slave */
    printf( "-I- Configuring the TWI in slave mode\n\r" ) ;
    TWI_ConfigureSlave( BOARD_BASE_TWI_SLAVE, SLAVE_ADDRESS ) ;

    /* Clear receipt buffer */
    TWI_ReadByte( BOARD_BASE_TWI_SLAVE ) ;

    TRACE_DEBUG( "TWI is in slave mode\n\r" ) ;

    /* Configure TWI interrupts */
    NVIC_DisableIRQ( TWI1_IRQn ) ;
    NVIC_ClearPendingIRQ( TWI1_IRQn ) ;
    NVIC_SetPriority( TWI1_IRQn, 0 ) ;
    NVIC_EnableIRQ( TWI1_IRQn ) ;

    TWI_EnableIt( BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC ) ;

    while ( 1 )
    {
    }
}


