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
 *  \page usart_synchronous  USART Synchronous Mode Example
 *
 *  \section Purpose
 *
 *  This example demonstrates the Synchronous  mode provided by the USART
 *  peripherals on SAM3S.
 *
 *  \section Requirements
 *
 *  This example can be used on sam3s evaluation kit and requires 2 boards to
 *   be connected directly through populated usart1 pins.
 *  - <b> master   --   slave</b>
 *   - TXD(PA22)   --  RXD(PA21)
 *   -  RXD(PA21)  --  TXD(PA22)
 *   -  SCK(PA23)  --  SCK(PA23)
 *
 *  \section Description
 *
 *  This application gives an example of how to use USART in synchronous mode.
 *  Synchronous operations provide a high speed transfer capability. The
 *  transfer under this mode needs a pair of master and slave, which is
 *  determined by which one offers the clock source.
 *
 *  The example initialized USART1 as master by default. To enable the
 *  communication between each side of the connection,the user should change
 *  the mode of another side to slave through user interface. If well configured,
 *  transfer could be started by typing 'r' and  'w' from terminal application.
 *  This example also leaves the interface to select the clock frequency.
 *
 *  The meaning of each input character could be found in items of the main menu.
 *
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">SAM-BA User Guide</a>,
 *     the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
 *     application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
 *     depending on your chosen solution.
 *  -# Connect a serial cable to the UART port on the evaluation kit.
 *  -# On the computer, open and configure a terminal application (e.g.
 *     HyperTerminal on Microsoft Windows) with these settings:
 *        - 115200 bauds
 *        - 8 data bits
 *        - No parity
 *        - 1 stop bit
 *        - No flow control
 *  -# Start the application. The following traces shall appear on the terminal:
 *     \code
 *     -- USART Synchronous Mode Example  xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -- Menu Choices for this example --
 *     -- [0-3]:Select clock frequency of master --
 *     -- i: Display configuration info
 *     -- w: Write data block .--
 *     -- r: Read data block.--
 *     -- s: Switch between master and slave mode.--
 *     -- m: Display this menu again.--
 *     --USART1 in MASTER mode--
 *
 *     \endcode
 *  -# The main menu will guide the user to configure the device and conduct
 *     operations.
 *
 *  \section References
 *  - usart_synchronous/main.c
 *  - pio.h
 *  - usart.h
 */

/** \file
 *
 *  This file contains all the specific code for the usart_synchronous example.
 *
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *         Local definition
 *----------------------------------------------------------------------------*/


/** size of the receive buffer used by the PDC, in bytes.*/
#define BUFFER_SIZE         2000


/** USART1 synchronous master*/
#define SYNC_MASTER    1
/** USART1 synchronous slave*/
#define SYNC_SLAVE    0

/** USART1 is reading*/
#define STATE_READ    0

/** USART1 is writing*/
#define STATE_WRITE   1

#define FREQ_OPTIONS_NUM 4

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** Pins to configure for the application.*/
const Pin pins[] = { BOARD_PIN_USART_RXD,
                     BOARD_PIN_USART_TXD,
                     PIN_USART1_SCK,
                     };

/** Transmit buffer. */
char
Buffer[BUFFER_SIZE]=

 "DESCRIPTION of this example: \n\r \
 **************************************************************************\n\r\
 *  This application gives an example of how to use USART in synchronous mode.\n\r\
 *  Synchronous operations provide a high speed transfer capability. The\n\r\
 *  transfer under this mode needs a pair of master and slave, which is\n\r\
 *  determined by which one offers the clock source.\n\r\
 *  \n\r\
 *  The example initialized USART1 as master by default. To enable the\n\r\
 *  communication between each side of the connection. The user should change\n\r\
 *  the mode of another side to slave through user interface. If well configured,\n\r\
 *  transfer could be started by typing 'r' and  'w' from terminal application.\n\r\
 *  This example also leaves the interface to select the clock frequency.\n\r\
 *  \n\r\
 *  The meaning of each input character could be found in items of the main menu.\n\r\
 *  \n\r\
 **************************************************************************\n\r\
 END of DESCRIPTION \n\r\
 ";


/** buffer for receiving */
char pRecvBufferUSART1[BUFFER_SIZE]= { 0 };

/** reception done*/
volatile bool recvDone = false;
/** sending done*/
volatile bool sentDone = false;

/** mode for usart1 and spi ,0 means usart1 as master 1 for
 another state*/
uint8_t transfer_mode = SYNC_MASTER;

/** state of reading or writing*/
uint8_t state = STATE_WRITE;

/** clock frequency*/
uint32_t frequency[FREQ_OPTIONS_NUM]={1000000UL,4000000UL,10000000UL,16000000UL};

/** present freqency index in list frequency[]*/
uint8_t freq_index = 0;
/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/* \brief USART1 IRQ handler
 *
 * Interrupt handler for USART. After reception is done,set recvDone to true,
 * and if transmission done, set sentDone to true.
 *
 *----------------------------------------------------------------------------*/
void USART1_IrqHandler(void)
{
    uint32_t status;

    /*  Read USART status */
    status = BOARD_USART_BASE->US_CSR;
    /* Receive buffer is full */
    if ((status & US_CSR_RXBUFF) == US_CSR_RXBUFF && state == STATE_READ) {
        recvDone = true;
        USART_DisableIt(USART1, US_IDR_RXBUFF);
    }
    if ((status & US_CSR_TXBUFE) == US_CSR_TXBUFE && state == STATE_WRITE){
        sentDone = true;
        USART_DisableIt(USART1, US_IDR_TXBUFE);
    }

}


/**
 * \brief Configures USART in synchronous mode,8N1
 * \param mode 1 for  master, 0 for slave
 */
static void _ConfigureUsart( uint8_t isMaster, uint32_t freq )
{
    uint32_t mode = US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK
                    | US_MR_CHMODE_NORMAL | US_MR_CLKO
                    | US_MR_SYNC | US_MR_MSBF
                    | US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT| US_MR_PAR_NO;

    if ( !isMaster )
    {
        mode = US_MR_USART_MODE_NORMAL | US_MR_USCLKS_SCK
        | US_MR_CHMODE_NORMAL
        | US_MR_SYNC | US_MR_MSBF
        | US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT| US_MR_PAR_NO;
    }
    /* Enable the peripheral clock in the PMC */
    PMC_EnablePeripheral( BOARD_ID_USART ) ;

    /* Configure the USART in the desired mode @USART_SPI_CLK bauds*/
    USART_Configure( BOARD_USART_BASE, mode, freq, BOARD_MCK ) ;

    /* enable USART1 interrupt */
    NVIC_EnableIRQ( USART1_IRQn ) ;

    /* Enable receiver & transmitter */
    USART_SetTransmitterEnabled( BOARD_USART_BASE, 1 ) ;
    USART_SetReceiverEnabled( BOARD_USART_BASE, 1 ) ;
}
/**
 * \brief Display main menu.
 */
static void _DisplayMainmenu( void )
{
    printf("-- Menu Choices for this example --\n\r");
    printf("-- [0-3]:Select clock frequency of master --\n\r");
    printf("-- i: Display configuration info\n\r");
    printf("-- w: Write data block .--\n\r");
    printf("-- r: Read data block.--\n\r");
    printf("-- s: Switch between master and slave mode.--\n\r");
    printf("-- m: Display this menu again.--\n\r");
}

/**
 * \brief Dump buffer to uart
 *
 */
static void _DumpInfo( char *buf, uint32_t size )
{
    uint32_t i = 0 ;

    while ( (i < size) && (buf[i] != 0) )
    {
        printf( "%c", buf[i++] ) ;
    }
}

/*------------------------------------------------------------------------------
 *         Global functions
 *------------------------------------------------------------------------------*/


/**
 * \brief Application entry point.
 *
 * Configures USART1 in synchronous master/slave mode start a transmission
 * between two boards.
 * \return Unused.
 */
extern int main( void )
{
    char c;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /*  Configure pins */
    PIO_Configure( pins, PIO_LISTSIZE( pins ) ) ;

    /* Example information log */
    printf( "-- USART Synchronous Mode Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* display main menu*/
    _DisplayMainmenu() ;

    /* configure USART1 in  Master and SPI in slave mode*/
    _ConfigureUsart( SYNC_MASTER, frequency[freq_index] ) ;

    transfer_mode = SYNC_MASTER ;

    state = STATE_WRITE ;

    printf( "--USART1 in MASTER mode--\n\r" ) ;

    while ( 1 )
    {
        c = UART_GetChar() ;

        switch ( c )
        {
            case '0':
            case '1':
            case '2':
            case '3':
                freq_index = c - '0';
                printf("-- The clock frequency is: %u\n\r", (unsigned int)frequency[freq_index] ) ;
                _ConfigureUsart( SYNC_MASTER, frequency[freq_index] ) ;
            break ;

            case 'i':
            case 'I':
            if ( transfer_mode == SYNC_MASTER )
            {
                printf( "-- USART1 is MASTER at %u Hz.\n\r", (unsigned int)frequency[freq_index] ) ;
            }
            else
            {
                printf( "-- USART1 is SLAVE \n\r" ) ;
            }
            break;

            case 's':
            case 'S':
            if ( transfer_mode == SYNC_MASTER )
            {
                transfer_mode = SYNC_SLAVE ;
                _ConfigureUsart( SYNC_SLAVE, frequency[freq_index] ) ;
                printf( "--USART1 in SLAVE mode--\n\r" ) ;
            }
            else
            {
                if ( transfer_mode == SYNC_SLAVE )
                {
                    transfer_mode = SYNC_MASTER;
                    _ConfigureUsart(SYNC_MASTER,frequency[freq_index]);
                    printf("--USART1 in MASTER mode--\n\r");
                }
            }
            break;

            case 'w':
            case 'W':
                state = STATE_WRITE;
                USART_WriteBuffer(BOARD_USART_BASE, Buffer, BUFFER_SIZE);
                USART_EnableIt(BOARD_USART_BASE,US_IER_TXBUFE);
                while(!sentDone);
                if(sentDone){
                    printf(" -- %s sent done\n\r", state? "MASTER":"SLAVE" );

                }
                break;
            case 'r':
            case 'R':
               state = STATE_READ;
            if(transfer_mode == SYNC_MASTER){
                printf("----USART1 MASTER Read----\n\r");

            }else{
                printf("----USART1 SLAVE Read----\n\r");
            }
            USART_ReadBuffer(BOARD_USART_BASE, pRecvBufferUSART1, BUFFER_SIZE);
            USART_EnableIt(BOARD_USART_BASE,US_IER_RXBUFF);
            while(!recvDone);
            if(recvDone) {

                if(strncmp(pRecvBufferUSART1,Buffer,BUFFER_SIZE)){
                    printf(" -F-: Failed!\n\r");
                }else{
                    /* successfully received*/
                    _DumpInfo(pRecvBufferUSART1,BUFFER_SIZE);
                }
                printf("----END of read----\n\r");
                memset(pRecvBufferUSART1,0,sizeof(pRecvBufferUSART1));
                recvDone = false;

            }
            break;


            case 'm':
            case 'M':
                _DisplayMainmenu() ;
            break;

        }

    }
}

