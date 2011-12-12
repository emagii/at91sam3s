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
 * \page usart_irda USART IRDA Example
 *
 * \section Purpose
 *
 * This example demonstrates the IrDA (Infrared Data Association) on sam3s.
 *
 * \section Requirements
 *
 * This example can be used on SAM3S-EK with external IRDA transceiver
 * component. Connect the board and external component with following paired
 * pins:
 *  - <b> SAM3S-EK  --  IRDA transceiver </b>
 *   - 3V3    --     VCC
 *   - TXD1(PA22) -- TXD
 *   - RXD1(PA21) -- RXD
 *   - PA11       -- SD
 *   - GND        -- GND
 *
 * \section Description
 *
 * The provided program uses the USART in IrDA mode for transmit and receive
 * serveral octets. This example can be used with two sam3s-ek board.
 * The program receives or transmits a series of octets according to its state
 * either receiver or  transmitter.
 *
 * \section Note
 * To receive IrDA signals, the following needs to be done:
 * <ul>
 * <li> Disable TX and Enable RX </li>
 * <li> Configure the TXD pin as PIO and set it as an output at 0 (to avoid LED emission). Disable
 * the internal pull-up (better for power consumption).</li>
 * <li> Receive data </li>
 * </ul>
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">SAM-BA User Guide</a>,
 *    the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
 *    application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# Download the program inside the second evaluation board.
 * -# Connect a serial cable to the UART port on the evaluation kit. It is
 *    labeled "UART" on sam3s-ek.
 * -# On the computer, open and configure a terminal application (e.g.
 *    HyperTerminal on Microsoft Windows) with these settings:
 *       - 115200 bauds
 *       - 8 data bits
 *       - No parity
 *       - 1 stop bit
 *       - No flow control
 * -# Start the application.
 * -# The following traces shall appear on the terminal:
 *    \code
 *     -- USART IRDA Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Press t to transmit, press r to receive...
 *    \endcode
 * -# Enable one board as transmitter and another as receiver to start the
 *    communication. If succeed, the side of receiver would output the received
 *    data.
 *
 *
 *  \section References
 *  - usart_irda/main.c
 *  - pio.h
 *  - usart.h
 */

/** \file
 *
 *  This file contains all the specific code for the usart_irda example.
 *
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/** buffer size for PDC transfer*/
#define BUFFER_SIZE  10
/** state TRANSMITTING*/
#define STATE_TRANSMIT   0
/** state RECEIVING*/
#define STATE_RECEIVE    1
/** state IDLE */
#define STATE_IDLE       3

/** IRDA SD pin*/
#define PIN_IRDA_SD {0x1 << 11, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

/** RS232 shutdown pin, disable the RS232 on the ek*/
#define PIN_RS232_SD {0x1 << 23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

/** To receive IrDA signals, configure the TXD pin as PIO **/
#define PIN_USART1_TXD_RECEIVE_MODE    {PIO_PA22A_TXD1, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}

/*------------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** List of pins to configure for the application */
static const Pin pins[] = {PIN_USART1_RXD, PIN_IRDA_SD, PIN_RS232_SD};

/** IrDA TXD pin in transmit mode */
static const Pin pinTxdTransmitMode  = PIN_USART1_TXD;

/** IrDA TXD pin in receive mode */
static const Pin pinTxdReceiveMode  = PIN_USART1_TXD_RECEIVE_MODE;

/** State of USART1*/
uint8_t state = STATE_IDLE;

/** Receiving Done status */
volatile bool recvDone = false;

/** Transmitting Done status*/
volatile bool sentDone = false;

/** data buffer for receiving*/
uint8_t recvData[BUFFER_SIZE] = {0x0};

/** data buffer for transmitting*/
uint8_t sentData[BUFFER_SIZE] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89};


/*------------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
  * \brief USART1 IRQ Handler. Handling RXBUFF and TXBUFE status.
  */
void USART1_IrqHandler(void)
{
    uint32_t status;

    status = USART_GetStatus(USART1);
    /* receiving done */
    if( (status & US_CSR_RXBUFF) == US_CSR_RXBUFF && (state == STATE_RECEIVE)){
        recvDone = true;
        USART_DisableIt(USART1,US_IDR_RXBUFF);
    }
    /* transmitting done*/
    if( (status & US_CSR_TXBUFE) == US_CSR_TXBUFE && (state == STATE_TRANSMIT)){
        sentDone = true;
        USART_DisableIt(USART1,US_IDR_TXBUFE);
    }
}

/**
 * \brief Disable receiver and Enable transmitter
 */
static void _FunTransmitter( void )
{
    /* Configure the TXD pin as peripheral. */
    PIO_Configure( &pinTxdTransmitMode, 1 ) ;

    /* Disable Receiver */
    USART_SetReceiverEnabled( USART1, 0 ) ;

    /* Enable transmitter */
    USART_SetTransmitterEnabled( USART1, 1 ) ;
}

/**
 * \brief Disable transmitter and Enable receiver.
 */
static void _FunReceiver( void )
{
    /* Disable Transmitter */
    USART_SetTransmitterEnabled( USART1, 0 ) ;

    /* Configure the TXD pin as PIO. */
    PIO_Configure( &pinTxdReceiveMode, 1 ) ;

    /* set it as an output at 0 (to avoid LED emission).  Disable the internal pull-up (better for power consumption). */
    PIO_Clear( &pinTxdReceiveMode ) ;

    /* Enable receiver */
    USART_SetReceiverEnabled( USART1, 1 ) ;

    /* read dummy , make sure that there are no characters in US_THR! */
    if ( ( USART1->US_CSR & US_CSR_RXRDY) == US_CSR_RXRDY )
    {
        USART1->US_RHR ;
    }
}

/**
 * \brief Dump received buffer to uart
 * \param buf pointer to received buffer
 * \param size the size of the buffer
 */
static void _DumpRecvBuf( uint8_t *buf, uint8_t size )
{
    uint8_t i = 0 ;

    for ( i = 0 ; i < size ; i++ )
    {
        printf( "0x%x\t", buf[i] ) ;
    }
    printf( "\n\r" ) ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point.
 *
 * Initializes the IrDA and starts the main loop
 *
 * \return Unused (ANSI-C compatibility)
 */
extern int main( void )
{
    char c ;

    /* Disable watchdog*/
    WDT_Disable( WDT ) ;

    /* Output Example log info*/
    printf( "-- USART IRDA Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Enable peripheral*/
    PMC_EnablePeripheral( ID_USART1 ) ;

    /* Configure pins for IRDA transceiver and Enable it*/
    PIO_Configure( pins, PIO_LISTSIZE( pins ) ) ;

    /* Disable shotdown mode.*/
    PIO_Clear( &(pins[2]) ) ;

    /* Configure USART0 in IRDA mode*/
    USART_Configure( USART1, US_MR_USART_MODE_IRDA, 9600, BOARD_MCK ) ;

    /* Set demodulator filter*/
    USART_SetIrdaFilter( USART1, 4 ) ;

    /* Enable USART1 interrupt request*/
    NVIC_EnableIRQ( USART1_IRQn ) ;

    printf( "-I- Press t to transmit, press r to receive...\n\r" ) ;

    /* main loop*/
    while ( 1 )
    {
        c = UART_GetChar() ;

        switch ( c )
        {
            case 't':
            case 'T':
                state = STATE_TRANSMIT ;
                /* Enable transmitter*/
                _FunTransmitter() ;
                USART_WriteBuffer( USART1, sentData, BUFFER_SIZE ) ;
                USART_EnableIt( USART1, US_IER_TXBUFE ) ;
                while( !sentDone ) ;

                printf( "-I- Sent Done!\n\r" ) ;
                sentDone = false ;
                state = STATE_IDLE ;
                printf( "-I- Press t to transmit, press r to receive...\n\r" ) ;
            break ;

            case 'r':
            case 'R':
                state = STATE_RECEIVE ;

                /* Enable receiver*/
                printf( "-I- IrDA receive mode\n\r" ) ;
                _FunReceiver() ;
                USART_ReadBuffer( USART1, recvData, BUFFER_SIZE ) ;
                USART_EnableIt( USART1, US_IER_RXBUFF ) ;

                while ( !recvDone ) ;

                printf( "-I- Received Done! \n\r" ) ;
                _DumpRecvBuf( recvData, BUFFER_SIZE ) ;
                memset( recvData, 0, BUFFER_SIZE ) ;
                state = STATE_IDLE ;
                recvDone = false ;
                printf( "-I- Press t to transmit, press r to receive...\n\r" ) ;
            break ;

            default:
            break;
        }
    }
}


