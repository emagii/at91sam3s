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
 *  \page usart_rs485 USART RS485 Example
 *
 *  \section Purpose
 *
 *  The USART RS485 Example demonstrates how to use USART in RS485 mode.
 *
 *  \section Requirements
 *
 *  This package can only be used with sam3s-ek. Before running,
 *  make sure to connect two boards with RS485 lines.The connector J4 is for
 *  this purpose. Match each paired pins of two boards respectively with A to A,
 *  B to B and FGND to FGND(the central pin of J4).
 *
 *  The R25 (0R) is not populated for conflicting between RS232 and RS485. Make
 *  sure to populate it properly before running this example.
 *
 *  \section Description
 *
 *  This example connects two boards through RS485 interface. One board acts
 *  as the transmitter and the other one as the receiver. It is determined by
 *  the sequence the two applications started.
 *
 *
 *  In any case, the application sends a sync character at running to seek an
 *  receiver.If the acknowledgment is received in a short time, it will act
 *  as the transmitter and then send a full frame text to the receiver.
 *
 *
 *  The earlier started board will act automatically as the receiver due to no
 *  acknowledgment received. The receiver will wait until sync character is
 *  received. Then it sends the acknowledgment and waits for the full frame
 *  sent by the transmitter. At the end of reception,it prints out message
 *  through UART interface to assert that the whole process succeeds.
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
 *  -# On the computer, open and configure a terminal application for each board
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start application from two boards in sequence. Make sure the second board
 *     should NOT be started unless the first board had run to wait for the
 *     synchronizing character. The output message in later section would
 *     describe this.
 *
 *  -# In the terminal  window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- USART RS485 Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# The consequent messages will indicate the boards' behavior.
 *
 *     -  The earlier started board would output the message below to indicate it
 *     was waiting for a synchronization character:
 *     \code
 *     -I- Receiving sync character.
 *     \endcode
 *     -  If received a sync character and prepared to receive a frame,it
 *     printed out the message below:
 *     \code
 *     -I- Start receiving!
 *     \endcode
 *     -  After successfully received a frame, the board output the following
 *     message to indicate that the whole process succeeded.
 *     \code
 *     -I- Received successfully!
 *     \endcode
 *     -  The later started one would act as transmitter,and if received an
 *     acknowledgment character successfully, it output the following message
 *     and started transmitting:
 *     \code
 *     -I- Start transmitting!
 *     \endcode
 *
 *  \section References
 *  - usart_rs485/main.c
 *  - pio.h
 *  - usart.h
 */

/** \file
 *
 *  This file contains all the specific code for the usart_rs485 example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE         1000

/** Size of the pdc buffer. */
#define PDC_BUF_SIZE    (BUFFER_SIZE)

/** Acknowledge time out*/
#define TIMEOUT             (1000)

/** Character to synchronize with the other end */
#define SYNC_CHAR             0x11

/** Character to acknowledge receipt of the sync char */
#define ACK_CHAR             0x13

/** baud rate*/
#define BAUDRATE_RS485      256000

/** state of the usart*/
typedef enum _tUSART_STATE
{
    INITIALIZED,
    TRANSMITTING,
    RECEIVING,
    RECEIVED,
    TRANSMITTED
} tUSART_STATE ;

/* SD/EN for RS232, shutdown the RS232 before running*/
#define PIN_USART1_SD         {0x1 << 23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
/* RO */
#define PIN_USART1_RXD_IOR    {0x1 << 21, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/* DI */
#define PIN_USART1_TXD_IOT    {0x1 << 22, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
/*\RE */
#define PIN_USART1_CTS_IOR    {0x1 << 25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/* DE */
#define PIN_USART1_RTS_IOR    {0x1 << 24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** global usart state*/
volatile tUSART_STATE state = INITIALIZED;

/** Pins to configure for usart peripheral. */
const Pin pins[] = { PIN_USART1_RXD, PIN_USART1_TXD, PIN_USART1_RTS } ;

/** Pins to configure for IO mode. */
const Pin pinsR[] = { PIN_USART1_TXD_IOT, PIN_USART1_RTS_IOR, PIN_USART1_CTS_IOR, PIN_USART1_RXD_IOR,PIN_USART1_SD } ;

/** temporary pointer for transmitting */
uint8_t* pBufTrans ;

/** temporary pointer for receiving */
uint8_t* pBufRecv ;

/** Transmit buffer. Pure ASCII text */
uint8_t pBufferTransmit[BUFFER_SIZE] =
                                " \
/* ----------------------------------------------------------------------------\n\
 *         ATMEL Microcontroller Software Support \n\
 * ----------------------------------------------------------------------------\n\
 * Copyright (c) 2009, Atmel Corporation\n\
 *\n\
 * All rights reserved.\n\
 *\n\
 * Redistribution and use in source and binary forms, with or without\n\
 * modification, are permitted provided that the following conditions are met:\n\
 *\n\
 * - Redistributions of source code must retain the above copyright notice,\n\
 * this list of conditions and the disclaimer below.\n\
 *\n\
 * Atmel's name may not be used to endorse or promote products derived from\n\
 * this software without specific prior written permission.\n\
 * \n\
 * ----------------------------------------------------------------------------\n\
 */";

/** Receive buffer */
uint8_t pBufferReceive[BUFFER_SIZE];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Accumulate time_stamp.
 */
void SysTick_Handler( void )
{
    TimeTick_Increment() ;
}

/**
 *  \brief Handler for USART1 interrupt.
 *
 *
 */
void USART1_IrqHandler( void )
{
    uint32_t status;
    status = USART_GetStatus( USART1 ) ;

    /* receiving interrupt */
    if ( (status & US_CSR_ENDRX) == US_CSR_ENDRX && state == RECEIVING )
    {
        /* partly received */
        if ( (uint32_t) pBufRecv < (uint32_t) (pBufferReceive + BUFFER_SIZE) )
        {
            if ( (status & US_CSR_ENDRX) == US_CSR_ENDRX )
            {
                pBufRecv += PDC_BUF_SIZE ;
            }

            /* prepare for next reception */
            USART_ReadBuffer( USART1, pBufRecv, PDC_BUF_SIZE ) ;
            USART_EnableIt( USART1, US_IER_ENDRX ) ;
        }
        else
        {
            /* indicate receiving finished */
            state = RECEIVED ;
            USART_DisableIt( USART1, US_IDR_ENDRX ) ;
        }
    }
    /* transmitting interrupt */
    else
    {
        if ((status & US_CSR_ENDTX) == US_CSR_ENDTX && state == TRANSMITTING)
        {
            /* transmit continuously */
            if (((uint32_t) pBufTrans) < (uint32_t) (pBufferTransmit + BUFFER_SIZE))
            {
                pBufTrans += PDC_BUF_SIZE;
                USART_WriteBuffer(USART1, pBufTrans, PDC_BUF_SIZE);
                USART_EnableIt(USART1, US_IER_ENDTX);

            }
            else
            {
                state = TRANSMITTED;
                USART_DisableIt(USART1, US_IDR_ENDTX);
            }
        }
    }

}

/**
 *  \brief USARM RS485 mode configuration
 *
 *   Configures USART in rs485 mode, asynchronous, 8 bits, 1 stop
 *  bit, no parity, 256000 bauds and enables its transmitter and receiver.
 */
static void _ConfigureUsart( void )
{
    uint32_t mode = US_MR_USART_MODE_RS485 | US_MR_USCLKS_MCK
                    | US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT
                    | US_MR_CHMODE_NORMAL;

    /* Enable the peripheral clock in the PMC*/
    PMC_EnablePeripheral( ID_USART1 ) ;

    /* Configure USART1 in the desired mode @256000 bauds*/
    USART_Configure( USART1, mode, BAUDRATE_RS485, BOARD_MCK ) ;

    /* Disable all the interrupts*/
    USART1->US_IDR = 0xFFFFFFFF ;

    /* Enable USART1 interrupt*/
    NVIC_EnableIRQ( USART1_IRQn ) ;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief usart_rs485 Application entry point.
 *
 *  Configures USART1 in RS485 mode,
 *  If the application started earlier,it act as
 *  a receiver. Otherwise, it should be a transmitter
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
extern int main( void )
{
    uint8_t cSync = SYNC_CHAR;
    uint32_t time_elapsed = 0;
    volatile uint32_t status = 0x0;
    int i ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- USART RS485 Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* initialize the temporary pointer for future use,which will
     also be modified in USART interrupt handler*/
    pBufTrans = pBufferTransmit;
    pBufRecv = pBufferReceive;

    /* reset the controlling pins*/
    PIO_Configure( pinsR, PIO_LISTSIZE( pinsR ) ) ;
    /* Configure pins to USART peripheral*/
    PIO_Configure( pins, PIO_LISTSIZE( pins ) ) ;

    /* Configure USART*/
    _ConfigureUsart() ;

    /* 1ms tick*/
    TimeTick_Configure( BOARD_MCK ) ;

    /* initialize receiving buffer to distinguish with the sent frame*/
    memset( pBufferReceive, 0x0, sizeof( pBufferReceive ) ) ;

    /* Read status register */
    USART_GetStatus( USART1 ) ;

    /* Enable transmitter here*/
    USART_SetTransmitterEnabled( USART1, 1 ) ;
    /* disable receiver first,to avoid receiving characters sent by itself*/
    /* It's necessary for half duplex RS485*/
    USART_SetReceiverEnabled( USART1, 0 ) ;

    /* send a sync character XON (0x11);*/
    USART_WriteBuffer( USART1, &cSync, 1 ) ;

    /* delay until the line is clear,an estimated time used*/
    Wait( 50 ) ;

    /* read the acknowledgment*/
    USART_ReadBuffer( USART1, &cSync, 1 ) ;

    /* status register*/
    USART_GetStatus( USART1 ) ;

    /* then enable receiver*/
    USART_SetReceiverEnabled(USART1,1);

    /* wait until time out or acknowledgment is received*/
    time_elapsed = GetTickCount() ;
    while ( (status & US_CSR_ENDRX) != US_CSR_ENDRX )
    {
        status = USART_GetStatus( USART1 ) ;
        if ( GetTickCount() - time_elapsed > TIMEOUT )
        {
            break ;
        }
    }

    /* if acknowledgment received in a short time*/
    if ( (status & US_CSR_ENDRX) == US_CSR_ENDRX )
    {
        /* Acknowledgment*/
        if ( cSync == ACK_CHAR )
        {
            /*act as transmitter, start transmitting*/
            state = TRANSMITTING ;
            printf( "-I- Start transmitting!\n\r" ) ;
            USART_WriteBuffer( USART1, pBufTrans, PDC_BUF_SIZE ) ;

            pBufTrans += PDC_BUF_SIZE ;
            /* enable transmitting interrupt */
            USART_EnableIt( USART1, US_IER_ENDTX ) ;
        }
    }
    else
    {
        /* start receiving*/
        /* act as receiver*/
        USART_ReadBuffer( USART1, &cSync, 1 ) ;
        status = USART_GetStatus( USART1 ) ;

        printf( "-I- Receiving sync character.\n\r" ) ;
        while ( (status & US_CSR_ENDRX) != US_CSR_ENDRX )
        {
            status = USART_GetStatus( USART1 ) ;
        }

        /* sync character is received */
        if ( cSync == SYNC_CHAR )
        {
            /* SEND XOff AS acknowledgment */
            cSync = ACK_CHAR ;

            /* Delay to prevent the character from being discarded by transmitter due to responding too soon */
            Wait( 100 ) ;
            USART_WriteBuffer( USART1, &cSync, 1 ) ;
            state = RECEIVING ;

            printf( "-I- Start receiving!\n\r" ) ;
            USART_ReadBuffer( USART1, pBufRecv, PDC_BUF_SIZE ) ;
            pBufRecv += PDC_BUF_SIZE ;

            /* enable receiving interrupt */
            USART_EnableIt( USART1, US_IER_ENDRX ) ;
        }
    }
    while ( state != RECEIVED ) ;
    Wait( 200 ) ;

    i = 0 ;
    /* print received frame out*/
    while ( i < BUFFER_SIZE && pBufferReceive[i] != '\0' )
    {
        if ( pBufferTransmit[i] != pBufferReceive[i] )
        {
            printf( "-E- Error occurred while receiving!\n\r" ) ;
            /* infinite loop here*/
            while( 1 ) ;
        }
        i++ ;
    }
    printf( "-I- Received successfully!\n\r" ) ;

    while( 1 ) ;
}
