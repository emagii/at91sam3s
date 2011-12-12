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
 *  \page usart_spi  USART SPI Mode Example
 *
 *  \section Purpose
 *
 *  This example demonstrates the SPI  mode provided by the USART peripherals on
 *  SAM3S.
 *
 *  \section Requirements
 *
 *  This example can be used on sam3s evaluation kit and usart1 and spi pins
 *  should be connected as following matching table:
 *  - <b> USART1 -- SPI </b>
 *   - SCK1(PA23) -- SPCK(PA14)
 *   - TXD1(PA22) -- MOSI(PA13
 *   - RXD1(PA21) -- MISO(PA12)
 *   - RTS(PA24)  -- NPCS0(PA11)  (for USART1 as SPI master)
 *
 *  - <b> USART1 -- SPI </b>
 *   - SCK1(PA23) -- SPCK(PA14)
 *   - RXD1(PA21) -- MOSI(PA13)
 *   - TXD1(PA22) -- MISO(PA12)
 *   - CTS(PA25)  -- NPCS0(PA11)  (for USART1 as SPI slave)
 *
 *  \section Description
 *
 * This example demonstrates how to use USART in SPI mode. The USART is
 * configured as SPI master or slave. Meanwhile, the SPI peripheral in the
 * microcontroller is configured respectively, making it communicate with the
 * USART peripheral. The example leaves UART to let user to switch the command
 * between READ and WRITE from the view of master.

 * The application first initializes UART as the interface to interact with
 * users. By default, it initializes USART as SPI master and the SPI peripheral
 * as SPI slave.

 * The application waits for input from UART:

 *     - If 'w' is received, the master initiate a transmission by sending a
 *     block of data. The slave receives the data and dumps it to UART.
 *     - If 'r' is received, the master wait for data from slave and dump it to
 *     UART until the reception is finished.
 *     - If 's' is received, the master and slave are exchanged between USART1
 *     and SPI. To assure the transferring succeeds, connect pins as the table
 *     above.
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
 *     -- USART SPI Mode Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - usart_spi/main.c
 *  - pio.h
 *  - usart.h
 */

/** \file
 *
 *  This file contains all the specific code for the usart_spi example.
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
/** clock of USART1 in spi mode*/
#define USART_SPI_CLK       8000000UL
/** usart1 master,spi slave*/
#define STATE_UM_SS    0
/** usart1 slave,spi master*/
#define STATE_US_SM    1

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** Pins to configure for the application.*/
const Pin pins[] = { BOARD_PIN_USART_RXD,
                     BOARD_PIN_USART_TXD,
                     PIN_USART1_SCK,
                     BOARD_PIN_USART_RTS,
                     BOARD_PIN_USART_CTS,
                     PINS_SPI,
                     PIN_SPI_NPCS0_PA11};

/** Transmit buffer. */
char
Buffer[BUFFER_SIZE]=

 " DESCRIPTION of this example: \n\r \
 **************************************************************************\n\r\
 This example demonstrates how to use USART in SPI mode. The USART is \n\r\
 configured as SPI master or slave. Meanwhile, the SPI peripheral in the \n\r\
 microcontroller is configured respectively, making it communicate with the \n\r \
 USART peripheral. The example leaves UART to let user to switch the command \n\r\
 between READ and WRITE from the view of master. \n\r\
 The application first initializes UART as the interface to interact with \n\r\
 users. By default, it initializes USART as SPI master and the SPI peripheral \n\r\
 as SPI slave. \n\r\
 The application waits for input from UART:\n\r\
      - If 'W' is received, the USART initiate a transmission by sending a \n\r\
      block of data. The SPI slave receives the data and dumps it to UART. \n\r\
      - If 'R' is received, the USART wait for data from SPI bus and dump it to\n\r\
      UART until the reception is finished. \n\r\
      - If 's' is received, the master and slave are exchanged between USART1\n\r\
      and SPI. To assure the transferring succeeds, connect pins as the table \n\r\
      above.\n\r\
 **************************************************************************\n\r\
 END of DESCRIPTION \n\r\
 ";
/** temporary buffer for master to generate clock. */
char Buffer1[BUFFER_SIZE]={0xaa,0x55,0x11,0x22,0x33};

/** buffer for receiving */
char pRecvBufferSPI[BUFFER_SIZE]= { 0 };
char pRecvBufferUSART1[BUFFER_SIZE]= { 0 };

/** reception done*/
volatile bool recvDone = false;

/** mode for usart1 and spi ,0 means usart1 as master and spi as slave,1 for
 another state*/
uint8_t glob_state = STATE_UM_SS;

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/** \brief SPI irq handlere
 *
 */
void SPI_IrqHandler(void)
{
    uint32_t spiSr;
    spiSr = SPI->SPI_SR;
    if (spiSr & SPI_SR_RXBUFF) {
        recvDone = true;
        SPI_DisableIt(SPI, SPI_IDR_RXBUFF);
    }
}
/* \brief USART1 IRQ handler
 *
 * Interrupt handler for USART. After reception is done,set recvDone to true
 * met yet.
 *----------------------------------------------------------------------------*/
void USART1_IrqHandler( void )
{
    uint32_t status ;

    /*  Read USART status */
    status = BOARD_USART_BASE->US_CSR ;

    /* Receive buffer is full */
    if ( (status & US_CSR_RXBUFF) == US_CSR_RXBUFF )
    {
        recvDone = true ;
        USART_DisableIt( USART1, US_IDR_RXBUFF ) ;
    }
}

/**
 *  \brief Configures spi in spi_mode with respect to state
 */
static void _ConfigureSpi( uint8_t state )
{
    uint32_t mode = SPI_MR_MSTR ;
    uint32_t csr0 ;

    /* spi in slave mode */
    if ( state == STATE_UM_SS )
    {
        mode &= (uint32_t) (~(SPI_MR_MSTR)) ;
    }
    csr0 = SPI_CSR_BITS_8_BIT | SPI_CSR_DLYBS(0xFF) ;

    /* spi clock */
    if ( state == STATE_US_SM )
    {
        csr0 |= ((BOARD_MCK/USART_SPI_CLK) << 8 ) ;
    }
    /* configure SPI mode */
    SPI_Configure( SPI, ID_SPI, mode ) ;

    NVIC_EnableIRQ( SPI_IRQn ) ;
    /* configure SPI csr0*/
    SPI_ConfigureNPCS( SPI, 0, csr0 ) ;

    SPI_Enable( SPI ) ;
}

/**
 * \brief Configures USART in spi mode  with respect to state
 * \param state the master or slave of USART
 */
static void _ConfigureUsart( uint8_t state )
{
    uint32_t mode = US_MR_USART_MODE_SPI_MASTER | US_MR_USCLKS_MCK | US_MR_CHRL_8_BIT
                    | US_MR_PAR_NO | US_MR_CHMODE_NORMAL | US_MR_CLKO
                    | US_SPI_BPMODE_1 ;

    /* slave mode configuration*/
    if ( state == STATE_US_SM )
    {
        mode = US_MR_USART_MODE_SPI_SLAVE | US_MR_CHRL_8_BIT
                        | US_MR_CHMODE_NORMAL
                        | US_SPI_BPMODE_1;
    }

    /* Enable the peripheral clock in the PMC */
    PMC_EnablePeripheral( BOARD_ID_USART ) ;

    /* Configure the USART in the desired mode @USART_SPI_CLK bauds*/
    USART_Configure( BOARD_USART_BASE, mode, USART_SPI_CLK, BOARD_MCK ) ;

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
    printf("-- w: Write data block by master.--\n\r");
    printf("-- r: Read data block by master.--\n\r");
    printf("-- s: Switch mode for USART1 and SPI between master and slave.--\n\r");
    printf("-- m: Display this menu again.--\n\r");
}

/**
 * \brief Dump buffer to uart
 *
 */
static void _DumpInfo( char* pcBuffer, uint32_t dwSize )
{
    uint32_t i = 0 ;

    while ( (i < dwSize) && (pcBuffer[i] != 0))
    {
        printf( "%c", pcBuffer[i++] ) ;
    }
}

/*------------------------------------------------------------------------------
 *         Global functions
 *------------------------------------------------------------------------------*/


/**
 * \brief Application entry point.
 *
 * Configures USART1 in spi master/slave mode and SPI in slave/master mode,start
 * a transmission between two peripherals.
 * \return Unused.
 */
extern int main( void )
{
    char c ;
    char data = 0x0 ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /*  Configure pins */
    PIO_Configure( pins, PIO_LISTSIZE( pins ) ) ;

    /* Example information log */
    printf( "-- USART SPI Mode Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* display main menu*/
    _DisplayMainmenu() ;

    /* configure USART1 in  Master and SPI in slave mode*/
    _ConfigureUsart( STATE_UM_SS ) ;
    _ConfigureSpi( STATE_UM_SS ) ;

    printf( "--  USART1 as MASTER,SPI as SLAVE.--\n\r" ) ;

    while ( 1 )
    {
        c = UART_GetChar() ;
        switch ( c )
        {
            case 'w':
            case 'W':
            if ( glob_state == STATE_UM_SS )
            {
                data = SPI->SPI_RDR;
                printf( "0x%x\n\r", data ) ;

                /* slave in */
                SPI_ReadBuffer( SPI, pRecvBufferSPI, BUFFER_SIZE ) ;
                SPI_EnableIt( SPI, SPI_IER_RXBUFF ) ;
                /* master out*/
                USART_WriteBuffer( BOARD_USART_BASE, Buffer, BUFFER_SIZE ) ;
                while ( !recvDone ) ;

                if ( recvDone )
                {
                    printf( "----USART1 MASTER WRITE----\n\r" ) ;

                    if ( strncmp( pRecvBufferSPI, Buffer, BUFFER_SIZE ) )
                    {
                        printf( " -F-: Failed!\n\r" ) ;
                    }
                    else
                    {
                       /* successfully received*/
                       _DumpInfo( pRecvBufferSPI, BUFFER_SIZE ) ;
                    }
                    printf( "----END of USART1 MASTER WRITE----\n\r" ) ;
                    memset( pRecvBufferSPI, 0, sizeof( pRecvBufferSPI ) ) ;
                    recvDone = false ;
                }
            }
            else
            {
                data = USART1->US_RHR ;
                printf( "US_RHR:0x%x\n\r", data ) ;

                /* slave in */
                USART_ReadBuffer( USART1, pRecvBufferUSART1, BUFFER_SIZE ) ;
                USART_EnableIt( USART1, US_IER_RXBUFF ) ;
                printf( "----SPI MASTER WRITE----\n\r" ) ;

                /* master out*/
                SPI_WriteBuffer( SPI, Buffer, BUFFER_SIZE ) ;
                while ( !recvDone ) ;
                if ( recvDone )
                {
                    if ( strncmp( pRecvBufferUSART1, Buffer, BUFFER_SIZE ) )
                    {
                       printf( " -F-: Failed!\n\r" ) ;
                    }
                    else
                    {
                       /* successfully received*/
                       _DumpInfo( pRecvBufferUSART1, BUFFER_SIZE ) ;
                    }

                    printf( "----END of SPI MASTER WRITE----\n\r" ) ;
                    memset( pRecvBufferUSART1, 0, sizeof( pRecvBufferUSART1 ) ) ;
                    recvDone = false ;
                }
            }
            break ;

            case 'r':
            case 'R':
            if ( glob_state == STATE_UM_SS )
            {
                data = USART1->US_RHR ;
                printf( "US_RHR:0x%x\n\r", data ) ;

                /* slave out */
                SPI_WriteBuffer( SPI, Buffer, BUFFER_SIZE ) ;

                /* master read */
                USART_ReadBuffer( USART1, pRecvBufferUSART1, BUFFER_SIZE ) ;
                USART_EnableIt( USART1, US_IER_RXBUFF ) ;

                /* start transmission */
                USART_WriteBuffer( BOARD_USART_BASE, Buffer1, BUFFER_SIZE ) ;
                printf( "----USART1 MASTER READ----\n\r" ) ;
                while ( !recvDone ) ;

                if ( recvDone )
                {
                    if ( strncmp( pRecvBufferUSART1, Buffer, BUFFER_SIZE ) )
                    {
                       printf( " -F-: Failed!\n\r" ) ;
                    }
                    else
                    {
                       /* successfully received*/
                       _DumpInfo( pRecvBufferUSART1, BUFFER_SIZE ) ;
                    }
                    printf( "----END of USART1 MASTER READ----\n\r" ) ;
                    memset( pRecvBufferUSART1, 0, sizeof( pRecvBufferUSART1 ) ) ;
                    recvDone = false ;
                }
            }
            else
            {
                data = SPI->SPI_RDR ;
                printf( "SPI_RDR:0x%x\n\r", data ) ;

                /* slave out */
                USART_WriteBuffer( USART1, Buffer, BUFFER_SIZE ) ;
                printf( "----SPI MASTER READ----\n\r" ) ;

                /* master read */
                SPI_ReadBuffer( SPI, pRecvBufferSPI, BUFFER_SIZE ) ;
                SPI_EnableIt( SPI, SPI_IER_RXBUFF ) ;

                /* start transmission */
                SPI_WriteBuffer( SPI, Buffer1, BUFFER_SIZE ) ;
                while ( !recvDone ) ;

                if ( recvDone )
                {
                    if ( strncmp( pRecvBufferSPI, Buffer, BUFFER_SIZE ) )
                    {
                        printf( " -F-: Failed!\n\r" ) ;
                    }
                    else
                    {
                       /* successfully received */
                       _DumpInfo( pRecvBufferSPI, BUFFER_SIZE ) ;
                    }
                    printf("----END of SPI MASTER READ----\n\r");
                    memset(pRecvBufferSPI,0,sizeof(pRecvBufferSPI));
                    recvDone = false;
                }

            }
            break ;

            case 's':
            case 'S':
                if ( glob_state == STATE_UM_SS )
                {
                    glob_state = STATE_US_SM ;
                    _ConfigureUsart( glob_state ) ;
                    _ConfigureSpi( glob_state ) ;

                    printf( "-- USART1 as SLAVE,SPI as MASTER\n\r" ) ;

                }
                else
                {
                    glob_state = STATE_UM_SS ;
                    _ConfigureUsart( glob_state ) ;
                    _ConfigureSpi( glob_state ) ;

                    printf( "-- USART1 as MASTER,SPI as SLAVE\n\r" ) ;
                }
            break ;

            case 'm':
            case 'M':
                _DisplayMainmenu() ;
            break ;

        }

    }
}

