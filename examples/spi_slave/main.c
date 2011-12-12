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
 * \page spi_slave SPI Slave Example
 *
 * \section Purpose
 *
 * This example uses Serial Peripheral Interface (SPI) in slave mode to
 * communicate to another SPI master) interface.
 *
 * \section Requirements
 *
 * This package can be used with two sam3s-ek board.
 * Please connect the SPI pins from one board to another.
 *        - <b>SAM3S(MASTER)--SAM3S(SLAVE)</b>
 *        - NPCS0--NPCS0
 *        - MISO--MISO
 *        - MOSI--MOSI
 *        - SPCK--SPCK
 *
 * \section Descriptions
 *
 * This example shows control of the SPI slave, how to configure and
 * transfer data with SPI slave. By default, example runs in SPI slave mode,
 * waiting SPI slave & UART inputs.

 * The code can be roughly broken down as follows:
 * <ul>
 * <li> 't' will start SPI transfer test
 * <ol>
 * <li>Configure SPI as master, setup SPI clock.
 * <li>Send 4 byte CMD_TEST to indicate the start of test.
 * <li>Send several 64 bytes block, on transmitting next block, the content of last block is returned.
 * <li>Send CMD_STATUS command and waiting for the status reprots from slave.
 * <li>Send CMD_END command to indicate the end of test.
 * </ol>
 * <li>Setup SPI clock for master.
 * </ul>
 *
 * \section Usage
 *
 * -# Compile the application.
 * -# Connect the DBGU port of the evaluation board to the computer and open
 * it in a terminal.
 *    - Settings: 115200 bauds, 8 bits, 1 stop bit, no parity, no flow control.
 * -# Download the program inside the evaluation board and run it. Please refer to
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Upon startup, the application will output the following line on the DBGU:
 *    \code
 *     -- Spi slave Example  xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The following traces detail operations on the spl slave example, displaying success
 *    or error messages depending on the results of the commands.
 *
 * \section References
 * - spi_slave/main.c
 * - spi.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi slave example.
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <stdarg.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI

/** SPI slave states for this example*/
#define SLAVE_STATE_IDLE           0
#define SLAVE_STATE_TEST           1
#define SLAVE_STATE_DATA           2
#define SLAVE_STATE_STATUS_ENTRY   3
#define SLAVE_STATE_STATUS         4
#define SLAVE_STATE_END            5

/** SPI example commands for this example */
/** slave test state, begin to return RC_RDY */
#define CMD_TEST    0x10101010
/** slave data state, begin to return last data block*/
#define CMD_DATA    0x29380000
/** slave status state, begin to return RC_RDY + RC_STATUS*/
#define CMD_STATUS  0x68390384
/** slave idle state, begin to return RC_SYN*/
#define CMD_END     0x68390484
/** General return value*/
#define RC_SYN      0x55AA55AA
/** Ready status*/
#define RC_RDY      0x12345678
/** Number of commands logged in status */
#define NB_STATUS_CMD   20
/** Number of SPI clock configurations */
#define NUM_SPCK_CONFIGURATIONS 4
/*----------------------------------------------------------------------------
 *        Tyoes
 *----------------------------------------------------------------------------*/

/** Status block */
typedef struct _StatusBlock
{
    /** Number of data blocks */
    uint32_t totalNumBlocks;
    /** Number of SPI commands (including data blocks) */
    uint32_t totalNumCommands;
    /** Command list */
    uint32_t cmdList[NB_STATUS_CMD];
} StatusBlock ;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIOs for all SPI modes */
static const Pin pSpiPins[] = {PINS_SPI, PIN_SPI_NPCS0_PA11};

/** SPI Clock setting (Hz) */
static uint32_t spiClock = 100000;
/** Current SPI return code */
static uint32_t spiCmd = RC_SYN;

/** Current SPI state */
static uint8_t spiState = 0;

/** 64 bytes data buffer for SPI transfer and receive */
static uint8_t spiBuffer[64];

/** SPI Status */
static StatusBlock spiStatus;

static uint8_t numTestBlocks;

/** SPI clock configuration */
static const uint32_t clockConfigurations[] = { 100000, 500000, 1000000, 5000000};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu( void )
{
    uint32_t i ;

    printf( "\n\rMenu :\n\r" ) ;
    printf( "------\n\r" ) ;

    for ( i = 0 ; i < NUM_SPCK_CONFIGURATIONS ; i++ )
    {
        printf("  %u: Set SPCK = %7d Hz\n\r", (unsigned int)i, (unsigned int)clockConfigurations[i] ) ;
    }
    printf( "  t: Perform SPI master\n\r" ) ;
    printf( "  h: Display this menu again\n\r\n\r" ) ;
}

/**
 * \brief Perform SPI slave transfer using PDC.
 * \param pBuf Pointer to 1st buffer to transfer.
 * \param size Size of the 1st buffer.
 * \param pNextBuf Pointer to 2nd buffer to transfer.
 * \param nextSize Size of the 2nd buffer.
 */
static void SpiSlaveTransfer( void* pBuf, uint16_t size, void* pNextBuf, uint16_t nextSize )
{
    uint32_t spiIER ;

    SPI_PdcSetTx( SPI_SLAVE_BASE, pBuf, size, pNextBuf, nextSize ) ;
    SPI_PdcSetRx( SPI_SLAVE_BASE, pBuf, size, pNextBuf, nextSize ) ;

    /* Enable the RX and TX PDC transfer requests */
    SPI_PdcEnableRx( SPI_SLAVE_BASE ) ;
    SPI_PdcEnableTx( SPI_SLAVE_BASE ) ;

    /* Transfer done handler is in ISR ... */
    spiIER = SPI_IER_NSSR | SPI_IER_RXBUFF | (pNextBuf ? SPI_IER_ENDRX : 0) ;
    SPI_EnableIt( SPI_SLAVE_BASE, spiIER ) ;
}

/**
 * \brief  SPI Command block process
 */
static void SpiSlaveCommandProcess( void )
{
    if ( spiCmd == CMD_END )
    {
        spiState = SLAVE_STATE_IDLE;
        spiStatus.totalNumBlocks   = 0;
        spiStatus.totalNumCommands = 0;
    }
    else
    {
        switch( spiState )
        {
            case SLAVE_STATE_IDLE:
                /* Only CMD_TEST accepted*/
                if ( spiCmd == CMD_TEST )
                {
                    spiState = SLAVE_STATE_TEST ;
                }
            break ;

            case SLAVE_STATE_TEST:
                /* Only CMD_DATA accepted*/
                if ( (spiCmd & 0xffff0000) == CMD_DATA )
                {
                    spiState = SLAVE_STATE_DATA ;
                }
                numTestBlocks = spiCmd & 0x0000ffff ;
            break;

            case SLAVE_STATE_DATA:
                spiStatus.totalNumBlocks++ ;

                if ( spiStatus.totalNumBlocks == numTestBlocks )
                {
                    spiState = SLAVE_STATE_STATUS_ENTRY;
                }
            break ;

            case SLAVE_STATE_STATUS_ENTRY :
                spiState = SLAVE_STATE_STATUS ;
            break ;

            case SLAVE_STATE_END:
            break;
        }
    }
}

/**
 * \brief  Start waiting new command
 */
static void SpiSlaveNewCommand( void )
{
    switch ( spiState )
    {
        case SLAVE_STATE_IDLE:
        case SLAVE_STATE_END:
            spiCmd = RC_SYN;
            SpiSlaveTransfer( &spiCmd, 4, 0, 0 ) ;
        break ;

        case SLAVE_STATE_TEST:
            spiCmd = RC_RDY;
            SpiSlaveTransfer( &spiCmd, 4, 0 , 0 ) ;
        break ;

        case SLAVE_STATE_DATA:
            if ( spiStatus.totalNumBlocks < numTestBlocks )
            {
                SpiSlaveTransfer( spiBuffer, 64, 0, 0 ) ;
            }
        break ;

        case SLAVE_STATE_STATUS_ENTRY:
            spiCmd = RC_RDY ;
            SpiSlaveTransfer( &spiCmd, 4, 0 , 0 ) ;
            spiState = SLAVE_STATE_STATUS ;
        break ;

        case SLAVE_STATE_STATUS:
            spiCmd = RC_SYN;
            SpiSlaveTransfer( &spiStatus, sizeof( StatusBlock ), 0, 0 ) ;
            spiState = SLAVE_STATE_END ;
        break ;
    }
}

/**
 * \brief Interrupt handler for the SPI slave.
 */
void SPI_IrqHandler( void )
{
    uint32_t status;
    uint8_t startNew = 0;
    status = SPI_GetStatus( SPI_SLAVE_BASE ) ;

    if ( status & SPI_SR_NSSR )
    {
        if ( status & SPI_SR_RXBUFF )
        {
            /* Disable the RX and TX PDC transfer requests */
            SPI_PdcDisableTx( SPI_SLAVE_BASE ) ;
            SPI_PdcDisableRx( SPI_SLAVE_BASE ) ;
        }

        if ( status & SPI_IDR_ENDRX )
        {
            SPI_DisableIt( SPI_SLAVE_BASE, SPI_IDR_ENDRX ) ;
        }

        switch ( status & (SPI_SR_RXBUFF | SPI_SR_ENDRX) )
        {
            case (SPI_SR_RXBUFF | SPI_SR_ENDRX):
            case (SPI_SR_RXBUFF):
                SpiSlaveCommandProcess() ;
                startNew = 1 ;
            break ;

            /* Maybe command break data transfer, start new */
            case SPI_SR_ENDRX:
            {
                /* Command breaks data transfer */
                SPI_PdcDisableTx( SPI_SLAVE_BASE ) ;
                SPI_PdcDisableRx( SPI_SLAVE_BASE ) ;
                SPI_Configure( SPI_SLAVE_BASE, ID_SPI, 0 ) ;
                SPI_ConfigureNPCS( SPI_SLAVE_BASE, 0 , 0 ) ;
                startNew = 1 ;
            }
            break;

            default:
            break;
        }

        if ( startNew )
        {
            if ( spiCmd != CMD_END )
            {
                spiStatus.cmdList[spiStatus.totalNumCommands] = spiCmd;
                spiStatus.totalNumCommands ++;
            }
            SpiSlaveNewCommand();
        }
    }

}

/**
 * \brief Initialize SPI as slave
 */
static void SpiSlaveInitialize( void )
{
    uint32_t i ;
    printf("-I- Initialize SPI as slave ...\n\r");
    /* Configures a SPI peripheral */
    SPI_Configure(SPI_SLAVE_BASE, ID_SPI, 0);
    SPI_ConfigureNPCS(SPI_SLAVE_BASE, 0 , 0);
    /* Disable the RX and TX PDC transfer requests */
    SPI_PdcDisableTx(SPI_MASTER_BASE);
    SPI_PdcDisableRx(SPI_MASTER_BASE);
    /* Enables a SPI peripheral. */
    SPI_Enable(SPI_SLAVE_BASE);

    /* Reset status */
    spiStatus.totalNumBlocks   = 0;
    spiStatus.totalNumCommands = 0;
    for ( i = 0; i < NB_STATUS_CMD; i++ )
    {
        spiStatus.cmdList[i] = 0 ;
    }

    /* Start waiting command */
    spiState = SLAVE_STATE_IDLE;
    spiCmd = RC_SYN;
    SpiSlaveTransfer(&spiCmd, 4, 0, 0);
}

/**
 * \brief Initialize SPI as master
 */
static void SpiMasterInitialize( void )
{
    printf( "-I- Configure SPI as master\n\r" ) ;

    /* Master mode */
    SPI_Configure( SPI_MASTER_BASE, ID_SPI, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS( 0 ) ) ;
    SPI_ConfigureNPCS( SPI_MASTER_BASE, 0, SPI_DLYBCT( 100000, BOARD_MCK ) | SPI_DLYBS(100000, BOARD_MCK) | SPI_SCBR( spiClock, BOARD_MCK) ) ;

    /* Disable the RX and TX PDC transfer requests */
    SPI_PdcDisableTx( SPI_MASTER_BASE ) ;
    SPI_PdcDisableRx( SPI_MASTER_BASE ) ;

    /* Enables a SPI peripheral. */
    SPI_Enable( SPI_MASTER_BASE ) ;
}

/**
 * \brief Sets the specified SPI clock configuration.
 * \param configuration  Index of the configuration to set.
 */
static void SetClockConfiguration( uint8_t configuration )
{
    spiClock = clockConfigurations[configuration];
    printf("Setting SPI clock #%d ... \n\r", (unsigned int)clockConfigurations[configuration]);
    SpiMasterInitialize();
}

/**
 * \brief Perform SPI master transfer using PDC.
 * \param pBuf Pointer to 1st buffer to transfer.
 * \param size Size of the 1st buffer.
 * \param pNextBuf Pointer to 2nd buffer to transfer.
 * \param nextSize Size of the 2nd buffer.
 */
static void SpiMasterTransfer( void * pBuf, uint16_t size, void * pNextBuf, uint16_t nextSize )
{

    SPI_PdcSetTx(SPI_MASTER_BASE, pBuf, size, pNextBuf, nextSize);
    SPI_PdcSetRx(SPI_MASTER_BASE, pBuf, size, pNextBuf, nextSize);

    /* Enable the RX and TX PDC transfer requests */
    SPI_PdcEnableRx(SPI_MASTER_BASE);
    SPI_PdcEnableTx(SPI_MASTER_BASE);

    /* Waiting transfer done*/
    while((SPI_GetStatus(SPI_MASTER_BASE)& SPI_SR_RXBUFF) == 0);

    /* Disable the RX and TX PDC transfer requests */
    SPI_PdcDisableTx(SPI_MASTER_BASE);
    SPI_PdcDisableRx(SPI_MASTER_BASE);
}

/**
 * \brief Start SPI transfer test.
 */
static void SpiMasterGo( void )
{
    uint32_t cmd;
    uint32_t block;
    uint32_t i;
    /* Configure SPI as master, setup SPI clock. */
    SpiMasterInitialize();

    /* Send CMD_TEST to indicate the start of test, device shall return RC_RDY */

    while(1) {
        cmd = CMD_TEST;
        printf("-> Master sending CMD_TEST... \n\r");
        SpiMasterTransfer(&cmd, 4, 0, 0);
        if (cmd == RC_RDY) {
            printf("   <- Slave response RC_SYN, RC_RDY \n\r");
            break;
        }
        if (cmd != RC_SYN) {
            printf("-E- Response unexpected: 0x%x \n\r", (unsigned int)cmd);
            return;
        }
    }
    /* Send CMD_DATA with 8 blocks(64 bytes per page). */
    printf("-> Master sending CMD_DATA... \n\r");
    cmd = CMD_DATA | 4;
    SpiMasterTransfer(&cmd, 4, 0, 0);
    printf("                                <---- Slave response RC_RDY \n\r");
    for (block = 0; block < 4; block++) {
        for (i = 0; i < 64; i ++) {
            spiBuffer[i] = block;
        }
        printf("-> Master sending block %d ... \n\r", (unsigned int)block);
        SpiMasterTransfer(spiBuffer, 64, 0, 0);
        if (block) {
            for (i = 0; i < 64; i++) {
                if (spiBuffer[i] != (block - 1)) {
                    break;
                }
            }
            if (i < 64) {
                printf("-E- block %d contains unexpected data \n\r", (unsigned int)block);
            }
            else {
                printf("   <- Slave response last block %x \n\r", (unsigned int)(block - 1));
            }
        }
    }

    for (i = 0; i < 4; i++) {
        cmd = CMD_STATUS;
        printf("-> Master sending CMD_STATUS... \n\r");
        SpiMasterTransfer(&cmd, 4, 0, 0);
        if (cmd == RC_RDY){
            printf("   <- Slave response RC_RDY \n\r");
            break;
        }
    }
    if (i >= 4) {
        printf("   <- Slave no response \n\r");
        return;
    }

    printf("-> Master request slave status... \n\r");
    SpiMasterTransfer(&spiStatus, sizeof(StatusBlock), 0, 0);

    printf("   <- Slave reports status...\n\r" ) ;
    printf("-I- Received  %d commands:", (unsigned int)spiStatus.totalNumCommands ) ;

    for ( i = 0 ; i < spiStatus.totalNumCommands ; i ++ )
    {
        printf( " 0x%08x", (unsigned int)spiStatus.cmdList[i] ) ;
    }
    printf( " \n\r-I- Received  %d data blocks \n\r", (unsigned int)spiStatus.totalNumBlocks ) ;

    for ( i = 0 ; i < 4 ; i ++ )
    {
        printf( "-> Master sending CMD_END... \n\r" ) ;
        cmd = CMD_END ;
        SpiMasterTransfer( &cmd, 4, 0, 0 ) ;

        if ( cmd == RC_SYN )
        {
            printf( "   <- Slave response RC_SYN \n\r" ) ;
            break ;
        }
    }

    if ( i >= 4 )
    {
        printf( "   <- Slave no response \n\r" ) ;
    }
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for spi_slave example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    uint8_t ucKey ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- spi slave example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure PIO Pins for SPI */
    PIO_Configure( pSpiPins, PIO_LISTSIZE( pSpiPins ) ) ;

    /* Configure SPI interrupts for Slave only*/
    NVIC_DisableIRQ( SPI_IRQn ) ;
    NVIC_ClearPendingIRQ( SPI_IRQn ) ;
    NVIC_SetPriority( SPI_IRQn, 0 ) ;
    NVIC_EnableIRQ( SPI_IRQn ) ;

    SpiSlaveInitialize() ;

    /* Display menu */
    DisplayMenu() ;

    while ( 1 )
    {
        ucKey = UART_GetChar() ;

        switch ( ucKey )
        {
            case 'h' :
                DisplayMenu() ;
            break ;

            case 't' :
                SpiMasterGo() ;
            break ;

            default :
                /* Set configuration #n */
                if ( (ucKey >= '0') && (ucKey <= ('0' + NUM_SPCK_CONFIGURATIONS - 1)) )
                {
                    SetClockConfiguration( ucKey - '0' ) ;
                }
            break ;
        }
    }
}
