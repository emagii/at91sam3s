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
 * \page hsmci_sdcard Basic SD/MMC Card Example
 *
 * \section Purpose
 *
 *  The hsmci_sdcard will help you to get familiar with sdmmc_mci interface on
 *  SAM microcontrollers. It can also help you to get familiar with the SD operation flow
 *  which can be used for fast implementation of your own sd drivers and other applications
 *  related.
 *
 *  You can find following information depends on your needs:
 *  - Usage of auto detection of sdcard insert and sdcard write-protection detection
 *  - MCI interface initialize sequence and interrupt installation
 *  - Sdcard driver implementation based on MCI interface
 *  - Sdcard physical layer initialize sequence implementation
 *  - Sample usage of sdcard write and read
 *
 *  See
 *
 *  - sdmmc : sdcard physical layer driver with mci-interface
 *  - hsmci : sdcard physical layer driver with mci-interface
 *  - mci_cmd : sdcard physical layer driver with mci-interface
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3S evaluation kit.
 *
 *  \section Description
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download this program to
 *  SRAM or Flash, make the program run, the HyperTerminal will give out the test results.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a> application note or to the
 *     <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *     IAR EWARM User Guide</a>, depending on your chosen solution.
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application
 *  -# In HyperTerminal, it will show something like
 *      \code
 *      -- Basic HSMCI SD/MMC Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Please connect a SD card ...
 *      -I- SD card connection detected
 *      -I- Cannot check if SD card is write-protected
 *      -I- SD/MMC card initialization successful
 *      -I- Card size: *** MB
 *      -I- Block size: *** Bytes
 *      -I- Testing block [  *** -   ***] ..."
 *      \endcode
 *
 *  \section References
 *  - hsmci_sdcard/main.c
 *  - hsmci.h
 *  - pio.h
 *
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the hsmci_sdcard example.
 *
 *  \section Contents
 *  The hsmci_sdcard application can be roughly broken down as follows:
 *     - Optional functions
 *        - CheckProtection
 *        - WaitSdConn
 *     - Interrupt handlers
 *        - ISR_Mci0
 *     - The main function, which implements the program behavior
 *        - I/O configuration
 *        - sd card auto-detect and check whether sd card is write-protected
 *          (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - write/read sdcard at max available sd clock
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "memories.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>



/*----------------------------------------------------------------------------
 *         Local constants
 *----------------------------------------------------------------------------*/

/** Number of blocks read once */
#define NB_MULTI_BLOCKS     5

/** Number of blocks read in first step */
#define NB_SPLIT_MULTI      2

/**  Number of errors displayed */
#define NB_ERRORS       5


/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/**  MCI driver instance. */
static Mcid mciDrv;

/**  SDCard driver instance. */
static SdCard sdDrv;

/**  SD card pins. */
static const Pin pinsSd[] = {BOARD_SD_PINS};

/** Date buffer */
static uint8_t pBuffer[SDMMC_BLOCK_SIZE * NB_MULTI_BLOCKS];

/** Number of errors found */
static uint32_t nbErrors;

/*----------------------------------------------------------------------------
 *         Local macros
 *----------------------------------------------------------------------------*/

/** Start block address for test */
#define TEST_BLOCK_START    0

/** End block address for test */
#define TEST_BLOCK_END      SD_TOTAL_BLOCK(&sdDrv) // Whole SD Card

/* Defined to test Multi-Block functions */
#define READ_MULTI
#define WRITE_MULTI

/** \def SDT_ReadFun()
 *  Function used for SD card test reading.
 */
#ifdef  READ_MULTI
#define SDT_ReadFun(pSd, blk, nbBlk, pData) \
    SD_Read(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define SDT_ReadFun(pSd, blk, nbBlk, pData) \
    SD_ReadBlocks(pSd, blk, nbBlk, pData)
#endif

/** \def SDT_WriteFun()
 *  Function used for SD card test writing.
 */
#ifdef  WRITE_MULTI
#define SDT_WriteFun(pSd, blk, nbBlk, pData) \
    SD_Write(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define SDT_WriteFun(pSd, blk, nbBlk, pData) \
    SD_WriteBlocks(pSd, blk, nbBlk, pData)
#endif
/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 *  MCI0 interrupt handler. Forwards the event to the MCI driver handler.
 */
void MCI_IrqHandler(void)
{
    Sdmmc_Handler(&mciDrv);
}

//------------------------------------------------------------------------------
//         Optional: SD card detection
//------------------------------------------------------------------------------

/**  SD card detection pin instance. */
static const Pin pinMciCardDetect = BOARD_SD_PIN_CD;

/**
 *  Waits for a SD card to be connected.
 */
static void WaitSdConn(void)
{
    PIO_Configure(&pinMciCardDetect, 1);
    printf("-I- Please connect a SD card ...\n\r");
    while (PIO_Get(&pinMciCardDetect) != 0);
    printf("-I- SD card connection detected\n\r");
}

/**
 *  Dummy implementation for Write Protection Detect.
 */
static void CheckProtection(void)
{
    printf("-I- Cannot check if SD card is write-protected\n\r");
}

/**
 *  Max Error Break
 */
static uint8_t MaxErrorBreak(void)
{
    if (NB_ERRORS) {
        if (nbErrors ++ > NB_ERRORS) {
            while(1);
        }
    }
    return 0;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief hsmci_sdcard Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    uint8_t rc;
    uint32_t block;
    uint32_t i;
    uint16_t multiBlock;
    uint8_t error;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- Basic HSMCI SD/MMC Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure SDcard pins */
    PIO_Configure(pinsSd, PIO_LISTSIZE(pinsSd));

    /* Wait for SD card connection (if supported) */
    WaitSdConn();

    /* Check if card is write-protected (if supported) */
    CheckProtection();

    /* Initialize the MCI driver */
    MCI_Init( &mciDrv, HSMCI, ID_HSMCI, BOARD_MCK ) ;
    NVIC_EnableIRQ( HSMCI_IRQn ) ;

    /* Initialize the SD card driver */
    rc = SD_Init(&sdDrv, &mciDrv);
    if ( rc )
    {
        printf("-E- SD/MMC initialization failed: %x\n\r", rc);
    }
    else
    {
        printf("-I- SD/MMC card initialization successful\n\r");
        printf("-I- Card size: %d MB", (int)(SD_GetTotalSizeKB(&sdDrv)/1000));
        printf(", %d * %dB\n\r", (int)SD_GetNumberBlocks(&sdDrv), (int)SD_GetBlockSize(&sdDrv));
    }

    if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDIO)
    {
        SDIO_DisplayCardInformation(&sdDrv);
    }

    if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDMMC)
    {
        SD_DisplayRegisterCID((SdCard*)&sdDrv);
        SD_DisplayRegisterCSD((SdCard*)&sdDrv);
        SD_DisplayRegisterECSD((SdCard*)&sdDrv);
    }
    else
    {
        printf("-I- Not SD/MMC Memory card, exit\n\r");
        return 0;
    }

    if ( rc )
    {
        return 0;
    }

    /* Perform tests on each block */
    multiBlock = 0;
    for ( block = TEST_BLOCK_START; block < TEST_BLOCK_END; block += multiBlock )
    {
         uint32_t splitMulti;

        /* Perform different single or multiple bloc operations */
        if (++multiBlock > NB_MULTI_BLOCKS)
        {
            multiBlock = 1;
        }

        /* Multi-block adjustment */
        if (block + multiBlock > TEST_BLOCK_END)
        {
            multiBlock = TEST_BLOCK_END - block;
        }

        // Perform single block or multi block transfer
        printf("\r-I- Testing block [%6u - %6u] ...", (unsigned int)block, (unsigned int)(block + multiBlock -1));

        // Clear the block
        memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
        for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++)
        {
            if ( pBuffer[i] != 0 )
            {
                printf("\n\r-E- Data @ %u before write : 0x00 <> 0x%02x\n\r", (unsigned int)i, pBuffer[i]);
                MaxErrorBreak();
                // Only find first verify error.
                continue;
            }
        }

        error = SDT_WriteFun(&sdDrv, block, multiBlock, pBuffer);
        if ( error )
        {
            printf("\n\r-E- 1. Write block (%d) #%u\n\r", error, (unsigned int)block);
            MaxErrorBreak();
            // Skip following test
            continue;
        }

        // Read back the data to check the write operation
        memset(pBuffer, 0xFF, SDMMC_BLOCK_SIZE * multiBlock);
        error = SDT_ReadFun(&sdDrv, block, multiBlock, pBuffer);
        if ( error )
        {
            printf("\n\r-E- 1. Read block (%d) #%u\n\r", error, (unsigned int)block);
            MaxErrorBreak();
            // Skip following test
            continue;
        }

        for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++)
        {
            if (pBuffer[i] != 0)
            {
                printf("\n\r-E- 1. Data @ %u : 0x00 <> 0x%02X\n\r", (unsigned int)i, pBuffer[i]);
                MaxErrorBreak();
                // Only find first verify error.
                break;
            }
        }

        // Write a checkerboard pattern on the block
        for ( i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++ )
        {
            if ( (i & 1) == 0 )
            {
                pBuffer[i] = (i & 0x55);
            }
            else
            {
                pBuffer[i] = (i & 0xAA);
            }
        }

        for (i = 0;i < multiBlock;)
        {
            splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ?
                                    NB_SPLIT_MULTI : (multiBlock - i);
            error = SDT_WriteFun(&sdDrv,
                                   block + i,
                                   splitMulti,
                                   &pBuffer[i * SDMMC_BLOCK_SIZE]);
            if (error)
                break;
            i += splitMulti;
        }

        assert( i == multiBlock ) ;//, "Unexpected W, %d!", i);
        if ( error )
        {
            printf("\n\r-E- 2. Write block #%u\n\r", (unsigned int)block);
            MaxErrorBreak();
            // Skip Following Test
            continue;
        }

        // Read back the data to check the write operation
        memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
        for ( i = 0;i < multiBlock; )
        {
            splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ? NB_SPLIT_MULTI : (multiBlock - i);
            error = SDT_ReadFun(&sdDrv,
                                  block + i,
                                  splitMulti,
                                  &pBuffer[i * SDMMC_BLOCK_SIZE]);
            if (error)
            {
                break;
            }
            i += splitMulti;
        }

        assert( i == multiBlock ) ; //, "Unexpected R %d!", i);

        if (error)
        {
            printf("\n\r-E- 2. Read block #%u\n\r", (unsigned int)block);
            MaxErrorBreak();
            // Skip Following Test
            continue;
        }

        for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++)
        {
            if (!(((i & 1) == 0) && (pBuffer[i] == (i & 0x55))) &&
                !(((i & 1) != 0) && (pBuffer[i] == (i & 0xAA))) )
            {
                printf("\n\r-E- 2. Data @ %u 0x%02X <> 0x%02X\n\r", (unsigned int)i, (unsigned int)(((i & 1) == 0) ? (i & 0x55) : (i & 0xAA)), pBuffer[i]);
                MaxErrorBreak();
                // Only find first verify error.
                break;
            }
        }

        // Uses DBGU key to skip blocks
        if ( UART_IsRxReady() )
        {
            uint8_t bKey = UART_GetChar();
            if (bKey == 27)
            {
                block += 500 * 2;  // Skip 500K data
            }
            else
            {
                if (bKey == 'c')
                {
                    printf("\n\r\n\r*** User cancel\n\r");
                    break;
                }
            }
        }
    }

    return 0;
}

