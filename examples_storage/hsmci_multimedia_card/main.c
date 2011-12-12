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

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "memories.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>


/**
 *  \page hsmci_multimedia_card Basic MultiMediaCard Example
 *
 *  \section Purpose
 *
 *  The Basic MultiMediaCard Example offers a set of functions to perform
 *  MultiMedia Card tests:
 *  -# Dump MultiMedia Card information
 *  -# Test all blocks on MultiMedia Card
 *  -# Test R/W Speed (performance) of the MultiMedia Card
 *
 *  The Basic MultiMediaCard Example will help you to get familiar with HSMCI
 *  interface on SAM microcontrollers. It can also help you to get familiar
 *  with the SD & MMC operation flow which can be used for fast implementation
 *  of your own SD/MMC drivers and other applications related.
 *
 *  You can find following information depends on your needs:
 *  - Usage of auto detection of sdcard insert and sdcard write-protection
 *  - (HS)MCI interface initialize sequence and interrupt installation
 *  - SD/MMC card driver implementation based on (HS)MCI interface
 *  - SD card physical layer initialize sequence implementation
 *  - MMC card physical layer initialize sequence implementation
 *  - Sample usage of SD/MMC card write and read
 *
 *  \par See Also
 *  - \ref hsmci_sdcard : Another Simple Example for SD/MMC access.
 *  - \ref sdmmc_lib : SD/MMC card driver with mci-interface.
 *  - \ref hsmci_module : sdcard physical layer driver with hsmci-interface.
 *
 *  \section Description
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download this
 *  program to FLASH, make the program run, the HyperTerminal will
 *  give out the test hints, you can run different tests on a inserted card.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6421.pdf">
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
 *  -# In HyperTerminal, it will show something like on start up
 *      \code
 *      -- Basic MultiMedia Card Project xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Cannot check if SD card is write-protected
 *
 *      ==========================================
 *      -I- Card Type 1, CSD_STRUCTURE 0
 *      -I- SD 4-BITS BUS
 *      -I- CMD6(1) arg 0x80FFFF01
 *      -I- SD HS Not Supported
 *      -I- SD/MMC TRANS SPEED 25000 KBit/s
 *      -I- SD/MMC card initialization successful
 *      -I- Card size: 483 MB, 990976 * 512B
 *      ...
 *      \endcode
 *  -# Test function menu is like this
 *      \code
 *      # i,I   : Re-initialize card
 *      # t     : Disk R/W/Verify test
 *      # T     : Disk performance test
 *      # p     : Change number of blocks in one access for test
 *      \endcode
 *
 *  \section References
 *  - hsmci_multimedia_card/main.c
 *  - hsmci.h
 *  - pio.h
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the hsmci_multimedia_card
 *  project.
 *
 *  \section Contents
 *  The hsmci_multimedia_card application can be roughly broken down as follows:
 *     - Optional functions for detection (card insert, card protect)
 *        - CardDetectConfigure(), CardIsConnected()
 *        - CardIsProtected()
 *     - Interrupt handlers
 *        - MCI_IrqHandler()
 *     - The main function, which implements the program behavior
 *        - I/O configuration
 *        - SD/MMC card auto-detect write-protected-check (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - write/read sdcard
 */

/*----------------------------------------------------------------------------
 *         Local consts
 *----------------------------------------------------------------------------*/

/** Maximum number of blocks read once (for performance test) */
#define NB_MULTI_BLOCKS     16

/** Split R/W to 2, first R/W 4 blocks then remaining */
#define NB_SPLIT_MULTI      4

/** Test settings: start block address (0) */
#define TEST_BLOCK_START    (0)

/** Test settings: end block address (total SD/MMC) */
#define TEST_BLOCK_END      SD_TOTAL_BLOCK(&sdDrv)

/** Test settings: skip size when "skip" key pressed */
#define TEST_BLOCK_SKIP     (100 * 1024 * 2)    // 100M

/** Test settings: Number of bytes to test performance */
#define TEST_PERFORMENCT_SIZE   (4*1024*1024)

/** Test settings: The value used to generate test data */
#define TEST_FILL_VALUE_U32     (0x5A6C1439)

/** Number of errors displayed */
#define NB_ERRORS       15

/** Number fo bad blocks displayed */
#define NB_BAD_BLOCK    200


/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** MCI driver instance. */
static Mcid mciDrv;

/** SDCard driver instance. */
static SdCard sdDrv;

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_SD_PINS};

/** SD card detection pin instance. */
static const Pin pinCardDetect = BOARD_SD_PIN_CD;

/** Buffer for test data blocks */
static uint8_t pBuffer[SDMMC_BLOCK_SIZE * NB_MULTI_BLOCKS];

/** Number or errors detected */
static uint32_t nbErrors;

/** Number of block r/w once to test performance */
static uint32_t performanceMultiBlock = NB_MULTI_BLOCKS;

/*----------------------------------------------------------------------------
 *         Local macros
 *----------------------------------------------------------------------------*/

/* Defined to test Multi-Block functions */

/** \def READ_MULTI
 *  \brief Define to test multi-read (SD_Read())
 *         or
 *         single-read is used (SD_ReadBlocks()) */
#define READ_MULTI
/** \def WRITE_MULTI
 *  \brief Define to test multi-write (SD_Write())
 *         or
 *         single-write is used (SD_WriteBlocks()) */
#define WRITE_MULTI

/** \macro SDT_ReadFun
 * Function used for SD card test reading.
 * \param pSd  Pointer to a SD card driver instance.
 * \param address  Address of the block to read.
 * \param nbBlocks Number of blocks to be read.
 * \param pData    Data buffer whose size is at least the block size.
 */
#ifdef  READ_MULTI
#define MMCT_ReadFun(pSd, blk, nbBlk, pData) \
    SD_Read(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define MMCT_ReadFun(pSd, blk, nbBlk, pData) \
    SD_ReadBlocks(pSd, blk, nbBlk, pData)
#endif

/** \macro SDT_WriteFun
 * Function used for SD card test writing.
 * \param pSd  Pointer to a SD card driver instance.
 * \param address  Address of the block to read.
 * \param nbBlocks Number of blocks to be read.
 * \param pData    Data buffer whose size is at least the block size.
 */
#ifdef  WRITE_MULTI
#define MMCT_WriteFun(pSd, blk, nbBlk, pData) \
    SD_Write(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define MMCT_WriteFun(pSd, blk, nbBlk, pData) \
    SD_WriteBlocks(pSd, blk, nbBlk, pData)
#endif

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * MCI interrupt handler. Forwards the event to the MCI driver handler.
 */
void MCI_IrqHandler(void)
{
    Sdmmc_Handler(&mciDrv);
}

/*----------------------------------------------------------------------------
 *         Optional: SD card detection (connection, protection)
 *----------------------------------------------------------------------------*/

/**
 * Configure for SD detect pin
 */
static void CardDetectConfigure(void)
{
    PIO_Configure(&pinCardDetect, 1);
    /* No protection detect pin */
}

/**
 * Return 1 if card is insertted.
 */
static uint8_t CardIsConnected(uint8_t slot)
{
    return PIO_Get(&pinCardDetect) ? 0 : 1;
}

/**
 * Return 1 if card is protected.
 */
static uint8_t CardIsProtected(uint8_t slot)
{
    printf("-I- Cannot check if SD card is write-protected\n\r");
    return 0;
}

/**
 * Get Dec Input
 * \param numChar Number of character to wait.
 * \param pInt    Pointer to uint32_t for input result.
 * \return 0 if valid data inputed.
 */
static uint8_t GetDecInput(uint8_t numChar, uint32_t *pInt)
{
    uint8_t key;
    uint32_t  i;
    uint32_t  result = 0;
    for (i = 0; i < numChar;) {
        key = UART_GetChar();
        if (key == 27) {
            printf(" Canceled\n\r");
            return key;
        }
        if (key > '9' || key < '0') continue;
        UART_PutChar(key);
        result = result * 10 + (key - '0');
        i ++;
    }
    if (pInt) *pInt = result;
    return 0;
}

/**
 * Get Delayed number of tick
 * \param startTick Start tick point.
 * \param endTick   End tick point.
 */
static uint32_t GetDelayInTicks(uint32_t startTick, uint32_t endTick)
{
    if (endTick > startTick) return (endTick - startTick);
    return (endTick + (0xFFFFFFFF - startTick));
}

/**
 * \brief Max Error Break
 * Check if max number of error achieved.
 * \param halt Whether halt the device if error number achieved.
 */
static uint8_t MaxErrorBreak(uint8_t halt)
{
    if (NB_ERRORS) {
        if (nbErrors ++ > NB_ERRORS) {

            while(halt);

            nbErrors = 0;
            return 1;
        }
    }
    return 0;
}

/**
 * Display: Dump Splitting row
 */
static void DumpSeperator(void)
{
    printf("\n\r==========================================\n\r");
}

/**
 * Display: Dump main menu
 */
static void DumpMenu(void)
{
    DumpSeperator();
    printf("# i,I   : Re-initialize card\n\r");
    printf("# t     : Disk R/W/Verify test\n\r");
    printf("# T     : Disk performance test\n\r");
    printf("# p     : Change number of blocks in one access for test\n\r");
}

/**
 * Dump buffer
 * \param pData Pointer to data buffer.
 * \param len   Buffer length.
 */
static void DumpBuffer(unsigned char * pData, unsigned int len)
{
    unsigned int i;
    printf("-I- buffer %d: %c .. %c .. %c .. %c..",
            len, pData[0], pData[3], pData[8], pData[8+5]);
    for (i = 0; i < len; i ++) {
        if((i % 16) == 0) printf("\n\r%3x:", i);
        printf(" %02X", pData[i]);
    }
    printf("\n\r");
}

/**
 * Dump block & information
 * \param pData Pointer to data block.
 * \param block Block number.
 */
static void DumpBlock(uint8_t * pData, uint32_t block)
{
    uint32_t i;
    printf("-I- Block %d: %c .. %c .. %c .. %c..",
            (int)block, pData[0], pData[3], pData[8], pData[8+5]);
    for (i = 0; i < 512; i ++) {
        if((i % 16) == 0) printf("\n\r%3x:", (unsigned int)i);
        printf(" %02X", pData[i]);
    }
    printf("\n\r");
}

/**
 * Dump card registers
 * \param slot Card slot (not used now).
 */
static void DumpCardInfo(uint8_t slot)
{
    if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDIO) {
        SDIO_DisplayCardInformation(&sdDrv);
    }

    if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDMMC) {
        SD_DisplayRegisterCID(&sdDrv);
        SD_DisplayRegisterCSD(&sdDrv);
        SD_DisplayRegisterECSD(&sdDrv);
        SD_DisplayRegisterSCR(&sdDrv);
        SD_DisplaySdStatus(&sdDrv);
    }
}

/**
 * Run tests on the inserted card
 * \param slot Card slot (not used now).
 */
static void CardInit(uint8_t slot)
{
    uint8_t error;

    DumpSeperator();

    MCI_Init(&mciDrv, HSMCI, ID_HSMCI, BOARD_MCK );
    error = SD_Init(&sdDrv, &mciDrv);
    if (error) {

        printf("-E- SD/MMC card initialization failed: %d\n\r", error);
        return;
    }
    else {

        printf("-I- SD/MMC card initialization successful\n\r");
        printf("-I- Card size: %d MB", (int)SD_GetTotalSizeKB(&sdDrv)/1000);
        printf(", %d * %dB\n\r", (int)SD_GetNumberBlocks(&sdDrv), (int)SD_GetBlockSize(&sdDrv));
    }

    DumpCardInfo(slot);
}

/**
 * Block Dump (read)
 * \param slot Card slot (not used now).
 */
static void BlockDump(uint8_t slot)
{
    uint32_t block;
    DumpSeperator();
    printf("-!- Input block:");
    if (GetDecInput(5, &block))
        return;
    printf("\n\r-I- Dump Block %d: %d\n\r", (int)block, MMCT_ReadFun(&sdDrv, block, 1, pBuffer));
    DumpBlock(pBuffer, block);
}

/**
 * Sdio test
 * \param slot Card slot.
 */
static void SdioTest(uint8_t slot)
{
    uint32_t i;

    DumpSeperator();

    /* SDIO always has FN1(IEN.1) and Mem(IEN.0), test with these bits */
    printf("R/W Direct test:\n\r");

    printf("CIA:\n\r");
    SDIO_ReadDirect(&sdDrv, SDIO_CIA, 0, &pBuffer[0], 0x14);
    DumpBuffer(pBuffer, 0x14);
    printf("Write 0x03 to IEN(CIA.4): rc %d\n\r",
           SDIO_WriteDirect(&sdDrv, SDIO_CIA, SDIO_IEN_REG, 0x03));
    printf("IEN After Write:");
    SDIO_ReadDirect(&sdDrv, SDIO_CIA, SDIO_IEN_REG, &pBuffer[1], 1);
    printf("0x%02X\n\r", pBuffer[1]);
    if (0x03 == pBuffer[1]) {
        printf("-- test OK\n\r");
    }
    else {
        printf("-- test FAIL\n\r");
    }
    SDIO_WriteDirect(&sdDrv, SDIO_CIA, SDIO_IEN_REG, pBuffer[SDIO_IEN_REG]);

    printf("R/W Extended test:\n\r");
    printf("Dump CIA:\n\r");
    for (i = 0; i < 0x40; i ++) pBuffer[i] = 0xFF; /* Clear Buffer */
    SDIO_ReadBytes(&sdDrv, SDIO_CIA, 0, 0, pBuffer, 0x40, 0, 0);
    DumpBuffer(pBuffer, 0x14);

    printf("Modify Some R/W bytes (2,4) for FN0 and write:\n\r");
    pBuffer[0x2] = 0x2; /* IOE */
    pBuffer[0x4] = 0x2; /* IEN */
    /* Dont write to CIA.0xD or function selection pause transfer */
    SDIO_WriteBytes(&sdDrv, SDIO_CIA, 2, 0, &pBuffer[2], (0xC-2), 0, 0);
    printf("CIA after write:\n\r");
    SDIO_ReadDirect(&sdDrv, SDIO_CIA, 0, pBuffer, 0x14);
    DumpBuffer(pBuffer, 0x14);
    if (pBuffer[0x2] != 0x2) {
        printf("-- CIA.2 Fail\n\r");
    }
    else if (pBuffer[0x4] != 0x2) {
        printf("-- CIA.4 Fail\n\r");
    }
    else {
        printf("-- test OK\n\r");
    }
    /* Restore data to 0 */
    SDIO_WriteDirect(&sdDrv, SDIO_CIA, SDIO_IOE_REG, 0);
    SDIO_WriteDirect(&sdDrv, SDIO_CIA, SDIO_IEN_REG, 0);
}

/**
 * Disk test
 * \param slot Card slot (not used now).
 * \param clr  Do block clear.
 * \param wr   Do block write.
 * \param rd   Do block read.
 */
static void DiskTest(uint8_t slot,
              uint8_t clr,
              uint8_t wr,
              uint8_t rd)
{
    uint8_t error = 0;
    uint32_t i, errcnt = 0;
    uint32_t multiBlock, block, splitMulti;

    DumpSeperator();

    /* Perform tests on each block */
    multiBlock = 0;
    for (block = TEST_BLOCK_START;
         block < TEST_BLOCK_END;
         block += multiBlock) {

        /* Perform different single or multiple bloc operations */
        if (++multiBlock > NB_MULTI_BLOCKS)
            multiBlock = 1;

        /* Multi-block adjustment */
        if (block + multiBlock > TEST_BLOCK_END) {
            multiBlock = TEST_BLOCK_END - block;
        }

        /* ** Perform single block or multi block transfer */
        printf("\r-I- Testing block [%6u - %6u] ...", (int)block, (int)(block + multiBlock -1));

        if (clr) {
            /* - Clear the block */
            memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
            for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
                if (pBuffer[i] != 0) {
                    printf("\n\r-E- Data @ %u for write : 0x00 <> 0x%02x\n\r", (int)i, pBuffer[i]);
                    if(MaxErrorBreak(0)) return;
                    /* Only find first verify error. */
                    continue;
                }
            }
            error = MMCT_WriteFun(&sdDrv, block, multiBlock, pBuffer);
            if (error) {
                printf("\n\r-E- 1. Write block (%d) #%u\n\r", error, (int)block);
                if(MaxErrorBreak(0)) return;
                /* Skip following test */
                continue;
            }
            /* - Read back the data to check the write operation */
            memset(pBuffer, 0xFF, SDMMC_BLOCK_SIZE * multiBlock);
            error = MMCT_ReadFun(&sdDrv, block, multiBlock, pBuffer);
            if (error) {
                printf("\n\r-E- 1. Read block (%d) #%u\n\r", error, (int)block);
                if(MaxErrorBreak(0)) return;
                /* Skip following test */
                continue;
            }
            for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
                if (pBuffer[i] != 0) {
                    printf("\n\r-E- 1. B%u.D[%u] : 0 <> 0x%02X\n\r", (unsigned int)block, (int)i, (int)pBuffer[i]);
                    if(MaxErrorBreak(0)) return;
                    /* Only find first verify error. */
                    break;
                }
            }
        }

        if (wr) {
            /* - Write a checkerboard pattern on the block */
            for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
                if ((i & 1) == 0)  pBuffer[i] = (i & 0x55);
                else               pBuffer[i] = (i & 0xAA);
            }
            for (i = 0; i < multiBlock; ) {
                splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ?
                                        NB_SPLIT_MULTI : (multiBlock - i);
                error = MMCT_WriteFun(&sdDrv,
                                      block + i,
                                      splitMulti,
                                      &pBuffer[i * SDMMC_BLOCK_SIZE]);
                if (error) break;
                i += splitMulti;
            }
            assert( i == multiBlock ) ; //, "Unexpected W, %u!", i);
            if (error)
            {
                printf("\n\r-E- 2. Write block #%u(%u+%u)\n\r",  (unsigned int)(block+i), (unsigned int)block, (unsigned int)i);
                if(MaxErrorBreak(0)) return;
                /* Skip Following Test */
                continue;
            }
        }

        if (rd) {
            /* - Read back the data to check the write operation */
            memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
            for (i = 0; i < multiBlock; ) {
                splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ?
                                        NB_SPLIT_MULTI : (multiBlock - i);
                error = MMCT_ReadFun(&sdDrv,
                                     block + i,
                                     splitMulti,
                                     &pBuffer[i * SDMMC_BLOCK_SIZE]);
                if (error) break;
                i += splitMulti;
            }
            assert( i == multiBlock ) ; // "Unexpected R, %u!", i);
            if (error)
            {
                printf("\n\r-E- 2. Read block #%u(%u+%u)\n\r", (unsigned int)(block + i), (unsigned int)block, (unsigned int)i);
                if(MaxErrorBreak(0)) return;
                /* Skip Following Test */
                continue;
            }
            errcnt = 0;
            for (i=0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {

                if (!(((i & 1) == 0) && (pBuffer[i] == (i & 0x55))) &&
                    !(((i & 1) != 0) && (pBuffer[i] == (i & 0xAA))) ) {
                    uint32_t j, js;
                    printf("\n\r-E- 2.%d. Data @ %u (0x%x)\n\r", (int)errcnt, (unsigned int)i, (unsigned int)i);
                    printf("  -Src:");
                    js = (i > 8) ? (i - 8) : 0;
                    for (j = js; j < i + 8; j ++)
                        printf(" %02x", (unsigned int)(((j & 1)!= 0) ? (j & 0xAA):(j & 0x55)));
                    printf("\n\r  -Dat:");
                    for (j = js; j < i + 8; j ++)
                        printf("%c%02x", (i == j) ? '!' : ' ', pBuffer[j]);
                    printf("\n\r");
                    if(MaxErrorBreak(0)) return;
                    // Only find first 3 verify error.
                    if (errcnt ++ >= 3)
                        break;
                }
            }
        }

        if (UART_IsRxReady()) {
            switch(UART_GetChar()) {
                /* Skip 100M */
                case 'k':
                    block += TEST_BLOCK_SKIP;
                    if (block > TEST_BLOCK_END) {
                        block -= 5 + multiBlock;
                    }
                    printf("\n\r");
                    break;
                /* Cancel */
                case 'c':
                    return;
            }
        }
    }

    printf("All block tested!\n\r");
}

/**
 * Run performence test
 * R/W test can be masked to verify previous written data only
 * \param slot Card slot (not used now).
 * \param wr   Do block write.
 * \param rd   Do block read.
 * \param errDetail Dump detailed error information.
 */
static void PerformanceTest(uint8_t slot,
                     uint8_t wr,
                     uint8_t rd,
                     uint8_t errDetail)
{
    uint8_t error = 0;
    uint32_t  block, i, nBadBlock = 0, nErrors;
    uint32_t  tickStart, tickEnd, ticks, rwSpeed;

    DumpSeperator();
    printf("-I- Performence test, size %dK, Multi %d, MCK %dMHz\n\r",
                                    TEST_PERFORMENCT_SIZE/1024,
                                    (int)performanceMultiBlock,
                                    (int)(BOARD_MCK/1000000));
    printf("-I- RW block by block, block size %d\n\r", (int)SDMMC_BLOCK_SIZE);

    if (wr) {
        printf("--- Write test .. ");
        for (i = 0; i < SDMMC_BLOCK_SIZE * performanceMultiBlock; i += 4) {
            *(uint32_t*)&pBuffer[i] = TEST_FILL_VALUE_U32;
        }
        nBadBlock = 0;
        tickStart = GetTickCount();
        for (block = TEST_BLOCK_START;
             block < (TEST_PERFORMENCT_SIZE/SDMMC_BLOCK_SIZE)
                        + TEST_BLOCK_START;
             block += performanceMultiBlock) {

            *(uint32_t*)pBuffer = block;
            error = MMCT_WriteFun(&sdDrv,
                                  block, performanceMultiBlock,
                                  pBuffer);
            if (error) {
                if (nBadBlock ++ >= NB_BAD_BLOCK) {
                    printf("-E- WR_B(%u)\n\r", (unsigned int)block);
                    break;
                }
                else error = 0;
            }
        }
        tickEnd = GetTickCount();
        ticks = GetDelayInTicks(tickStart, tickEnd);
        rwSpeed = (TEST_PERFORMENCT_SIZE
                    - nBadBlock * performanceMultiBlock * SDMMC_BLOCK_SIZE)
                         / ticks;
        printf("Done, Bad %u, Speed %uK\n\r", (unsigned int)nBadBlock, (unsigned int)rwSpeed);
    }

    if (rd) {
        printf("--- Read test .. ");
        nBadBlock = 0;
        tickStart = GetTickCount();
        for (block = TEST_BLOCK_START;
             block < (TEST_PERFORMENCT_SIZE/SDMMC_BLOCK_SIZE)
                        + TEST_BLOCK_START;
             block += performanceMultiBlock) {

            error = MMCT_ReadFun(&sdDrv,
                                 block, performanceMultiBlock,
                                 pBuffer);
            if (error) {
                if (nBadBlock ++ >= NB_BAD_BLOCK) {
                    printf("-E- RD_B(%u)\n\r", (unsigned int)block);
                    break;
                }
                else error = 0;
            }
            if (error) break;
        }
        tickEnd = GetTickCount();
        ticks = GetDelayInTicks(tickStart, tickEnd);
        rwSpeed = (TEST_PERFORMENCT_SIZE
                    - nBadBlock * performanceMultiBlock * SDMMC_BLOCK_SIZE)
                         / ticks;
        printf("Done, Bad %u, Speed %uK\n\r", (unsigned int)nBadBlock, (unsigned int)rwSpeed);
    }

    printf("--- Data verify .. ");
    nErrors = 0;
    for (block = TEST_BLOCK_START;
         block < (TEST_PERFORMENCT_SIZE/SDMMC_BLOCK_SIZE) + TEST_BLOCK_START;
         block += performanceMultiBlock) {

        memset(pBuffer, 0x00, SDMMC_BLOCK_SIZE * performanceMultiBlock);
        error = MMCT_ReadFun(&sdDrv,
                             block, performanceMultiBlock,
                             pBuffer);
        if (error) {
            printf("-E- RD_B(%u)\n\r", (unsigned int)block);
            break;
        }
        if (*(uint32_t*)pBuffer != block) {
            if (errDetail) {
                if (nErrors ++ < NB_ERRORS) {
                    printf("-E- Blk(%u)[0](%08x<>%08x)\n\r", (unsigned int)block, (unsigned int)block, (unsigned int)(*(uint32_t*)pBuffer));
                }
            }
            else {
                printf("-E- BlkN(%x<>%x)\n\r", (unsigned int)block, (unsigned int)(*(uint32_t*)pBuffer));
                error = 1;
                break;
            }
        }
        for (i = 4; i < SDMMC_BLOCK_SIZE * performanceMultiBlock; i += 4) {
            if ( (*(uint32_t*)&pBuffer[i]) != TEST_FILL_VALUE_U32) {
                if (errDetail) {
                    /* Dump 10 errors only */
                    if (nErrors ++ < NB_ERRORS) {
                        uint32_t j;
                        printf("-E- Blk(%u)[%u](%08x.. <>", (unsigned int)block, (unsigned int)i, (unsigned int)TEST_FILL_VALUE_U32);
                        for (j = (i > 4) ? (i - 4) : i;
                             j <= i + 4;
                             j += 4) {
                            printf("%c%08X",
                                    (i == j) ? '!' : ' ',
                                    (unsigned int)(*(uint32_t*)&pBuffer[j]));
                        }
                        printf(")\n\r");
                    }
                }
                else {
                printf("-E- Blk(%u)[%u](%x<>%x)\n\r", (unsigned int)block, (unsigned int)i,
                        (unsigned int)TEST_FILL_VALUE_U32,
                        (unsigned int)(*(uint32_t*)&pBuffer[i]));
                error = 1;
                break;
                }
            }
        }
        if (error) break;
    }
    if (errDetail && nErrors) {
        printf("-I- %u u32 ERRORS found!\n\r", (unsigned int)nErrors);
    }
    if (error)
        return;
    printf("OK\n\r");
}

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Accumulate time_stamp.
 */
void SysTick_Handler( void )
{
    TimeTick_Increment() ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief hsmci_multimedia_card Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    uint8_t connected = 0;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information*/
    printf("-- Basic MultiMedia Card Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
    printf("-- Buffer@%x,size 0x%x\n\r", (unsigned int)pBuffer, sizeof(pBuffer));

    /* Initialize system tick */
    TimeTick_Configure(BOARD_MCK);

    /* Configure SDcard pins */
    PIO_Configure(pinsSd, PIO_LISTSIZE(pinsSd));
    /* Configure SD card detection */
    CardDetectConfigure();
    /* Check if card is write-protected (if supported) */
    CardIsProtected(0);

    /* Initialize the HSMCI driver */
    MCI_Init(&mciDrv, HSMCI, ID_HSMCI, BOARD_MCK ) ;
    NVIC_EnableIRQ(HSMCI_IRQn);

    /* Card insert detection loop */
    for(;;) {

        if (CardIsConnected(0)) {
            if (connected == 0) {
                connected = 1;
                /* Delay before card initialize */
                Wait(400);
                /* Do card test */
                CardInit(0);
            }
        }
        else if (connected) {
            connected = 0;
            printf("** Card Disconnected\n\r");
        }

        if (connected) {
            if (UART_IsRxReady()) {
                switch(UART_GetChar()) {
                    /* Dump block contents */
                    case 'd':               BlockDump(0);  break;
                    /* Initialize the card again */
                    case 'I':  case 'i':    CardInit(0);   break;
                    /* Run test on whole disk */
                    case 't':
                        if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDIO)
                            SdioTest(0);
                        if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDMMC)
                            DiskTest(0, 1, 1, 1);
                        printf("\n\r");
                        break;
                    /* Run performence test */
                    case 'T':
                        PerformanceTest(0, 1, 1, 0);
                        printf("\n\r");
                        break;
                    /* Read/Verify ONLY test */
                    case 'v':
                        DiskTest(0, 0, 0, 1);
                        printf("\n\r");
                        break;
                    /* Read/Verify ONLY performance test */
                    case 'V':
                        PerformanceTest(0, 0, 1, 1);
                        printf("\n\r");
                        break;
                    /* Change performance test block size */
                    case 'p':
                    {   if (performanceMultiBlock >= NB_MULTI_BLOCKS)
                            performanceMultiBlock = 1;
                        else
                            performanceMultiBlock <<= 1;
                        printf("-!- Performance Multi set to %d\n\r", (int)performanceMultiBlock);
                    }
                        break;
                    /* Show help information */
                    default:    DumpMenu();
                }
            }
        }
    }
}

