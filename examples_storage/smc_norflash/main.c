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
 * \page smc_norflash SMC NOR Flash Example
 *
 * \section Purpose
 *
 * The smc_norflash project gives you CFI supported %Norflash programming
 * through the External Bus Interfac (EBI), so that can help develop your own
 * norflash devices applications with maximum efficiency.
 *
 * You can find following information depends on your needs:
 * - Configures the EBI for %NorFlash access.
 * - Usage of auto detection of CFI supported  %Norflash device.
 * - API layer consists of several functions that allow user to
 *   do operations with %Norflash in a unified way.
 * - Low-level driver implement procedures to program basic operations
 *   described in the datasheets for %Norflash devices.
 * - Sample code for accessing %Norflash device.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits with external
 * %Norflash components.
 *
 * \section Description
 *
 * At startup, the program configures the SMC to access the %NorFlash and tries
 * to identify it by CFI detectiion. If it succeed, it retrieves its parameter
 * and starts testing its blocks. Each block is first erased, then all its pages
 * are written and verified.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6421.pdf">SAM-BA User Guide</a>,
 *    the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
 *    application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
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
 *     -- SMC NorFlash Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Common Flash Interface detecting...
 *     -I- CFI detected and driver initialized
 *     -I- manufactureID : 0xxxxx, deviceID : 0xxxxx
 *     -I- Test in progress on block: xxxx
 *    \endcode
 * -# Eventually, the test result (pass or fail) will be output on the terminal.
 *
 * \section References
 * - smc_norflash/main.c
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "memories.h"

#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local macros
 *----------------------------------------------------------------------------*/

#define min( a, b ) (((a) < (b)) ? (a) : (b))

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Norflash device structure. */
static NorFlash norFlash ;

/** Pins to configure for the application */
#ifdef PINS_NORFLASH
static const Pin pPins[] =
{
   PINS_NORFLASH
};
#endif

/** Temporary buffer for unaligned read/write operations. */
static uint8_t pBuffer[1024];

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Tests the norflash connected to the board by performing several
 * command on each of its pages.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    uint32_t blockNumber, block, blockSize, blockAddress, pageSize, packetSize, i;
    uint8_t testFailed;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("\n\r\n\r\n\r");
    printf("-- SMC_NorFlash Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure pins */
#ifdef PINS_NORFLASH
    PIO_Configure(pPins, PIO_LISTSIZE(pPins));
#endif

    /* Check device CFI and get Vendor setting from it. */
    norFlash.norFlashInfo.baseAddress = BOARD_NORFLASH_ADDR;
    printf("Common Flash Interface detecting...\n\r");
    BOARD_ConfigureNorFlash(SMC);
    NorFlash_CFI_Detect(&norFlash, FLASH_CHIP_WIDTH_8BITS);

    if (norFlash.norFlashInfo.cfiCompatible == 0) {
        printf("Device Unknown\n\r");
        testFailed = 1;
        goto exit;
    }

    printf("CFI detected and driver initialized\n\r");
    printf("manufactureID : 0x%08x, deviceID : 0x%08x\n\r",
            (unsigned int)NORFLASH_ReadManufactoryID(&norFlash),
            (unsigned int)NORFLASH_ReadDeviceID(&norFlash));

    /* Test all pages */
    testFailed = 0;
    block = 0;
    blockNumber = NorFlash_GetDeviceNumOfBlocks(&(norFlash.norFlashInfo));
    pageSize = min(NorFlash_GetDeviceMinBlockSize(&(norFlash.norFlashInfo)), 1024);

    while (!testFailed && (block < blockNumber)) {

        printf("Test in progress on block: %6d\r", (int)block);
        /* Erase block */
        NORFLASH_EraseSector(&norFlash, NorFlash_GetDeviceSectorAddress(&(norFlash.norFlashInfo), block));

        blockSize =  NorFlash_GetDeviceBlockSize(&(norFlash.norFlashInfo), block);
        blockAddress = NorFlash_GetDeviceSectorAddress(&(norFlash.norFlashInfo), block);
        packetSize = pageSize;

        while (blockSize) {
            /* Verify that page has been erased correctly */
            memset(pBuffer, 0, packetSize);
            NORFLASH_ReadData(&norFlash, blockAddress, pBuffer, packetSize);
            for (i=0; i < packetSize; i++) {

                if (pBuffer[i] != 0xFF) {
                    printf("Could not erase block %d\n\r", (int)block);
                    testFailed = 1;
                    goto exit;
                }
            }

            blockAddress += packetSize;
            blockSize -= packetSize;
            if (blockSize < pageSize) {
                 packetSize = blockSize;
            }
        }

        blockSize =  NorFlash_GetDeviceBlockSize(&(norFlash.norFlashInfo), block);
        blockAddress = NorFlash_GetDeviceSectorAddress(&(norFlash.norFlashInfo), block);
        packetSize = pageSize;

        while (blockSize) {
            /* Write page */
            for (i = 0; i < packetSize; i++) {
                pBuffer[i] = i & 0xFF;
            }
            NORFLASH_WriteData(&norFlash, blockAddress, pBuffer, packetSize);
            /* Check that data has been written correctly */
            memset(pBuffer, 0, packetSize);
            NORFLASH_ReadData(&norFlash, blockAddress, pBuffer, packetSize);

            for (i = 0; i < packetSize; i++) {
                if (pBuffer[i] != (i & 0xFF)) {
                    printf("Could not write block %d\n\r", (int)block);
                    testFailed = 1;
                    goto exit;
                }
            }
            blockAddress += packetSize;
            blockSize -= packetSize;
            if (blockSize < pageSize) {
                 packetSize = blockSize;
            }
        }
        block++;
    }
exit:
    /* Display test result */
    if (testFailed) {

        printf("Test failed.\n\r");
    }
    else {

         printf("Test passed.\n\r");
    }

    return 0;
}

