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
 * \page smc_nandflash SMC NAND Flash Example
 *
 * \section Purpose
 *
 * This basic nandflash example shall show how to read and write data from/to
 * a nandflash connected to the SMC, taking ECC and Bad Block marking into account
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * The required steps are:
 * <ul>
 * <li> Configure the SMC to interface with the NAND Flash. </li>
 * <li> Read a page.</li>
 * <li> Calculate the ECC by software and check it is correct.</li>
 * <li> Prepare a buffer and calculate the ECC by software.</li>
 * <li> Write the buffer into a NAND flash page and store the ECC.</li>
 * <li> Read the page and check that ECC is correct.</li>
 * </ul>
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
 *     \code
 *     -- SMC NandFlash Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Nandflash ID is 0x1580DA2C
 *     -I-     Nandflash driver initialized
 *     -I- Size of the whole device in bytes : 0x10000000
 *     -I- Size in bytes of one single block of a device : 0x20000
 *     -I- Number of blocks in the entire device : 0x800
 *     -I- Size of the data area of a page in bytes : 0x800
 *     -I- Number of pages in the entire device : 0x40
 *    \endcode
 *
 * \section References
 * - smc_nandflash/main.c
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <memories.h>

#include <string.h>

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/
/** Nandflash memory size. */
static unsigned int memSize;
/** Number of blocks in nandflash.*/
static unsigned int numBlocks;
/** Size of one block in the nandflash, in bytes.*/
static unsigned int blockSize;
/** Size of one page in the nandflash, in bytes.*/
static unsigned int pageSize;
/** Number of page per block*/
static unsigned int numPagesPerBlock;


/** Pins used to access to nandflash.*/
static const Pin pPinsNf[] = {PINS_NANDFLASH};
/** Nandflash device structure.*/
static struct SkipBlockNandFlash skipBlockNf;
/** Address for transferring command bytes to the nandflash.*/
static unsigned int cmdBytesAddr = BOARD_NF_COMMAND_ADDR;
/** Address for transferring address bytes to the nandflash.*/
static unsigned int addrBytesAddr = BOARD_NF_ADDRESS_ADDR;
/** Address for transferring data bytes to the nandflash.*/
static unsigned int dataBytesAddr = BOARD_NF_DATA_ADDR;
/** Nandflash chip enable pin.*/
static const Pin nfCePin = BOARD_NF_CE_PIN;
/** Nandflash ready/busy pin.*/
static const Pin nfRbPin = BOARD_NF_RB_PIN;

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
extern int main( void )
{
    /* Temporary buffer */
    unsigned char pageBuffer[2048];
    unsigned char readBuffer[2048];
    unsigned short block, page;
    unsigned int i;
    /* Errors returned by SkipNandFlash functions */
    unsigned char error = 0;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- SMC NandFlash Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure SMC for Nandflash accesses */
    BOARD_ConfigureNandFlash(SMC);
    PIO_Configure(pPinsNf, PIO_LISTSIZE(pPinsNf));

    memset(&skipBlockNf, 0, sizeof(skipBlockNf));

    if (SkipBlockNandFlash_Initialize(&skipBlockNf,
                                0,
                                cmdBytesAddr,
                                addrBytesAddr,
                                dataBytesAddr,
                                nfCePin,
                                nfRbPin)) {

        printf("-E- Device Unknown\n\r");
        return 0;
    }

    printf("-I- Nandflash driver initialized\n\r");

    /* Get device parameters */
    memSize = NandFlashModel_GetDeviceSizeInBytes(&skipBlockNf.ecc.raw.model);
    blockSize = NandFlashModel_GetBlockSizeInBytes(&skipBlockNf.ecc.raw.model);
    numBlocks = NandFlashModel_GetDeviceSizeInBlocks(&skipBlockNf.ecc.raw.model);
    pageSize = NandFlashModel_GetPageDataSize(&skipBlockNf.ecc.raw.model);
    numPagesPerBlock = NandFlashModel_GetBlockSizeInPages(&skipBlockNf.ecc.raw.model);

    printf("-I- Size of the whole device in bytes : 0x%x \n\r",memSize);
    printf("-I- Size in bytes of one single block of a device : 0x%x \n\r",blockSize);
    printf("-I- Number of blocks in the entire device : 0x%x \n\r",numBlocks);
    printf("-I- Size of the data area of a page in bytes : 0x%x \n\r",pageSize);
    printf("-I- Number of pages in the entire device : 0x%x \n\r",numPagesPerBlock);

    block = 12;
    page = 6;

    /* Read a page */
    printf("-I- Read page %d of block %d with ECC check.\n\r", page, block);
    error = SkipBlockNandFlash_ReadPage(&skipBlockNf, block, page, pageBuffer, 0);
    if (error)
    {
        printf("-E- Read page %d of block %d failed.\n\r", page, block);
        if ( error == NandCommon_ERROR_BADBLOCK )
        {
            printf("-E- Block %d is BAD block. \n\r",  block);
            printf("-E- Test terminate, try another block. \n\r");
        }
        else
        {
            if (error == NandCommon_ERROR_CORRUPTEDDATA)
            {
                printf("-E- Block have %d. page %d unrecoverable data\n\r", block, page);
            }
        }

        return error;
    }

    /* Erase block */
    error = SkipBlockNandFlash_EraseBlock(&skipBlockNf, block, NORMAL_ERASE);

    if ( error == NandCommon_ERROR_BADBLOCK )
    {
        printf("-E- Block %d is BAD block. \n\r",  block);
        printf("-E- Test terminate, try another block. \n\r");
        return error;
    }

    /* Prepare a page size buffer in SRAM.*/
    printf("-I- Preparing a buffer in SRAM ...\n\r");
    for ( i = 0; i < pageSize; i++ )
    {
        pageBuffer[i] = i & 0xFF;
    }

    /* Reset read buffer.*/
    memset( readBuffer, 0, sizeof( readBuffer ) ) ;

    /* ECC is calculated by software, it writes the buffer into a NAND flash page and stores the ECC in spare area. */
    printf("-I- Write the buffer in page %d of block %d with ECC stored in page spare area.\n\r", page, block);
    error = SkipBlockNandFlash_WritePage(&skipBlockNf, block, page, pageBuffer, 0);
    if ( error )
    {
        printf("-E- Cannot write page %d of block %d.\n\r", page, block);
        return error;
    }

    /* Read the page and check that ECC is correct. */
    printf("-I- Read page %d of block %d with ECC check.\n\r", page, block);
    error = SkipBlockNandFlash_ReadPage(&skipBlockNf, block, page, readBuffer, 0);
    if ( error )
    {
        printf("-E- Read page %d of block %d failed.\n\r", page, block);
        if ( error == NandCommon_ERROR_CORRUPTEDDATA )
        {
            printf("-E- Block have %d. page %d unrecoverable data\n\r", block, page);
        }
        return error;
    }
    /* Test if the read buffer is the same as SRAM buffer */
    error = memcmp(readBuffer, pageBuffer, sizeof(pageBuffer));
    if ( error )
    {
        printf("-I- Read data is different from SRAM buffer.\n\r");
        printf("-I- Test KO.\n\r");

        return 0;
    }
    printf("-I- Read data matches SRAM buffer.\n\r");

    printf("-I- Test passed.\n\r");
    return 0;
}

