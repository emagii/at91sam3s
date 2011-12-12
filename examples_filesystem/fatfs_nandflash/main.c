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
 *  \page fatfs_nandflash FATFS Nandflash Example
 *
 *  \section Purpose
 *
 *  The FATFS Nandflash Example will help you to get familiar with
 *  filesystem on sam3s.
 *  It supplies sample code of common operations of a filesystem through a NAND
 *  FLASH based filesystem.
 *  It can also help you to configure the filesystem according to your own
 *  needs, which can be used for fast implementation of your own filesystem and
 *  other applications related.
 *  FatFs is a generic file system module to implement the FAT file system to
 *  small embedded systems.
 *
 *  \section See
 *
 *  <a href="http://elm-chan.org/fsw/ff/00index_e.html">
 *  FATFs Website</a>
 *  <a href="../../../external_libs/fat/fatfs/doc/00index_e.html">
 *  FAT File System Module </a>
 *
 *  You can find following information depends on your needs:
 *  - Supply a set of file system interface for user to do file related work,
 *    e.g. mount a disk, initialize a disk, create/open a file, write/read a
 *    file e.t.c
 *  - Supply a set of disk level interface for user's reference, which is
 *    called by filesystem level
 *  - Teaches user how to use these interfaces through sample code
 *  - Configuration of a filesystem, e.g. user can build up a tiny filesystem
 *    which consume low memory, or a filesystem with read-only functions
 *
 *  \section Requirements
 *
 *  This package can be used with sam3s-ek which has NAND FLASH device.
 *
 *  \section Description
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download this
 *  program to flash , make the program run, the HyperTerminal will give
 *  out the test results.
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
 *     \code
 *     -- FatFS Full Version with NAND Example xxx --
 *     -- SAMxxx
 *     -- Compiled: xxx --
 *     \endcode
 *
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the fatfs_nandflash example
 *
 *  \section Contents
 *  The fatfs_nandflash application can be roughly broken down as
 *  follows:
 *     - The main function, which implements the program behavior
 *        - NAND Flash initialize
 *        - Mount a disk
 *        - Initialize a disk in NAND Flash
 *        - Create a new file
 *        - Write something to a opened file
 *        - Close a file
 *        - Open an existing file
 *        - Read a file
 *        - Close a file
 *        - Compare the reading result, print the compare result to DBGU
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "memories.h"

#include "fatfs_config.h"

/*----------------------------------------------------------------------------
 *        Local constants
 *----------------------------------------------------------------------------*/

/** Maximum number of Medias which can be defined. */
#define MAX_MEDS        1

/** Size of buffer used for create and test a file */
#define DATA_SIZE       512

/** Size of the reserved Nand Flash (4M) */
#define NF_RESERVE_SIZE     (4*1024*1024)

/** Size of the managed Nand Flash (128M) */
#define NF_MANAGED_SIZE     (128*1024*1024)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

#if _FS_TINY == 0
#define STR_ROOT_DIRECTORY "0:"
#else
#define STR_ROOT_DIRECTORY ""
#endif

/** Available medias. */
Media medias[MAX_MEDS];

/** File name */
const char* FileName = STR_ROOT_DIRECTORY "Basic.bin";

/** Buffer for create and use a file */
uint8_t data[DATA_SIZE];

/** Pins used to access to nandflash. */
static const Pin pPinsNf[] = {PINS_NANDFLASH};
/** Nandflash device structure. */
static struct TranslatedNandFlash translatedNf;
/** Address for transferring command bytes to the nandflash. */
static uint32_t cmdBytesAddr = BOARD_NF_COMMAND_ADDR;
/** Address for transferring address bytes to the nandflash. */
static uint32_t addrBytesAddr = BOARD_NF_ADDRESS_ADDR;
/** Address for transferring data bytes to the nandflash. */
static uint32_t dataBytesAddr = BOARD_NF_DATA_ADDR;
/** Nandflash chip enable pin. */
static const Pin nfCePin = BOARD_NF_CE_PIN;
/** Nandflash ready/busy pin. */
static const Pin nfRbPin = BOARD_NF_RB_PIN;

/*----------------------------------------------------------------------------
 *        Local Functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Wait DEBUG Key Input
 *  \param ms   Wait time in ms.
 *  \param echo Wether echo the key.
 *  \return key or 0 for nothing.
 */
static uint8_t WaitKey(volatile uint32_t ms, bool echo)
{
    uint8_t key;
    uint32_t tick = GetTickCount();
    do {

        if (UART_IsRxReady()) {

            key = UART_GetChar();
            if (echo) UART_PutChar(key);
            return key;
        }

        /* ms > 0, check tick */
        if (ms && (GetTickCount() - tick > ms)) {
            break;
        }
    } while (1);

    return 0;
}

/**
 *  \brief Scan Files on Disk
 */
static FRESULT scan_files (char* path)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif


    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (*fn == '.') continue;
            if (fno.fattrib & AM_DIR) {
                sprintf(&path[i], "/%s", fn);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
                printf("%s/%s\n\r", path, fn);
            }
        }
    }

    return res;
}

/**
 *  \brief Initialize Nand Flash
 *  \return true if initialized succesfully.
 */
static uint8_t NandFlashInitialize(void)
{
    uint8_t  nfRc;
    uint16_t nfBaseBlock = 0;
    struct RawNandFlash *pRaw = (struct RawNandFlash*)&translatedNf;
    struct NandFlashModel *pModel = (struct NandFlashModel*)&translatedNf;
    uint32_t nfManagedSize;

    /* Configure for NandFlash */
    BOARD_ConfigureNandFlash(SMC);
    /* Configure PIO for Nand Flash */
    PIO_Configure(pPinsNf, PIO_LISTSIZE(pPinsNf));

    /* Nand Flash Initialize (ALL flash mapped) */
    nfRc = RawNandFlash_Initialize(pRaw,
                                   0,
                                   cmdBytesAddr,
                                   addrBytesAddr,
                                   dataBytesAddr,
                                   nfCePin,
                                   nfRbPin);
    if ( nfRc )
    {
        printf("Nand not found\n\r");
        return false;
    }
    else
    {
        printf("NF\tNb Blocks %d\n\r", NandFlashModel_GetDeviceSizeInBlocks(pModel));
        printf("\tBlock Size %dK\n\r", NandFlashModel_GetBlockSizeInBytes(pModel)/1024);
        printf("\tPage Size %d\n\r", NandFlashModel_GetPageDataSize(pModel));
        nfBaseBlock = NF_RESERVE_SIZE / NandFlashModel_GetBlockSizeInBytes(pModel);
    }
    printf("NF disk will use area from %dM(B%d)\n\r", NF_RESERVE_SIZE/1024/1024, nfBaseBlock);

    /* Wait 1.2s for input */
    printf("!! Erase the NF Disk? (y/n):");

    if ( WaitKey( 1200, 0 ) == 'y' )
    {
        if ( nfRc == 0 )
        {
            uint32_t block;
            printf(" Erase from %d ... ", nfBaseBlock ) ;
            for ( block = nfBaseBlock ; block < NandFlashModel_GetDeviceSizeInBlocks(pModel); block ++ )
            {
                RawNandFlash_EraseBlock(pRaw, block);
            }
            printf("OK");
        }
    }
    printf("\n\r");

    nfManagedSize = ((NandFlashModel_GetDeviceSizeInMBytes(pModel) - NF_RESERVE_SIZE/1024/1024) > NF_MANAGED_SIZE/1024/1024) ? \
                        NF_MANAGED_SIZE/1024/1024 : (NandFlashModel_GetDeviceSizeInMBytes(pModel) - NF_RESERVE_SIZE/1024/1024);
    if (TranslatedNandFlash_Initialize(&translatedNf,
                                       0,
                                       cmdBytesAddr,
                                       addrBytesAddr,
                                       dataBytesAddr,
                                       nfCePin,
                                       nfRbPin,
                                       nfBaseBlock, nfManagedSize * 1024 * 1024/NandFlashModel_GetBlockSizeInBytes(pModel))) {
        printf("Nand init error\n\r");
        return false;
    }
    /* Media initialize */
    MEDNandFlash_Initialize(&medias[DRV_NAND], &translatedNf);

    return true;
}

/**
 *  \brief Application entry point for FATFS Nand Flash Example
 */
int main( void )
{
    uint8_t key;
    uint32_t i;
    uint32_t ByteToRead;
    UINT ByteRead;
    UINT ByteWritten;

    FATFS fs;             // File system object
    DIR dirs;
    FIL FileObject;
    FRESULT res;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */

#if _FS_TINY != 1
    printf("-- FatFS Full Version with NAND Example %s --\n\r", SOFTPACK_VERSION);
#else
    printf("-- FatFS Tiny Version with NAND Example %s --\n\r", SOFTPACK_VERSION);
    printf("(Full Version has to be launched before to pass test)\n\r");
#endif
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


    /* Init NandFlash Disk */
    if (!NandFlashInitialize()) {
        printf("-F- NF Init FAIL\n\r");
        return 0;
    }

    /* Mount Disk */
    printf("-I- Mount disk 0\n\r");
    memset(&fs, 0, sizeof(FATFS));  // Clear file system object
    res = f_mount(0, &fs);
    if( res != FR_OK ) {
        printf("-F- f_mount pb\n\r");
        return 0;
    }

    /* Test if the disk is formated */
    res = f_opendir (&dirs,STR_ROOT_DIRECTORY);
    if(res == FR_OK )
    {

        /* erase NAND to re-format it ? */
        printf("-I- The disk is already formated.\n\r");

        /* Display the file tree */
        printf("-I- Display files contained on the NAND :\n\r");
        scan_files((char*)STR_ROOT_DIRECTORY);

        printf("-I- Erase the NAND to re-format disk ? (y/n)!\n\r");

        /* Wait 1.5s for input */
        key = WaitKey(1500, 0);
        if( (key == 'y') ||  (key == 'Y'))
        {
            TranslatedNandFlash_EraseAll(&translatedNf ,NandEraseDATA);
            res = FR_NO_FILESYSTEM;
        }
    }

#if _FS_TINY != 1
    /* Format disk */
    if (res == FR_NO_FILESYSTEM)
    {
        printf("-I- Format disk 0\n\r");
        printf("-I- Please wait a moment during formating...\n\r");
        res = f_mkfs(0,    /* Drv */
                     0,    /* FDISK partition */
                     512); /* AllocSize */
        printf("-I- Format disk finished !\n\r");
        if( res != FR_OK ) {
            printf("-E- f_mkfs 0x%Xpb\n\r", res);
            return 0;
        }
    }

    /* Create a new file */
    printf("-I- Create a file : \"%s\"\n\r", FileName);
    res = f_open(&FileObject, FileName, FA_CREATE_ALWAYS|FA_WRITE);
    if( res != FR_OK )
    {
        printf("-E- f_open create pb: 0x%X \n\r", res);
        return 0;
    }

    /* Write a checkerboard pattern in the buffer */
    for (i=0; i < sizeof(data); i++)
    {
        if ((i & 1) == 0)
        {
            data[i] = (i & 0x55);
        }
        else
        {
            data[i] = (i & 0xAA);
        }
    }
    printf("-I- Write file\n\r");
    res = f_write(&FileObject, data, DATA_SIZE, &ByteWritten);
    printf("-I- ByteWritten=%d\n\r", (int)ByteWritten);
    if( res != FR_OK )
    {
        printf("-E- f_write pb: 0x%X\n\r", res);
        return 0;
    }
    else
    {
        printf("-I- f_write ok: ByteWritten=%d\n\r", (int)ByteWritten);
    }

    /* Close the file */
    printf("-I- Close file\n\r");
    res = f_close(&FileObject);
    if( res != FR_OK )
    {
        printf("-E- f_close pb: 0x%X\n\r", res);
        return 0;
    }
#else
    printf("-E- Format not supported in Tiny version!\n\r");
    return 0;
#endif

    /* Open the file */
    printf("-I- Open file : %s\n\r", FileName);
    res = f_open(&FileObject, FileName, FA_OPEN_EXISTING|FA_READ);
    if( res != FR_OK )
    {
        printf("-E- f_open read pb: 0x%X\n\r", res);
        return 0;
    }

    assert( FileObject.fsize == DATA_SIZE ) ; /* "File size value not expected!\n\r"); */

    /* Read file */
    printf("-I- Read file\n\r");
    memset(data, 0, DATA_SIZE);
    ByteToRead = FileObject.fsize;

    res = f_read(&FileObject, data, ByteToRead, &ByteRead);
    if(res != FR_OK) {
        printf("-E- f_read pb: 0x%X\n\r", res);
        return 0;
    }
    assert( FileObject.fsize == DATA_SIZE ) ; /* "File size value not expected!\n\r" */

    /* Close the file */
    printf("-I- Close file\n\r");
    res = f_close(&FileObject);
    if( res != FR_OK )
    {
        printf("-E- f_close pb: 0x%X\n\r", res);
        return 0;
    }

    /* compare read data with the expected data */
    for ( i=0 ; i < sizeof( data ) ; i++ )
    {
        assert ((((i & 1) == 0) && (data[i] == (i & 0x55))) || (data[i] == (i & 0xAA)) ) ;
                   /* \n\r-F- Invalid data at data[%u] (expected 0x%02X, read 0x%02X)\n\r", i, ((i & 1) == 0) ? (i & 0x55) : (i & 0xAA), data[i]); */
    }
    printf("-I- File data Ok !\n\r");

    printf("-I- Test passed !\n\r");

    return 0;
}

