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
 * \page hsmci_sdio Basic SDIO Card Example
 *
 * \section Purpose
 *
 *  The hsmci_sdio will help you to get familiar with sdmmc_mci interface on
 *  SAM microcontrollers. It can also help you to get familiar with the SDIO
 *  operation flow which can be used for fast implementation of your own SD
 *  drivers and other applications related.
 *
 *  You can find following information depends on your needs:
 *  - Usage of auto detection of sdcard insert and sdcard write-protection
 *    detection
 *  - MCI interface initialize sequence and interrupt installation
 *  - Sdio driver implementation based on MCI interface
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
 *  Open HyperTerminal before running this program, use SAM-BA to download
 *  this program to SRAM or Flash, make the program run, the HyperTerminal
 *  will give out the test results.
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
 *      -- Basic HSMCI SDIO Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Please connect a SD card ...
 *      -I- SD card connection detected
 *      -I- Cannot check if SD card is write-protected
 *      -I- SD/MMC card initialization successful
 *      R/W Direct test:
 *      ...
 *      \endcode
 *  -# After card inserted following commands can be used:
 *     - 'f' Change current function number
 *     - 'r' Dump SDIO register value
 *     - 'w' Write to SDIO register
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
 *  The hsmci_sdio application can be roughly broken down as follows:
 *     - Optional functions
 *        - CardDetectConfigure
 *        - CardIsConnected
 *     - Interrupt handlers
 *        - MCI_IrqHandler
 *     - The main function, which implements the program behavior
 *        - I/O configuration
 *        - sd card auto-detect (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - Test RW_DIRECT and RW_EXTENDED at SDIO CIA area.
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
static uint8_t pBuffer[SDMMC_BLOCK_SIZE];

/** Current function */
static uint8_t curFunc = 0;

/*----------------------------------------------------------------------------
 *         Local macros
 *----------------------------------------------------------------------------*/

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

/*----------------------------------------------------------------------------
 *         Optional: SD card detection
 *----------------------------------------------------------------------------*/

/**  SD card detection pin instance. */
static const Pin pinCardDetect = BOARD_SD_PIN_CD;

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
 * Display: Dump Splitting row
 */
static void DumpSeperator(void)
{
    printf("\n\r==========================================\n\r");
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
 * \brief Get 32-bit number input (Dec or Hex).
 * Before first character input format can be changed once by
 * 'x' to hex.
 * \param nbChar Number of character to wait.
 * \param pNum   Pointer to uint32_t for input result.
 * \return 0 if valid data inputed.
 */
static uint8_t GetU32Input(uint8_t nbChar, uint32_t *pU32)
{
    uint8_t key, isHex = 0;
    uint32_t  i;
    uint32_t  result = 0;
    for (i = 0; i < nbChar;) {
        key = UART_GetChar();
        /* User cancel input */
        if (key == 27) {
            printf(" Canceled\n\r");
            return key;
        }
        /* Short input */
        if (key == '\r') break;
        if (key == 'x' && i == 0) {
            if (isHex == 0) {
                isHex = 1;
                UART_PutChar(key);
            }
            continue;
        }
        if (key > '9' || key < '0') {
            if (isHex) {
                if (key < 'a' || key > 'z') {
                    continue;
                }
            }
            else {
                continue;
            }
        }
        UART_PutChar(key);
        if (isHex) {
            if (key >= 'a')
                result = result * 16 + (key - 'a' + 10);
            else
                result = result * 16 + (key - '0');
        }
        else {
            result = result * 10 + (key - '0');
        }
        i ++;
    }
    if (pU32) *pU32 = result;
    return 0;
}

/**
 *  Dump a register
 */
static uint8_t DumpReg(uint8_t slot)
{
    uint32_t addr;
    uint32_t data;

    DumpSeperator();
    printf("Address to read: ");
    if (0 == GetU32Input(5, &addr)) {
        printf("\n\r");
        if (0 == SDIO_ReadDirect(&sdDrv, curFunc, addr, (uint8_t*)&data, 1)) {
            printf("- SDIO.%u.0x%x: 0x%x\n\r",
                curFunc, (unsigned int)addr, (uint8_t)data);
        }
    }
    return 0;
}

/**
 *  Write a register
 */
static uint8_t WriteReg(uint8_t slot)
{
    uint32_t addr;
    uint32_t data;

    DumpSeperator();
    printf("Address to write: ");
    if (0 == GetU32Input(5, &addr)) {
        printf("\n\rData to write: ");
        if (0 == GetU32Input(3, &data)) {
            printf("\n\r- SDIO.%u.0x%x = 0x%x\n\r",
                curFunc, (unsigned int)addr, (unsigned int)data);
            SDIO_WriteDirect(&sdDrv, curFunc, addr, data);
        }
    }

    return 0;
}

/**
 *  Initialize the inserted card
 */
static uint8_t InitCard(uint8_t slot)
{
    uint8_t rc;

    /* Initialize the SD card driver */
    rc = SD_Init(&sdDrv, &mciDrv);
    if (rc) {

        printf("-E- SD/MMC initialization failed: %x\n\r", rc);
    }
    else {

        printf("-I- SD/MMC card initialization successful\n\r");
    }

    if (SD_GetCardType(&sdDrv) & CARD_TYPE_bmSDIO) {
        SDIO_DisplayCardInformation(&sdDrv);
        return 1;
    }
    else {
        printf("-I- Not a SDIO card, type %x\n\r", SD_GetCardType(&sdDrv));
    }
    return 0;
}

/**
 *  Perform test on SDIO.CIA
 */
static uint8_t TestCIA(uint8_t slot)
{
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
    uint8_t sdState = 0; /* 0: no card, 1: connected, 2: error */

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- Basic HSMCI SDIO Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure SDcard pins */
    PIO_Configure(pinsSd, PIO_LISTSIZE(pinsSd));

    /* Wait for SD card connection (if supported) */
    CardDetectConfigure();
    while(!CardIsConnected(0));

    /* Initialize the MCI driver */
    MCI_Init( &mciDrv, HSMCI, ID_HSMCI, BOARD_MCK ) ;
    NVIC_EnableIRQ( HSMCI_IRQn ) ;

    while ( 1 )
    {
        /* Check card connection */
        if ( CardIsConnected( 0 ) )
        {
            if ( sdState == 0 )
            {
                if ( InitCard( 0 ) )
                {
                    sdState = 1;
                    TestCIA(0);
                }
                else
                {
                    sdState = 2;
                }
            }

            if ( UART_IsRxReady() )
            {
                uint8_t key = UART_GetChar();

                switch ( key )
                {
                    case 'r': DumpReg(0);   break;
                    case 'w': WriteReg(0);  break;
                    case 't':
                        TestCIA(0); printf("\n\r");
                        break;
                    case 'f':
                        curFunc = ((curFunc + 1) % 7);
                        printf("** SDIO Function -> %u\n\r", curFunc);
                        break;
                }
            }
        }
        else
            if (sdState)
            {
            printf("\n\r** Card disconnected\n\r");
            sdState = 0;
        }
    }
}

