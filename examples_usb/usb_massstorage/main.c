/*----------------------------------------------------------------------------
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
 * \page usb_massstorage USB Device Mass Storage Example
 *
 * \section Purpose
 *
 * The USB Massstorage Example will help you to get familiar with the
 * USB Device Port(UDP) on SAM microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Mass
 * Storage class (MSD).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB MSD driver.
 * - USB MSD driver development based on the USB Framework.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 *
 * See
 * - memories: Storage Media interface for MSD
 * - usb: USB Framework, USB MSD driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usbd_msd
 *       - \ref usbd_msd_drv
 *
 * \section Requirements
 *
 * This package can be used with all Atmel evaluation kits that have USB interface
 *
 * \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, the EK appears as a USB Disk for the host. Then the host can
 * format/read/write on the disk.
 *
 * If there is SDRAM on the EK, the disk can be up to 10M so that read/write
 * speed can be tested.
 *
 * If there is no SDRAM but only internal flash, the disk is about 30K and
 * only small file can be tested.
 *
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
 *     -- USB Device Mass Storage Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB %device attachment and Disk installation.
 *  . Then new "USB Mass Storage Device" and
 *    "ATMEL Mass Storage MSD USB Device" and "Generic volume" appear in
 *    hardware %device list.
 * -# You can find the new disk on host, and to create/write file to it.
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_massstorage.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "USBDDriver.h"
#include "MSDDriver.h"
#include "MSDLun.h"
#include "device_descriptor.h"

#include "memories.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            2

/** Media index for different disks */
#define DRV_RAMDISK         0    /** RAM disk */
#define DRV_IFLASH          0    /** Internal flash */
#define DRV_SDMMC           0    /** SD card */
#define DRV_NAND            1    /** Nand flash */

/** Delay for pushbutton debouncing (ms) */
#define DEBOUNCE_TIME       10

/** PIT period value (seconds) */
#define PIT_PERIOD          1000

/** Delay for display view update (*250ms) */
#define UPDATE_DELAY        4

/** Delay for waiting DBGU input (*250ms) */
#define INPUT_DELAY         15

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (6K, more the better). */
#define MSD_BUFFER_SIZE     (12*BLOCK_SIZE)

/* Ramdisk size: 20K (WinXP can not format the disk if lower than 20K) */
/* #define RAMDISK_SIZE    (20*1024) */

/** Size of the reserved Nand Flash (4M) */
#define NF_RESERVE_SIZE     (4*1024*1024)

/** Size of the managed Nand Flash (128M) */
#define NF_MANAGED_SIZE     (128*1024*1024)

/*----------------------------------------------------------------------------
 *        Global variables
 *----------------------------------------------------------------------------*/

/** MSD Driver Descriptors List */
//extern const USBDDriverDescriptors msdDriverDescriptors;

/** Available medias. */
Media medias[MAX_LUNS];

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Device LUNs. */
MSDLun luns[MAX_LUNS];

/** LUN read/write buffer. */
uint8_t msdBuffer[MSD_BUFFER_SIZE];

/** Total data read/write by MSD */
unsigned int msdReadTotal = 0;
unsigned int msdWriteTotal = 0;
unsigned short msdFullCnt = 0;
unsigned short msdNullCnt = 0;

/** Update delay counter, tick is 250ms */
uint32_t updateDelay = UPDATE_DELAY;

/** Flag to update Display View */
uint8_t updateView = 0;

/** Pins used to access to nandflash. */
static const Pin pPinsNf[] = {PINS_NANDFLASH};
/** Nandflash device structure. */
static struct TranslatedNandFlash translatedNf;
/** Address for transferring command bytes to the nandflash. */
static unsigned int cmdBytesAddr = BOARD_NF_COMMAND_ADDR;
/** Address for transferring address bytes to the nandflash. */
static unsigned int addrBytesAddr = BOARD_NF_ADDRESS_ADDR;
/** Address for transferring data bytes to the nandflash. */
static unsigned int dataBytesAddr = BOARD_NF_DATA_ADDR;
/** Nandflash chip enable pin. */
static const Pin nfCePin = BOARD_NF_CE_PIN;
/** Nandflash ready/busy pin. */
static const Pin nfRbPin = BOARD_NF_RB_PIN;

/*----------------------------------------------------------------------------
 *        VBus monitoring (optional)
 *----------------------------------------------------------------------------*/

/** VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;

/**
 * \brief Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus(const Pin *pPin)
{
    /* Check current level on VBus */
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("VBUS conn\n\r");
        USBD_Connect();
    }
    else {

        TRACE_INFO("VBUS discon\n\r");
        USBD_Disconnect();
    }
}

/**
 * \brief Configures the VBus Pin
 *
 * To trigger an interrupt when the level on that pin changes.
 */
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\n\r");

    /* Configure PIO */
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    /* Check current level on VBus */
    if (PIO_Get(&pinVbus)) {

        /* if VBUS present, force the connect */
        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else {
        USBD_Disconnect();
    }
}

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked after the USB driver has been initialized. By default, configures
 * the UDP/UDPHS interrupt.
 */
void USBDCallbacks_Initialized(void)
{
    NVIC_EnableIRQ(UDP_IRQn);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    MSDDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Resets the mass
 * storage driver.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    MSDDriver_ConfigurationChangeHandler(cfgnum);
}

/*----------------------------------------------------------------------------
 *        Callbacks
 *----------------------------------------------------------------------------*/

/**
 * Invoked when the MSD finish a READ/WRITE.
 * \param flowDirection 1 - device to host (READ10)
 *                      0 - host to device (WRITE10)
 * \param dataLength Length of data transferred in bytes.
 * \param fifoNullCount Times that FIFO is NULL to wait
 * \param fifoFullCount Times that FIFO is filled to wait
 */
static void MSDCallbacks_Data( uint8_t flowDirection, uint32_t dataLength,
                               uint32_t fifoNullCount, uint32_t fifoFullCount )
{
    if ( flowDirection )
    {
        msdReadTotal += dataLength;
    }
    else
    {
        msdWriteTotal += dataLength;
    }

    msdFullCnt += fifoFullCount;
    msdNullCnt += fifoNullCount;
}

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Interrupt handler for TC0.
 */
void TC0_IrqHandler(void)
{
    volatile uint32_t dummy;
    /* Clear status bit to acknowledge interrupt */
    dummy = TC0->TC_CHANNEL[0].TC_SR;

    if (-- updateDelay == 0) {

        updateDelay = UPDATE_DELAY;
        updateView = 1;
    }
}

/**
 * \brief Configure 48MHz Clock for USB
 */
static void _ConfigureUsbClock(void)
{
    /* Enable PLLB for USB */
    PMC->CKGR_PLLBR = CKGR_PLLBR_DIVB(1)
                    | CKGR_PLLBR_MULB(7)
                    | CKGR_PLLBR_PLLBCOUNT_Msk;
    while((PMC->PMC_SR & PMC_SR_LOCKB) == 0);
    /* USB Clock uses PLLB */
    PMC->PMC_USB = PMC_USB_USBDIV(1)    /* /2   */
                 | PMC_USB_USBS;        /* PLLB */
}


/**
 * \brief Configure Timer Counter 0
 *
 *  Configure TC0 to generate an interrupt every 250ms.
 */
static void _ConfigureTc0(void)
{
    uint32_t div;
    uint32_t tcclks;

    /* Enable peripheral clock */
    PMC->PMC_PCER0 = 1 << ID_TC0;

    /* Configure TC for a 4Hz frequency and trigger on RC compare */
    TC_FindMckDivisor(4, BOARD_MCK, &div, &tcclks, BOARD_MCK);
    TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
    TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / div) / 4;

    /* Configure and enable interrupt on RC compare */
    NVIC_EnableIRQ((IRQn_Type)ID_TC0);
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;

    TC_Start(TC0, 0);
}

/**
 * Initialize Nand Flash for LUN
 */
static void NandFlashInitialize(void)
{
    unsigned char nfRc;
    unsigned short nfBaseBlock = 0;
    struct RawNandFlash *pRaw = (struct RawNandFlash*)&translatedNf;
    struct NandFlashModel *pModel = (struct NandFlashModel*)&translatedNf;
    unsigned int nfMamagedSize;

    /* Configure SMC for NandFlash (8-bit) */
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
    if (nfRc) {
        printf("Nand not found\n\r");
        return;
    }
    else {
        printf("NF\tNb Blocks %d\n\r",
               NandFlashModel_GetDeviceSizeInBlocks(pModel));
        printf("\tBlock Size %dK\n\r",
               NandFlashModel_GetBlockSizeInBytes(pModel)/1024);
        printf("\tPage Size %d\n\r",
               NandFlashModel_GetPageDataSize(pModel));
        nfBaseBlock =
            NF_RESERVE_SIZE / NandFlashModel_GetBlockSizeInBytes(pModel);
    }
    printf("NF disk will use area from %dM(B%d)\n\r",
           NF_RESERVE_SIZE/1024/1024, nfBaseBlock);
    printf("!! Erase the NF Disk? (y/n):");
    updateDelay = INPUT_DELAY;
    updateView = 0;
    while(1) {
        if(UART_IsRxReady()) {
            char key = UART_GetChar();
            UART_PutChar(key);
            if (key == 'y') {
                if (nfRc == 0) {
                    unsigned int block;
                    printf(" Erase from %d ... ", nfBaseBlock);
                    for (block = nfBaseBlock;
                     block < NandFlashModel_GetDeviceSizeInBlocks(pModel);
                     block ++) {
                        RawNandFlash_EraseBlock(pRaw, block);
                    }
                    printf("OK");
                }
            }
            printf("\n\r");
            break;
        }
        if (updateView) {
            printf("No\n\r");
            break;
        }
    }
    nfMamagedSize = ((NandFlashModel_GetDeviceSizeInMBytes(pModel) - NF_RESERVE_SIZE/1024/1024) > NF_MANAGED_SIZE/1024/1024) ? \
                        NF_MANAGED_SIZE/1024/1024 : (NandFlashModel_GetDeviceSizeInMBytes(pModel) - NF_RESERVE_SIZE/1024/1024);
    if (TranslatedNandFlash_Initialize(&translatedNf,
                                       0,
                                       cmdBytesAddr,
                                       addrBytesAddr,
                                       dataBytesAddr,
                                       nfCePin,
                                       nfRbPin,
                                       nfBaseBlock, nfMamagedSize * 1024 * 1024/NandFlashModel_GetBlockSizeInBytes(pModel))) {
        printf("Nand init error\n\r");
        return;
    }

    /* Media initialize */
    MEDNandFlash_Initialize(&medias[DRV_NAND], &translatedNf);

    /* Initialize LUN */
    LUN_Init(&(luns[DRV_NAND]), &(medias[DRV_NAND]),
             msdBuffer, MSD_BUFFER_SIZE,
             0, 0, 0, 0,
             MSDCallbacks_Data);
}

/**
 * \brief Initialize all medias and LUNs.
 */
static void MemoryInitialization(void)
{
    uint32_t i;
    for (i = 0; i < MAX_LUNS; i ++)
        LUN_Init(&luns[i], 0, 0, 0, 0, 0, 0, 0, 0);

    // TODO: Add LUN Init here

    /* Nand Flash Init */
    NandFlashInitialize();
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief usb_massstorage Application entry point.
 *
 * Configures UART,
 * Configures TC0, USB MSD Driver and run it.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    printf("-- USB Device Mass Storage Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Start TC for timing & status update */
    _ConfigureTc0();

    /* Initialize memories and LUN */
    MemoryInitialization();

    /* BOT driver initialization */
    MSDDriver_Initialize(&msdDriverDescriptors,
                         luns, MAX_LUNS);

    /* connect if needed */
    VBus_Configure();

    /* Infinite loop */
    updateDelay = UPDATE_DELAY;
    updateView = 0;
    while (1) {

        /* Mass storage state machine */
        if (USBD_GetState() < USBD_STATE_CONFIGURED){}
        else MSDDriver_StateMachine();

        /* Update status view */
        if (updateView) {

            updateView = 0;

            if (msdWriteTotal < 50 * 1000)
                MED_Flush(&medias[DRV_NAND]);

            printf("Read %5dK, Write %5dK, IO %5dK; Null %4d, Full %4d\r",
                msdReadTotal/(UPDATE_DELAY*250),
                msdWriteTotal/(UPDATE_DELAY*250),
                (msdReadTotal+msdWriteTotal)/(UPDATE_DELAY*250),
                msdNullCnt, msdFullCnt);

            msdReadTotal = 0;
            msdWriteTotal = 0;
            msdNullCnt = 0;
            msdFullCnt = 0;
        }
    }
}
