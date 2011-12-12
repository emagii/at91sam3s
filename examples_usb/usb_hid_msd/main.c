/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
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
 * \page usb_hid_msd USB Composite HID+MSD Example
 *
 * \section Purpose
 *
 * The USB COMPOSITE Project will help you to get familiar with the
 * USB Device Port(UDP)interface and also some of the other interfaces in
 * SAM microcontrollers. Also it can help you to be familiar with the USB
 * Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single composite device (such as CDC + MSD).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB Device Framework.
 * - USB COMPOSITE device and functions driver development based on the
 *   USB Device Framework and other re-usable class driver code.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 *
 * \subsection See
 * - pio: Pin configurations and peripheral configure.
 * - memories: Storage Media interface for MSD
 * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_hid "hid" \\ \ref usbd_hid_key "hid-keyboard"
 *       - \ref usbd_hid_kbd_drv
 *    - \ref usbd_msd "massstorage"
 *       - \ref usbd_msd_drv
 * - projects:
 *    - \ref usb_hid_keyboard
 *    - \ref usb_massstorage
 *
 * \section Requirements
 *
 * This package can be used with some of Atmel evaluation kits that have UDP
 * interface, depending on the functions included.
 *
 *  \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio. The
 * example composite devices are tested under windows XP (SP3). For CDC
 * serial port, additional windows driver file (CompositeCDCSerial.inf) can
 * be found at libraries\\usb\\device\\composite\\drv.
 *
 * The following is alternate update to fix the composite device support
 * on windows XP:
 *
 * \subsection install_win_sp3 Install Windows Service Pack 3 (SP3)
 *
 * All the fixes for USB generic driver are included in window XP service pack
 * 3. It can be found at
 * http://technet.microsoft.com/zh-cn/windows/bb794714(en-us).aspx .
 *
 * \subsection install_win_hotfix Install Windows Hot Fixes
 *
 * Two hot fixes are necessary for window to recognize the composite device
 * correctly:
 *
 * -# http://support.microsoft.com/kb/814560
 * -# http://support.microsoft.com/kb/918365
 *
 * \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, host will notice the attachment of a USB %device. No %device
 * driver offered for the %device now.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6421.pdf">
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
 *     -- USB HIDMSD Device Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB %device attachment.
 * -# For the windows driver installation and the test functions, please
 *      refer to "USB HID Keyboard Project" &
 *      "USB Device Mass Storage Project".
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_hid_msd
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - VBus_Configure
 *       - PIO configurations in start of main
 *    - Interrupt handlers
 *       - ISR_Vbus
 *    - Callback functions
 *       - USBDCallbacks_RequestReceived
 *    - The main function, which implements the program behavior
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <memories.h>

#include <include/USBD_Config.h>
#include <include/USBD_LEDs.h>

#include <HIDMSDDriver.h>
#include <HIDDKeyboard.h>
#include <MSDFunction.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 *      Definitions
 *---------------------------------------------------------------------------*/

/** Master clock frequency in Hz */
#define MCK                         BOARD_MCK

/** Number of keys used in the example. */
#define NUM_KEYS                    2

/** Number of non-modifiers keys. */
#define NUM_NORMAL_KEYS             1

/** Number of modifier keys. */
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/** Num lock LED index. */
#define LED_NUMLOCK                 USBD_LEDOTHER


/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            1
/** Media index for different disks */
#define DRV_NAND            0    /**< Nand flash */

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (6K, more the better). */
#define MSD_BUFFER_SIZE     (12*BLOCK_SIZE)

/** Size of the reserved Nand Flash (4M) */
#define NF_RESERVE_SIZE     (4*1024*1024)

/** Size of the managed Nand Flash (128M) */
#define NF_MANAGED_SIZE     (128*1024*1024)

/** Delay loop for MSD refresh */
#define MSD_REFRESH_LOOP    500000

/*----------------------------------------------------------------------------
 *        External variables
 *----------------------------------------------------------------------------*/

/** Descriptor list for device enumeration */
extern const USBDDriverDescriptors hidmsddDriverDescriptors;

/*----------------------------------------------------------------------------
 *        Global variables
 *----------------------------------------------------------------------------*/

/*- HID */
/** List of pinsPushButtons to configure for the applicatino. */
static Pin pinsPushButtons[] = {PINS_PUSHBUTTONS};

/** Array of key codes produced by each button. */
static uint8_t keyCodes[NUM_KEYS] = {
    HIDKeypad_A,
    HIDKeypad_NUMLOCK,
    //HIDKeypad_9,
    //HIDKeypad_RIGHTSHIFT
};

/** Current status (pressed or not) for each key. */
static uint8_t keyStatus[NUM_KEYS];

/*- MSD */
/** Available medias. */
Media medias[MAX_LUNS];

/** Device LUNs. */
MSDLun luns[MAX_LUNS];

/** LUN read/write buffer. */
uint8_t msdBuffer[MSD_BUFFER_SIZE];

/** Total data write to disk */
uint32_t msdWriteTotal = 0;
/** Delay for data write refresh */
uint32_t msdRefreshDelay = MSD_REFRESH_LOOP;

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
 *         Remote wake-up support (optional)
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *         VBus monitoring (optional)
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
 * Configures the VBus pin to trigger an interrupt when the level on that pin
 * changes.
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
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    HIDMSDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    HIDMSDDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Callbacks
 *----------------------------------------------------------------------------*/

/**
 * Invoked when the MSD finish a READ/WRITE.
 * \param flowDirection 1 - device to host (READ10)
 *                      0 - host to device (WRITE10)
 * \param dataLength Length of data transferred in bytes.
 * \param fifoNullCount Times that FIFO is NULL to wait
 * \param fifoFullCount Times that FIFO is filled to wait
 */
static void MSDCallbacks_Data(uint8_t flowDirection,
                       uint32_t  dataLength,
                       uint32_t  fifoNullCount,
                       uint32_t  fifoFullCount)
{
    if (!flowDirection) {

        msdWriteTotal += dataLength;
    }
}

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Invoked when the status of the keyboard LEDs changes. Turns the num. lock
 * LED on or off.
 * \param numLockStatus Indicates the current status of the num. lock key.
 * \param capsLockStatus Indicates the current status of the caps lock key.
 * \param scrollLockStatus Indicates the current status of the scroll lock key
 */
void HIDDKeyboardCallbacks_LedsChanged(
    uint8_t numLockStatus,
    uint8_t capsLockStatus,
    uint8_t scrollLockStatus)
{
    /* Num. lock */
    if (numLockStatus) {

        LED_Set(LED_NUMLOCK);
    }
    else {

        LED_Clear(LED_NUMLOCK);
    }
}

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Monitor keyboard buttons & Update key status in HID driver
 */
static void HIDDKeyboardProcessKeys(void)
{
    uint32_t i;
    uint8_t pressedKeys[NUM_KEYS];
    uint8_t pressedKeysSize = 0;
    uint8_t releasedKeys[NUM_KEYS];
    uint8_t releasedKeysSize = 0;

    /* Monitor buttons */
    for (i=0; i < PIO_LISTSIZE(pinsPushButtons); i++) {

        /* Check if button state has changed */
        uint8_t isButtonPressed = PIO_Get(&(pinsPushButtons[i]));
        if (isButtonPressed != keyStatus[i]) {

            /* Update button state */
            if (!isButtonPressed) {

                /* Key has been pressed */
                TRACE_INFO("-I- Key %u has been pressed\n\r", (unsigned int)i);
                keyStatus[i] = 0;
                pressedKeys[pressedKeysSize] = keyCodes[i];
                pressedKeysSize++;
                HIDDKeyboard_RemoteWakeUp();
            }
            else {

                /* Key has been released */
                TRACE_INFO("-I- Key %u has been released\n\r", (unsigned int)i);
                keyStatus[i] = 1;
                releasedKeys[releasedKeysSize] = keyCodes[i];
                releasedKeysSize++;
            }
        }
    }

    /* Update key status in the HID driver if necessary */
    if ((pressedKeysSize != 0) || (releasedKeysSize != 0)) {

        uint8_t status;

        do {

            status = HIDDKeyboard_ChangeKeys(pressedKeys,
                                             pressedKeysSize,
                                             releasedKeys,
                                             releasedKeysSize);
        }
        while (status != USBD_STATUS_SUCCESS);
    }
}

/**
 * Initialize Nand Flash for LUN
 */
static void NandFlashInitialize(void)
{
    uint8_t nfRc;
    uint16_t nfBaseBlock = 0;
    struct RawNandFlash *pRaw = (struct RawNandFlash*)&translatedNf;
    struct NandFlashModel *pModel = (struct NandFlashModel*)&translatedNf;
    uint32_t nfMamagedSize;
    uint32_t inputDelay = 1000000;

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
    while(1) {
        if(UART_IsRxReady()) {
            char key = UART_GetChar();
            UART_PutChar(key);
            if (key == 'y') {
                if (nfRc == 0) {
                    uint32_t block;
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
        if (inputDelay -- == 0) {
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
 * Initialize MSD Media & LUNs
 */
static void MemoriesInitialize(void)
{
    uint32_t i ;
	
    /* Reset all LUNs */
    for (i = 0; i < MAX_LUNS; i ++)
        LUN_Init(&luns[i], 0, 0, 0, 0, 0, 0, 0, 0);

    /* TODO: Add LUN Init here */

    /* Nand Flash Init */
    NandFlashInitialize();
}

/**
 * \brief Configure 48MHz Clock for USB
 */
static void ConfigureUsbClock(void)
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

/*---------------------------------------------------------------------------
 *         Exported function
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 *          Main
 *---------------------------------------------------------------------------*/

/**
 * Initializes drivers and start the USB composite device.
 */
int main(void)
{
    uint8_t usbConnected = 0;

    /* Disable watchdog */
    WDT_Disable(WDT);

    printf("-- USB HIDMSD Device Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    ConfigureUsbClock();

    /* ----- HID Function Initialize */
    /* Initialize key statuses and configure push buttons */
    PIO_Configure(pinsPushButtons, PIO_LISTSIZE(pinsPushButtons));
    memset(keyStatus, 1, NUM_KEYS);

    /* Configure LEDs */
    LED_Configure(LED_NUMLOCK);

    MemoriesInitialize();

    /* USB COMPOSITE driver initialization */
    HIDMSDDriver_Initialize(&hidmsddDriverDescriptors, luns, MAX_LUNS);

    /* connect if needed */
    VBus_Configure();

    /* Driver loop */
    while (1) {

        /* Device is not configured */
        if (USBD_GetState() < USBD_STATE_CONFIGURED) {

            if (usbConnected) {
                printf("-I- USB Disconnect/Suspend\n\r");
                usbConnected = 0;
            }
        }
        else {

            if (usbConnected == 0) {
                printf("-I- USB Connect\n\r");
                usbConnected = 1;
            }
            HIDDKeyboardProcessKeys();
            MSDFunction_StateMachine();

            if (msdRefreshDelay -- > 0) {
                msdRefreshDelay = MSD_REFRESH_LOOP;
                if (msdWriteTotal < 5 * 1024) {
                    MED_Flush(&medias[DRV_NAND]);
                }
                msdWriteTotal = 0;
            }
        }
    }
}

