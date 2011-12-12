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
 * \page usb_iad_cdc_msd USB Composite CDC+MSD Example
 *
 * \section Purpose
 *
 * The USB CDCMSD Project will help you to get familiar with the
 * USB Device Port(UDP)interface and also some of the other interfaces in
 * SAM microcontrollers. Also it can help you to be familiar with the USB
 * Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single CDCMSD device (such as CDC + MSD).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB Device Framework.
 * - USB CDCMSD device and functions driver development based on the
 *   USB Device Framework and other re-usable class driver code.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 *
 * \section See
 *
 * - pio: Pin configurations and peripheral configure.
 * - memories: Storage Media interface for MSD
 * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_msd "massstorage"
 *       - \ref usbd_msd_drv
 *    - \ref usbd_cdc "cdc-serial"
 *       - \ref usbd_cdc_serial_drv
 * - projects:
 *    - \ref usb_massstorage
 *    - \ref usb_cdc_serial
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
 *     -- USB CDCMSD Device Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB %device attachment.
 * -# For the windows driver installation and the test functions, please
 *      refer to "USB CDC serial converter" &
 *      "USB Device Mass Storage Project".
 * -# You can use the inf file
 *    libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
 *    to install the CDC serial  port.
 *
 */

/**
 * \file
 *
 * \section Purpose
 *
 * This file contains all the specific code for the
 * usb_iad_cdc_msd project
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "board.h"

#include <memories.h>

#include <include/USBD_Config.h>
#include <CDCMSDDriver.h>
#include <CDCDSerial.h>
#include <MSDFunction.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 *      Definitions
 *---------------------------------------------------------------------------*/

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE \
    (CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)+2)

/** Pins used for USART transfer */
#define PINS_USART      PIN_USART1_TXD, PIN_USART1_RXD, PIN_USART1_EN
/** Register base for USART operatoin */
#define BASE_USART      USART1
/** USART ID */
#define ID_USART        ID_USART1
/** IRQ number for USART */
#define IRQn_USART      USART1_IRQn
/** Handler for USART interrupt service */
#define USART_Handler   USART1_IrqHandler

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

/** Delay for MSD refresh (*4ms) */
#define MSD_REFRESH_DELAY    250

/** Delay for waiting DBGU input (*4ms) */
#define INPUT_DELAY          (2*250)

/*---------------------------------------------------------------------------
 *      External variables
 *---------------------------------------------------------------------------*/

/** Descriptor list for the device to bring up */
extern const USBDDriverDescriptors cdcmsddDriverDescriptors;

/*---------------------------------------------------------------------------
 *      Internal variables
 *---------------------------------------------------------------------------*/

/*- CDC */
/** List of pins that must be configured for use by the application. */
static const Pin pinsUsart[] = {PINS_USART};

/** Double-buffer for storing incoming USART data. */
static uint8_t usartBuffers[2][DATABUFFERSIZE];

/** Current USART buffer index. */
static uint8_t usartCurrentBuffer = 0;

/** Buffer for storing incoming USB data. */
static uint8_t usbSerialBuffer0[DATABUFFERSIZE];

/** Serial port openned */
static uint8_t isSerialPortON = 0;

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
uint32_t msdDelay = MSD_REFRESH_DELAY;
/** Delay TO event */
uint8_t  msdRefresh = 0;

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

/*---------------------------------------------------------------------------
 *         VBus monitoring (optional)
 *---------------------------------------------------------------------------*/

/** VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;

/**
 * Handles interrupts coming from PIO controllers.
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
    CDCMSDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    CDCMSDDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Handles interrupts coming from Timer #0.
 */
void TC0_IrqHandler(void)
{
    Tc *pTc0 = TC0;
    uint8_t size;
    volatile uint32_t status = pTc0->TC_CHANNEL[0].TC_SR;

    if ((status & TC_SR_CPCS) != 0) {

        /* MSD Timer */
        if (-- msdDelay == 0) {
            msdRefresh = 1;
            msdDelay = MSD_REFRESH_DELAY;
        }

        /* Serial Timer */
        if (isSerialPortON) {

            /* Flush PDC buffer */
            size = DATABUFFERSIZE - BASE_USART->US_RCR;
            if (size == 0) {

                pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
                return;
            }
            BASE_USART->US_RCR = 0;

            /* Send current buffer through the USB */
            CDCDSerial_Write(usartBuffers[usartCurrentBuffer], size, 0, 0);

            /* Restart read on buffer */
            USART_ReadBuffer(BASE_USART,
                             usartBuffers[usartCurrentBuffer],
                             DATABUFFERSIZE);
            usartCurrentBuffer = 1 - usartCurrentBuffer;
            pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
        }
    }
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 *----------------------------------------------------------------------------*/
static void UsbDataReceived(uint32_t unused,
                            uint8_t status,
                            uint32_t received,
                            uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Send data through USART */
        while (!USART_WriteBuffer(BASE_USART, usbSerialBuffer0, received));
        BASE_USART->US_IER = US_IER_TXBUFE;

        /* Check if bytes have been discarded */
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            TRACE_WARNING(
                      "UsbDataReceived: %u bytes discarded\n\r", (unsigned int)remaining);
        }
    }
    else {

        TRACE_WARNING( "UsbDataReceived: Transfer error\n\r");
    }
}

/*----------------------------------------------------------------------------
 * Handles interrupts coming from USART
 *----------------------------------------------------------------------------*/
void USART_Handler(void)
{
    uint32_t status;
    unsigned short serialState;

    status  = BASE_USART->US_CSR;
    status &= BASE_USART->US_IMR;

    /* If USB device is not configured, do nothing */
    if (!isSerialPortON) {

        BASE_USART->US_IDR = 0xFFFFFFFF;
        return;
    }

    /* Buffer has been read successfully */
    if ((status & US_CSR_ENDRX) != 0) {

        /* Disable timer */
        TC_Stop(TC0, 0);

        /* Send buffer through the USB */
        while (CDCDSerial_Write(usartBuffers[usartCurrentBuffer],
                                DATABUFFERSIZE, 0, 0) != USBD_STATUS_SUCCESS);

        /* Restart read on buffer */
        USART_ReadBuffer(BASE_USART,
                         usartBuffers[usartCurrentBuffer],
                         DATABUFFERSIZE);
        usartCurrentBuffer = 1 - usartCurrentBuffer;

        /* Restart timer */
        TC_Start(TC0, 0);
    }

    /* Buffer has been sent */
    if ((status & US_IER_TXBUFE) != 0) {

        /* Restart USB read */
        CDCDSerial_Read(usbSerialBuffer0,
                        DATABUFFERSIZE,
                        (TransferCallback) UsbDataReceived,
                        0);
        BASE_USART->US_IDR = US_IER_TXBUFE;
    }

    /* Errors */
    serialState = CDCDSerial_GetSerialState();

    /* Overrun */
    if ((status & US_CSR_OVRE) != 0) {

        TRACE_WARNING( "USART1_IrqHandler: Overrun\n\r");
        serialState |= CDCSerialState_OVERRUN;
    }

    /* Framing error */
    if ((status & US_CSR_FRAME) != 0) {

        TRACE_WARNING( "USART1_IrqHandler: Framing error\n\r");
        serialState |= CDCSerialState_FRAMING;
    }

    CDCDSerial_SetSerialState(serialState);
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

/**
 * Invoked when the CDC ControlLineState is changed
 * \param DTR   New DTR value.
 * \param RTS   New RTS value.
 */
void CDCDSerial_ControlLineStateChanged(uint8_t DTR,
                                        uint8_t RTS)
{
    isSerialPortON = DTR;
}

/**
 * Invoked when the CDC LineCoding is requested to changed
 * \param port          Port number.
 * \param pLineCoding   Pointer to new LineCoding settings.
 */
uint8_t CDCDSerial_LineCodingIsToChange(CDCLineCoding* pLineCoding)
{
    return USBD_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------
 *         Callbacks re-implementation
 *----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Configure USART to work @ 115200
 */
static void ConfigureUsart(void)
{
    PIO_Configure(pinsUsart, PIO_LISTSIZE(pinsUsart));
    PMC_EnablePeripheral(ID_USART);
    BASE_USART->US_IDR = 0xFFFFFFFF;
    USART_Configure(BASE_USART,
                    USART_MODE_ASYNCHRONOUS,
                    115200,
                    BOARD_MCK);

    USART_SetTransmitterEnabled(BASE_USART, 1);
    USART_SetReceiverEnabled(BASE_USART, 1);
    NVIC_EnableIRQ(IRQn_USART);
}

/**
 * Configure TC0 to generate an interrupt every 4ms
 */
static void ConfigureTc0(void)
{
    uint32_t div, tcclks;

    /* Enable TC0 peripheral */
    PMC_EnablePeripheral(ID_TC0);
    /* Configure TC0 for 250Hz frequency and trigger on RC compare */
    TC_FindMckDivisor(250, BOARD_MCK, &div, &tcclks, BOARD_MCK);
    TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
    TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / div) / 250;
    /* Configure and enable interrupt on RC compare */
    NVIC_EnableIRQ(TC0_IRQn);
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    /* Start TC as event timer */
    TC_Start(TC0, 0);
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
    msdDelay = INPUT_DELAY;
    msdRefresh = 0;
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
        if (msdRefresh) {
            msdRefresh = 0;
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
 * Initializes drivers and start the USB CDCMSD device.
 */
int main(void)
{
    uint8_t usbConnected = 0, serialON = 0;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB CDCMSD Device Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* If there is on board power, switch it off */

    /* Enable UPLL for USB */
    ConfigureUsbClock();

    /* Configure timer 0 */
    ConfigureTc0();

    /* ----- CDC Function Initialize */
    /* Configure USART */
    ConfigureUsart();

    /* ----- MSD Function Initialize */
    /* Configure memories */
    MemoriesInitialize();

    /* USB CDCMSD driver initialization */
    CDCMSDDriver_Initialize(&cdcmsddDriverDescriptors, luns, MAX_LUNS);

    /* connect if needed */
    VBus_Configure();

    /* Driver loop */
    while (1) {

        /* Device is not configured */
        if (USBD_GetState() < USBD_STATE_CONFIGURED) {

            if (usbConnected) {
                printf("-I- USB Disconnect/Suspend\n\r");
                usbConnected = 0;

                /* Serial port closed */
                isSerialPortON = 0;
            }
        }
        else {

            if (usbConnected == 0) {
                printf("-I- USB Connect\n\r");
                usbConnected = 1;
            }

            if (!serialON && isSerialPortON) {
                printf("-I- SerialPort ON\n\r");
                /* Start receiving data on the USART */
                usartCurrentBuffer = 0;
                USART_ReadBuffer(BASE_USART,
                                 usartBuffers[0], DATABUFFERSIZE);
                USART_ReadBuffer(BASE_USART,
                                 usartBuffers[1], DATABUFFERSIZE);
                BASE_USART->US_IER = US_CSR_ENDRX
                                     | US_CSR_FRAME
                                     | US_CSR_OVRE;

                /* Start receiving data on the USB */
                CDCDSerial_Read(usbSerialBuffer0,
                                DATABUFFERSIZE,
                                (TransferCallback) UsbDataReceived,
                                0);
                serialON = 1;
            }
            else if (serialON && !isSerialPortON) {
                printf("-I- SeriaoPort OFF\n\r");
                serialON = 0;
            }

            MSDFunction_StateMachine();

            if (msdRefresh) {
                msdRefresh = 0;

                if (msdWriteTotal < 50 * 1000)
                    MED_Flush(&medias[DRV_NAND]);
                msdWriteTotal = 0;
            }
        }
    }
}

