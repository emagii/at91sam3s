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
 * \page usb_iad_cdc_cdc USB Composite DUAL CDC Example
 *
 * \section Purpose
 *
 * The USB DUALCDC Project will help you to get familiar with the
 * USB Device Port(UDP)interface and also some of the other interfaces in
 * SAM microcontrollers. Also it can help you to be familiar with the USB
 * Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single composite device (such as Dual CDC port).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB Device Framework.
 * - USB DUALCDC device and functions driver development based on the
 *   USB Device Framework and other re-usable class driver code.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 *
 * \subsection See
 * - pio: Pin configurations and peripheral configure.
  * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_cdc "cdc-serial"
 *       - \ref usbd_cdc_serial_drv
 * - projects:
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
 * be found at
 * libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf.
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
 *     -- USB Dual CDC Device Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB %device attachment.
 * -# You can use the inf file
 *    libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
 *    to install the serial  port. Then new
 *    "SAM3S USB to Serial Converter (COMx)" appears in the
 *    hardware %device list.
 * -# You can run hyperterminal to send data to the port. And it can be seen
 *    at the other hyperterminal connected to the USART port of the EK or
 *    another USB serial port.
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_iad_cdc_cdc
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <include/USBD_Config.h>

#include <DUALCDCDDriver.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors dualcdcdDriverDescriptors;

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

/** Buffer for storing incoming USB data for serial port 0. */
static uint8_t usbSerialBuffer0[DATABUFFERSIZE];
/** Buffer for storing incoming USB data for serial port 1. */
static uint8_t usbSerialBuffer1[DATABUFFERSIZE];

/** Usart openned */
static uint8_t isUsartON = 0;
/** Serial port 0 openned */
static uint8_t isSerialPort0ON = 0;
/** Serial port 1 openned */
static uint8_t isSerialPort1ON = 0;

/*----------------------------------------------------------------------------
 *         VBus monitoring (optional)
 *----------------------------------------------------------------------------*/

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
    DUALCDCDDriver_ConfigurationChangeHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    DUALCDCDDriver_RequestHandler(request);
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

        /* Serial Timer */
        if (isUsartON) {

            /* Flush PDC buffer */
            size = DATABUFFERSIZE - BASE_USART->US_RCR;
            if (size == 0) {

                pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
                return;
            }
            BASE_USART->US_RCR = 0;

            /* Send current buffer through the USB */
            if (isSerialPort0ON) {
                CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
                                     usartBuffers[usartCurrentBuffer],
                                     size, 0, 0);
            }
            if (isSerialPort1ON) {
                CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
                                     usartBuffers[usartCurrentBuffer],
                                     size, 0, 0);
            }

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
 * For USB CDC Serial Port 0
 *----------------------------------------------------------------------------*/
static void UsbDataReceived0(uint32_t unused,
                             uint8_t status,
                             uint32_t received,
                             uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Send data through USBSerial 1 */
        while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
                                 usbSerialBuffer0,
                                 received, 0, 0) != USBD_STATUS_SUCCESS);

        /* Send data through USART */
        while (!USART_WriteBuffer(BASE_USART, usbSerialBuffer0, received));
        BASE_USART->US_IER = US_IER_TXBUFE;

        /* Check if bytes have been discarded */
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            TRACE_WARNING(
                      "UsbDataReceived0: %u bytes discarded\n\r", (unsigned int)remaining);
        }
    }
    else {

        TRACE_WARNING( "UsbDataReceived0: Transfer error\n\r");
    }
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 * For USB CDC Serial Port 1
 *----------------------------------------------------------------------------*/
static void UsbDataReceived1(uint32_t unused,
                             uint8_t status,
                             uint32_t received,
                             uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Send data through USBSerial 0 */
        while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
                                 usbSerialBuffer1,
                                 received, 0, 0) != USBD_STATUS_SUCCESS);

        /* Send data through USART */
        while (!USART_WriteBuffer(BASE_USART, usbSerialBuffer1, received));
        BASE_USART->US_IER = US_IER_TXBUFE;

        /* Check if bytes have been discarded */
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            TRACE_WARNING(
                      "UsbDataReceived1: %u bytes discarded\n\r", (unsigned int)remaining);
        }
    }
    else {

        TRACE_WARNING( "UsbDataReceived1: Transfer error\n\r");
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
    if (!isUsartON) {

        BASE_USART->US_IDR = 0xFFFFFFFF;
        return;
    }

    /* Buffer has been read successfully */
    if ((status & US_CSR_ENDRX) != 0) {

        /* Disable timer */
        TC_Stop(TC0, 0);

        /* Send buffer through the USB */
        if (isSerialPort0ON) {
            CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
                                     usartBuffers[usartCurrentBuffer],
                                     DATABUFFERSIZE, 0, 0);
        }
        if (isSerialPort1ON) {
            CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
                                     usartBuffers[usartCurrentBuffer],
                                     DATABUFFERSIZE, 0, 0);
        }
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
        if (isSerialPort0ON) {
            CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(0),
                                usbSerialBuffer0,
                                DATABUFFERSIZE,
                                (TransferCallback) UsbDataReceived0,
                                0);
        }
        if (isSerialPort1ON) {
            CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(1),
                                usbSerialBuffer1,
                                DATABUFFERSIZE,
                                (TransferCallback) UsbDataReceived1,
                                0);
        }
        BASE_USART->US_IDR = US_IER_TXBUFE;
    }

    /* Errors */
    serialState = CDCDSerialPort_GetSerialState(
                        DUALCDCDDriver_GetSerialPort(0));

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

    CDCDSerialPort_SetSerialState(
            DUALCDCDDriver_GetSerialPort(0), serialState);
}

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
 * Initializes drivers and start the USB Dual CDC device.
 */
int main(void)
{
    uint8_t usbConnected = 0;
    uint8_t serial0ON = 0, serial1ON = 0, usartON = 0;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB Dual CDC Device Project %s --\n\r", SOFTPACK_VERSION);
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


    /* USB DualCDC driver initialization */
    DUALCDCDDriver_Initialize(&dualcdcdDriverDescriptors);

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
                isSerialPort0ON = 0;
                isSerialPort1ON = 0;
                isUsartON = 0;
            }
        }
        else {

            isSerialPort0ON = CDCDSerialPort_GetControlLineState(
                                DUALCDCDDriver_GetSerialPort(0))
                             & CDCControlLineState_DTR;
            isSerialPort1ON = CDCDSerialPort_GetControlLineState(
                                DUALCDCDDriver_GetSerialPort(1))
                             & CDCControlLineState_DTR;

            if (usbConnected == 0) {
                printf("-I- USB Connect\n\r");
                usbConnected = 1;
            }

            isUsartON = isSerialPort0ON || isSerialPort1ON;
            if (!usartON && isUsartON) {
                usartON = 1;
                printf("-I- USART ON\n\r");
                /* Start receiving data on the USART */
                usartCurrentBuffer = 0;
                USART_ReadBuffer(BASE_USART,
                                 usartBuffers[0], DATABUFFERSIZE);
                USART_ReadBuffer(BASE_USART,
                                 usartBuffers[1], DATABUFFERSIZE);
                BASE_USART->US_IER = US_CSR_ENDRX
                                     | US_CSR_FRAME
                                     | US_CSR_OVRE;
            }
            else if (usartON && !isUsartON) {
                usartON = 0;
                printf("-I- USART OFF\n\r");
            }

            if (!serial0ON && isSerialPort0ON) {
                serial0ON = 1;
                printf("-I- SerialPort0 ON\n\r");
                /* Start receiving data on the USB */
                CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(0),
                                        usbSerialBuffer0,
                                        DATABUFFERSIZE,
                                        (TransferCallback) UsbDataReceived0,
                                        0);
            }
            else if (serial0ON && !isSerialPort0ON) {
                serial0ON = 0;
                printf("-I- SeriaoPort0 OFF\n\r");
            }

            if (!serial1ON && isSerialPort1ON) {
                serial1ON = 1;
                printf("-I- SerialPort1 ON\n\r");
                /* Start receiving data on the USB */
                CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(1),
                                        usbSerialBuffer1,
                                        DATABUFFERSIZE,
                                        (TransferCallback) UsbDataReceived1,
                                        0);
            }
            else if (serial1ON && !isSerialPort1ON) {
                serial1ON = 0;
                printf("-I- SeriaoPort1 OFF\n\r");
            }
        }
    }
}

