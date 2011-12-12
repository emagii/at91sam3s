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
 * \page usb_cdc_serial USB CDC Serial Converter Example
 *
 * \section Purpose
 *
 * The USB CDC Serial Project will help you to get familiar with the
 * USB Device Port(UDP) and USART interface on SAM microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Communication
 * Device class (CDC).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB CDC driver and USART driver.
 * - USB CDC driver development based on the USB Framework.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 * - The initialize sequence and usage of USART interface with PDC.
 *
 * \section See
 * - usart: USART interface driver
 * - tc: TIMER/COUNTER interface driver
 * - usb: USB Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usbd_cdc
 *       - \ref usbd_cdc_serial_drv
 *       - \ref usbd_cdc_host_drv
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, the EK appears as a Seriao COM port for the host, after driver
 * installation with the offered 6119.inf. Then the host can send or receive
 * data through the port with host software. The data stream from the host is
 * then sent to the EK, and forward to USART port of SAM chips. The USART
 * port of the EK is monitored by the timer and the incoming data will be sent
 * to the host.
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
 *     -- USB Device CDC Serial Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB %device attachment (if it's the first time you connect
 *    an %audio speaker demo board to your host). You can use the inf file
 *    libraries\\usb\\device\\cdc-serial\\drv\\6119.inf to install the serial
 *    port. Then new "SAM3S USB to Serial Converter (COMx)" appears in the
 *    hardware %device list.
 * -# You can run hyperterminal to send data to the port. And it can be seen
 *    at the other hyperterminal connected to the USART port of the EK.
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_cdc_serial example.
 *
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "CDCDSerialDriver.h"

#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (64+2)

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

/** Speed test buffer size */
#define TEST_BUFFER_SIZE    (2*1024)
/** Speed test loop count */
#define TEST_COUNT          (30)

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdcdSerialDriverDescriptors;

/*----------------------------------------------------------------------------
 *      Internal variables
 *----------------------------------------------------------------------------*/

/** List of pins that must be configured for use by the application. */
static const Pin pins[] = {PINS_USART};

/** Double-buffer for storing incoming USART data. */
static uint8_t usartBuffers[2][DATABUFFERSIZE];

/** Current USART buffer index. */
static uint8_t usartCurrentBuffer = 0;

/** Buffer for storing incoming USB data. */
static uint8_t usbBuffer[DATABUFFERSIZE];

/** Serial Port ON/OFF */
static uint8_t isCdcSerialON = 0;

/** CDC Echo back ON/OFF */
static uint8_t isCdcEchoON = 0;

/** DBG Port Activity ON/OFF */
static uint8_t isDbgStrON = 0;

/** DEBUG output: US->USB */
static char hdrS2U[] = {'\n','\r','>'};
/** DEBUG output: USB->US */
static char hdrU2S[] = {'\n','\r','<'};

/** TC tick: 1/250 s */
static uint32_t tcTick = 0;

/** USB Tx flag */
static uint8_t txDoneFlag = 0;
/** Test buffer */
static uint8_t testBuffer[TEST_BUFFER_SIZE];

/*----------------------------------------------------------------------------
 *         VBus monitoring
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
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
    CDCDSerialDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    CDCDSerialDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 * DBGU Inoformation dump
 * Uses PDC, all input data should in RAM.
 * Data buffer should reserve (size + 2) memory space, and last 2 bytes for
 * '\n' and '\r'.
 */
static void _DebugLog(const char *pHdr, uint8_t hdrLen,
                      uint8_t* pData, uint32_t size)
{
    if (isDbgStrON) {

        if (pHdr) {

            USART_WriteBuffer((Usart*)UART0,
                             (void*)pHdr, hdrLen);
        }
        if (pData) {

            pData[size] = '\n'; pData[size+1] = '\r';
            USART_WriteBuffer((Usart*)UART0,
                              (void*)pData, size+2);
        }
    }
}

/**
 * DBGU help dump
 */
static void _DebugHelp(void)
{
    printf("-- ESC to Enable/Disable ECHO on cdc serial --\n\r");
    printf("-- TAB to Enable/Disable DEBUG log output   --\n\r");
}

/**
 * Handles interrupts coming from Timer #0.
 */
void TC0_IrqHandler(void)
{
    Tc *pTc0 = TC0;
    uint8_t size;
    volatile uint32_t status = pTc0->TC_CHANNEL[0].TC_SR;

    if ((status & TC_SR_CPCS) != 0) {

        tcTick ++;

        if (isCdcSerialON) {

            /* Flush PDC buffer */
            size = DATABUFFERSIZE - BASE_USART->US_RCR;
            if (size == 0) {

                pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
                return;
            }
            BASE_USART->US_RCR = 0;

            /* Debug log: US->USB */
            _DebugLog(hdrS2U, 3, usartBuffers[usartCurrentBuffer], size);
            //_DebugLog("\n\r>", 3, usartBuffers[usartCurrentBuffer], size);

            /* Send current buffer through the USB */
            while (CDCDSerialDriver_Write(usartBuffers[usartCurrentBuffer],
                                          size, 0, 0) != USBD_STATUS_SUCCESS);

            /* Restart read on buffer */
            USART_ReadBuffer(BASE_USART,
                             usartBuffers[usartCurrentBuffer],
                             DATABUFFERSIZE);
            usartCurrentBuffer = 1 - usartCurrentBuffer;
        }
        pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
    }
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 *----------------------------------------------------------------------------*/
static void _UsbDataReceived(uint32_t unused,
                             uint8_t status,
                             uint32_t received,
                             uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Send back CDC data */
        if (isCdcEchoON) {

            CDCDSerialDriver_Write(usbBuffer, received, 0, 0);
        }

        /* Debug log: USB->US */
        _DebugLog(hdrU2S, 3, usbBuffer, received);
        //_DebugLog("\n\r<", 3, usbBuffer, received);

        /* Send data through USART */
        if (isCdcSerialON) {

            while (!USART_WriteBuffer(BASE_USART, usbBuffer, received));
            BASE_USART->US_IER = US_IER_TXBUFE;
        }

        /* Check if bytes have been discarded */
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            TRACE_WARNING(
                      "_UsbDataReceived: %u bytes discarded\n\r", (unsigned int)remaining);
        }
    }
    else {

        TRACE_WARNING( "_UsbDataReceived: Transfer error\n\r");
    }
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been sent.
 *----------------------------------------------------------------------------*/
static void _UsbDataSent( void )
{
    txDoneFlag = 1;
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
    if (!isCdcSerialON) {

        BASE_USART->US_IDR = 0xFFFFFFFF;
        return;
    }

    /* Buffer has been read successfully */
    if ((status & US_CSR_ENDRX) != 0) {

        /* Disable timer */
        TC_Stop(TC0, 0);

        /* Debug Log: US->USB */
        _DebugLog(hdrS2U, 3,
                 usartBuffers[usartCurrentBuffer], DATABUFFERSIZE);
        //_DebugLog("\n\r>", 3,
        //         usartBuffers[usartCurrentBuffer], DATABUFFERSIZE);

        /* Send buffer through the USB */
        while (CDCDSerialDriver_Write(usartBuffers[usartCurrentBuffer],
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
        CDCDSerialDriver_Read(usbBuffer,
                              DATABUFFERSIZE,
                              (TransferCallback) _UsbDataReceived,
                              0);
        BASE_USART->US_IDR = US_IER_TXBUFE;
    }

    /* Errors */
    serialState = CDCDSerialDriver_GetSerialState();

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

    CDCDSerialDriver_SetSerialState(serialState);
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
    PMC->PMC_USB = PMC_USB_USBDIV(1)       /* /2   */
                 | PMC_USB_USBS;           /* PLLB */
}

/**
 * Configure USART to work @ 115200
 */
static void _ConfigureUsart(void)
{
    PIO_Configure(pins, PIO_LISTSIZE(pins));
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
static void _ConfigureTc0(void)
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
    /* Start TC when USB connected & RX enabled */
}

/**
 * Test USB CDC Serial Speed
 */
static void _TestSpeed(void)
{
    uint32_t startT, endT;
    uint32_t i, testCnt;

    if (!isCdcSerialON) {
        printf("\n\r!! Host serial program not ready!\n\r");
        return;
    }
    printf("\n\r- USB CDC Serial Speed test:\n\r");

    /* Test data initialize */
    for (i = 0; i < TEST_BUFFER_SIZE; i ++) testBuffer[i] = (i % 10) + '0';

    printf("- Send 0,1,2 ... to host:\n\r");
    startT = tcTick;
    for (testCnt = 0; testCnt < TEST_COUNT; testCnt ++) {
        txDoneFlag = 0;
        CDCDSerialDriver_Write(testBuffer,
                               TEST_BUFFER_SIZE,
                               (TransferCallback) _UsbDataSent, 0);
        while(!txDoneFlag);
    }
    /* Finish sending */
    CDCDSerialDriver_Write(testBuffer, 0, 0, 0);
    endT = tcTick;
    printf("- Done: Size %d, Count %d, Time %d ~ %d\n\r", (int)TEST_BUFFER_SIZE, (int)testCnt, (int)startT, (int)endT);
    printf("- Speed %dKB/s\n\r", (int)((TEST_BUFFER_SIZE*testCnt)/4/(endT-startT)));
}

/*----------------------------------------------------------------------------
 *          Main
 *----------------------------------------------------------------------------*/

/**
 * \brief usb_cdc_serial Application entry point.
 *
 * Initializes drivers and start the USB <-> Serial bridge.
 */
int main(void)
{
    uint8_t isUsbConnected = 0;

    /* Disable watchdog */
    WDT_Disable( WDT );

    /* Output example information */
    printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Configure USART */
    _ConfigureUsart();

    /* Configure timer 0 */
    _ConfigureTc0();

    /* CDC serial driver initialization */
    CDCDSerialDriver_Initialize(&cdcdSerialDriverDescriptors);

    /* Help informaiton */
    _DebugHelp();

    /* connect if needed */
    VBus_Configure();

    /* Driver loop */
    while (1) {

        /* Device is not configured */
        if (USBD_GetState() < USBD_STATE_CONFIGURED) {

            if (isUsbConnected) {

                isUsbConnected = 0;
                isCdcSerialON  = 0;
                TC_Stop(TC0, 0);
            }
        }
        else if (isUsbConnected == 0) {

            isUsbConnected = 1;
            TC_Start(TC0, 0);
        }

        /* Serial port ON/OFF */
        if (CDCDSerialDriver_GetControlLineState() & CDCControlLineState_DTR) {

            if (!isCdcSerialON) {

                isCdcSerialON = 1;

                /* Start receiving data on the USART */
                usartCurrentBuffer = 0;
                USART_ReadBuffer(BASE_USART, usartBuffers[0], DATABUFFERSIZE);
                USART_ReadBuffer(BASE_USART, usartBuffers[1], DATABUFFERSIZE);
                BASE_USART->US_IER = US_CSR_ENDRX
                                     | US_CSR_FRAME
                                     | US_CSR_OVRE;
                /* Start receiving data on the USB */
                CDCDSerialDriver_Read(usbBuffer,
                                      DATABUFFERSIZE,
                                      (TransferCallback) _UsbDataReceived,
                                      0);
            }
        }
        else if (isCdcSerialON) {

            isCdcSerialON = 0;
        }

        if (UART_IsRxReady()) {

            uint8_t key = UART_GetChar();
            /* ESC: CDC Echo ON/OFF */
            if (key == 27) {
                printf("** CDC Echo %s\n\r",
                       isCdcEchoON ? "OFF" : "ON");
                isCdcEchoON = !isCdcEchoON;
            }
            /* Tab: DBG String ON/OFF */
            else if (key == 9) {
                printf("** DBG Log %s(<:USB->US,>:US->USB)\n\r",
                       isDbgStrON ? "OFF" : "ON");
                isDbgStrON = !isDbgStrON;
            }
            /* 't': Test CDC writing speed */
            else if (key == 't') {
                _TestSpeed();
            }
            else {
                printf("Alive\n\r");
                CDCDSerialDriver_Write("Alive\n\r", 8, 0, 0);
                _DebugHelp();
            }
        }

    }
}

