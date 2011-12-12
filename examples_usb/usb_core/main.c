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
 *  \page usb_core USB Device Enumeration Example
 *
 *  \section Purpose
 *
 *  The USB Device Enumeration Example will help you to get familiar with the
 *  USB Device Port(UDP)interface on SAM microcontrollers.
 *
 *  You can find following information depends on your needs:
 *  - Sample usage of USB Device Framework.
 *  - USB enumerate sequence, the standard and class-specific descriptors and
 *    requests handling.
 *  - The initialize sequence and usage of UDP interface.
 *
 *  \section See
 *  - pio: Pin configurations and peripheral configure.
 *  - usb: USB Device Framework and UDP interface driver
 *     - \ref usbd_framework
 *       - \ref usbd_api
 *     - \ref usbd_enum
 *
 *  \section Requirements
 *
 *  This package can be used with all Atmel evaluation kits that have UDP
 *  interface.
 *
 *  \section Description
 *
 *  When an EK running this program connected to a host (PC for example), with
 *  USB cable, host will notice the attachment of a USB %device. No %device
 *  driver offered for the %device now.
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
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear:
 *      \code
 *      -- USB Device Core Project xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *  -# When connecting USB cable to windows, the LED blinks, and the host
 *     reports a new USB %device attachment.
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_core example.
 *
 *  \section Contents
 *
 *  The code can be roughly broken down as follows:
 *     - Configuration functions
 *        - VBus_Configure
 *        - PIO configurations in start of main
 *     - Interrupt handlers
 *        - ISR_Vbus
 *     - Callback functions
 *        - USBDCallbacks_RequestReceived
 *     - The main function, which implements the program behavior
 *
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "board.h"

#include <USBDescriptors.h>
#include <USBRequests.h>
#include <USBD.h>
#include <USBDDriver.h>

//#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *         Local types
 *----------------------------------------------------------------------------*/

/**  Configuration descriptors with one interface. */
struct SimpleConfigurationDescriptors {

    USBConfigurationDescriptor configuration;
    USBInterfaceDescriptor interface;
};

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/**  Device descriptor. */
const USBDeviceDescriptor usbDeviceDescriptor = {

    sizeof(USBDeviceDescriptor),
    USBGenericDescriptor_DEVICE,
    USBDeviceDescriptor_USB2_00,
    0, // No device class code
    0, // No device subclass code
    0, // No device protocol code
    CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
    0x03EB, // Atmel vendor ID
    0x0001, // Product ID
    0x0001, // Product release 0.01
    0, // No manufacturer string descriptor
    0, // No product string descriptor
    0, // No serial number string descriptor
    1 // One possible configuration
};

/**  Configuration descriptors. */
const struct SimpleConfigurationDescriptors configurationDescriptors = {

    // Configuration descriptor
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_CONFIGURATION,
        sizeof(struct SimpleConfigurationDescriptors),
        0, // No interface in this configuration
        1, // This is configuration #1
        0, // No string descriptor for this configuration
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
    // Interface descriptor
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        0, // This is interface #0
        0, // This is setting #0 for interface
        0, // Interface has no endpoint
        0, // No interface class code
        0, // No interface subclass code
        0, // No interface protocol code
        0, // No string descriptor
    }
};

/**  List of descriptors used by the device. */
const USBDDriverDescriptors usbdDriverDescriptors = {

    &usbDeviceDescriptor,
    (const USBConfigurationDescriptor *) &configurationDescriptors,
    0, // No full-speed device qualifier descriptor
    0, // No full-speed other speed configuration descriptor
    0, // No high-speed device descriptor
    0, // No high-speed configuration descriptor
    0, // No high-speed device qualifier descriptor
    0, // No high-speed other speed configuration descriptor
    0, // No string descriptor
    0  // No string descriptor
};

/** USB standard device driver. */
USBDDriver usbdDriver ;

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


/**
 *        Callbacks re-implementation
 */

/**
 * Invoked after the USB driver has been initialized. By default, configures
 * the UDP/UDPHS interrupt.
 */
void USBDCallbacks_Initialized(void)
{
    NVIC_EnableIRQ(UDP_IRQn);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    USBDDriver_RequestHandler(&usbdDriver, request);
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

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  Initializes the system, connects the USB and waits indefinitely.
 *
 * \callgraph
 */
int main(void)
{
    /* Disable watchdog */
    WDT_Disable( WDT );

    /* Output example information */
    printf( "-- USB Device Core Project %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0) ;

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* USB initialization, Disable Pull-up */
    TRACE_INFO("USB initialization\n\r");
    USBDDriver_Initialize(&usbdDriver, &usbdDriverDescriptors, 0);
    USBD_Init();

    /* Wait about 10ms so that host detach the device to re-enumerate
       Device connection */
    TRACE_INFO("Connecting device\n\r");

    /* connect if needed */
    VBus_Configure();
    while (USBD_GetState() < USBD_STATE_CONFIGURED);

    // Infinite loop
    while (1) {
    }
}

