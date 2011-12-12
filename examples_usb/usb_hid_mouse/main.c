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
 * \page usb_hid_mouse USB HID Mouse Example
 *
 * \section Purpose
 *
 * The USB HID Mouse Example will help you to get familiar with the
 * USB Device Port(UDP) and PIO interface on SAM microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Humen
 * Interface Device class (HID).
 *
 * You can find following information depends on your needs:
 * - Sample usage of USB HID driver and PIO driver.
 * - USB HID driver development based on the USB Framework.
 * - USB enumerate sequence, the standard and class-specific descriptors and
 *   requests handling.
 * - The initialize sequence and usage of UDP interface.
 *
 * \section See
 * - pio: PIO interface driver
 * - usb: USB Framework, USB HID driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - hid-mouse
 *       - \ref usbd_hid_mouse_drv
 *
 * \section Requirements
 *
 * This package can be used with all Atmel evaluation kits that has UDP
 * interface and have push button or joystick on it.
 *
 * \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, the EK appears as a HID-compliant mouse for the host. Then you
 * can use the joystick or buttons on the EK to control the pointer on the host.
 * E.g., to move it.
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
 *     -- USB Device HID Mouse Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks.
 *    Then new "HID Mouse Device" appears in the
 *    hardware %device list.
 * -# Once the device is connected and configured, pressing the joystick or
 *    the configurated board buttons move the cursor.
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_hid_mouse
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - VBus_Configure
 *       - PIO & Timer configurations in start of main
 *    - Interrupt handlers
 *       - ISR_Vbus
 *    - The main function, which implements the program behavior
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *-----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "HIDDMouseDriver.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/** Speed of pointer movement X */
#define SPEED_X             4

/** Speed of pointer movement Y */
#define SPEED_Y             4


#define JOYSTICK_LEFT  0
#define JOYSTICK_RIGHT 1
/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

extern USBDDriverDescriptors hiddMouseDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

/** List of pinsJoystick (push button) to configure for the application. */
static Pin pinsJoystick[] = {PINS_PUSHBUTTONS};

/*----------------------------------------------------------------------------
 *         Remote wake-up support (optional)
 *----------------------------------------------------------------------------*/

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

/**
 * Monitor buttons of joystick status.
 * \param pBtnStatus Pointer to button status bitmap.
 * \param pDx        Pointer to fill x value.
 * \param pDy        Pointer to fill y value.
 */
static uint8_t _ButtonsMonitor(uint8_t *pBtnStatus,
                               int8_t *pDx,
                               int8_t *pDy)
{
    uint8_t isChanged = 0;

      /* - Movment buttons, Joystick or Push buttons */
      /* Left */
      if (PIO_Get(&pinsJoystick[JOYSTICK_LEFT]) == 0) {

          *pDx = -SPEED_X;
          isChanged = 1;
      }
      /* Right */
      else if (PIO_Get(&pinsJoystick[JOYSTICK_RIGHT]) == 0) {

          *pDx = SPEED_X;
          isChanged = 1;
      }
      else {
          *pDx = 0;
      }

    return isChanged;
}

/*----------------------------------------------------------------------------
 *         Callbacks re-implementation
 *----------------------------------------------------------------------------*/

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
    HIDDMouseDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    HIDDMouseDriver_ConfigurationChangedHandler(cfgnum);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

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

/*----------------------------------------------------------------------------
 *         Exported function
 *----------------------------------------------------------------------------*/

/**
 * usb_hid_mouse application entry.
 *
 * Initializes the system and then monitors buttons, sending the
 * corresponding character when one is pressed.
 */
int main(void)
{
    uint8_t bmButtons = 0;
    int8_t dX = 0, dY = 0;
    uint8_t isChanged;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB Device HID Mouse Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Initialize key statuses and configure push buttons */
    PIO_Configure(pinsJoystick, PIO_LISTSIZE(pinsJoystick));

    /* HID driver initialization */
    HIDDMouseDriver_Initialize(&hiddMouseDriverDescriptors);

    /* connect if needed */
    VBus_Configure();

    /* Infinite loop */
    while (1) {

        if (USBD_GetState() < USBD_STATE_CONFIGURED)
            continue;

        isChanged = _ButtonsMonitor(&bmButtons, &dX, &dY);

        if (isChanged) {

            uint8_t status;

            do {

                status = HIDDMouseDriver_ChangePoints(bmButtons,
                                                      dX, dY);

            } while (status != USBD_STATUS_SUCCESS);
        }

    }
}

