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
 * \page usb_hid_keyboard USB HID Keyboard Example
 *
 * \section Purpose
 *
 * The USB HID Keyboard Example will help you to get familiar with the
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
 *      - \ref usbd_api
 *    - \ref usbd_hid_kbd_drv
 *    - hid-keyboard
 *
 * \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, the EK appears as a HID Keyboard for the host. Then you can use
 * the push buttons on the EK to input letter to the host. E.g, to open a
 * editor and input a letter 'a' or '9'.
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
 *     -- USB Device HID Keyboard Project xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks.
 *    Then new "HID Keyboard Device" appears in the
 *    hardware %device list.
 * -# Once the device is connected and configured, pressing any of the board buttons
 *    should send characters to the host PC. Pressing num. lock should also make the third
 *    LED toggle its state (on/off).
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_hid_keyboard
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - VBus_Configure
 *       - ConfigurePit
 *       - ConfigureWakeUp
 *       - PIO & Timer configurations in start of main
 *    - Interrupt handlers
 *       - ISR_Vbus
 *       - ISR_Pit
 *       - WakeUpHandler
 *    - Callback functions
 *       - HIDDKeyboardCallbacks_LedsChanged
 *    - The main function, which implements the program behavior
 *
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *-----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "HIDDKeyboardDriver.h"
#include "include/USBD_LEDs.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*-----------------------------------------------------------------------------
 *         Definitions
 *-----------------------------------------------------------------------------*/

/** Number of keys used in the example. */
#define NUM_KEYS                    2

/** Number of non-modifiers keys. */
#define NUM_NORMAL_KEYS             1

/** Number of modifier keys. */
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/** Num lock LED index. */
#define LED_NUMLOCK                 USBD_LEDOTHER

/*---------------------------------------------------------------------------
 *         External variables
 *---------------------------------------------------------------------------*/

/** Descriptor list for HID keyboard device */
extern USBDDriverDescriptors hiddKeyboardDriverDescriptors;

/*---------------------------------------------------------------------------
 *         Internal variables
 *---------------------------------------------------------------------------*/

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

/*---------------------------------------------------------------------------
 *         Callbacks re-implementation
 *---------------------------------------------------------------------------*/

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
    HIDDKeyboardDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    HIDDKeyboardDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when the status of the keyboard LEDs changes. Turns the num. lock
 * LED on or off.
 * \param numLockStatus  Indicates the current status of the num. lock key.
 * \param capsLockStatus  Indicates the current status of the caps lock key.
 * \param scrollLockStatus  Indicates the current status of the scroll lock key.
 */
void HIDDKeyboardCallbacks_LedsChanged(
    uint8_t numLockStatus,
    uint8_t capsLockStatus,
    uint8_t scrollLockStatus)
{
    printf("%c %c %c\n\r",
        numLockStatus ? 'N':'_',
        capsLockStatus ? 'C':'_',
        scrollLockStatus ? 'S':'_'
        );
    /* Num. lock */
    if (numLockStatus) {

        LED_Set(LED_NUMLOCK);
    }
    else {

        LED_Clear(LED_NUMLOCK);
    }
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

/*---------------------------------------------------------------------------
 *         Exported function
 *---------------------------------------------------------------------------*/

/**
 * Initializes the system and then monitors buttons, sending the
 * corresponding character when one is pressed.
 *  \callgraph
 */
int main(void)
{
    unsigned int i;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB Device HID Keyboard Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Initialize key statuses and configure push buttons */
    PIO_Configure(pinsPushButtons, PIO_LISTSIZE(pinsPushButtons));
    memset(keyStatus, 1, NUM_KEYS);

    /* Configure LEDs */
    LED_Configure(LED_NUMLOCK);

    /* HID driver initialization */
    HIDDKeyboardDriver_Initialize(&hiddKeyboardDriverDescriptors);

    /* connect if needed */
    VBus_Configure();

    /* Infinite loop */
    while (1) {

        uint8_t pressedKeys[NUM_KEYS];
        uint8_t pressedKeysSize = 0;
        uint8_t releasedKeys[NUM_KEYS];
        uint8_t releasedKeysSize = 0;

        if (USBD_GetState() < USBD_STATE_CONFIGURED)
            continue;

        /* Monitor buttons */
        for (i=0; i < PIO_LISTSIZE(pinsPushButtons); i++) {

            /* Check if button state has changed */
            uint8_t isButtonPressed = PIO_Get(&(pinsPushButtons[i]));
            if (isButtonPressed != keyStatus[i]) {

                /* Update button state */
                if (!isButtonPressed) {

                    /* Key has been pressed */
                    printf("BP %u pressed\n\r", i);
                    keyStatus[i] = 0;
                    pressedKeys[pressedKeysSize] = keyCodes[i];
                    pressedKeysSize++;
                    HIDDKeyboardDriver_RemoteWakeUp();
                }
                else {

                    /* Key has been released */
                    printf("BP %u released\n\r", i);
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

                status = HIDDKeyboardDriver_ChangeKeys(pressedKeys,
                                                       pressedKeysSize,
                                                       releasedKeys,
                                                       releasedKeysSize);
            }
            while (status != USBD_STATUS_SUCCESS);
        }

    }
}


