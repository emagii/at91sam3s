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
 *  \page usb_hid_transfer USB HID Transfer Example
 *
 *  \section Purpose
 *
 *  The USB HID Transfer Project will help you to get familiar with the
 *  USB Device Port(UDP) and PIO interface on SAM microcontrollers. Also
 *  it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB Humen
 *  Interface Device class (HID).
 *
 *  You can find following information depends on your needs:
 *  - Sample usage of USB HID driver and PIO driver.
 *  - USB HID driver development based on the USB Framework.
 *  - USB enumerate sequence, the standard and class-specific descriptors and
 *    requests handling.
 *  - The initialize sequence and usage of UDP interface.
 *
 *  \subsection See
 *  - pio: PIO interface driver
 *  - usb: USB Framework, USB HID driver and UDP interface driver
 *     - \ref usbd_framework
 *        - \ref usbd_api
 *     - \ref usbd_hid_tran "hid-Transfer"
 *        - \ref usbd_hid_xfr_drv
 *
 *  \section Requirements
 *
 *  This package can be used with all Atmel evaluation kits that has UDP
 *  interface.
 *
 *  \section Description
 *
 *  When an EK running this program connected to a host (PC for example), with
 *  USB cable, the EK appears as a "USB Human Interface Device" for the host.
 *  Then you can use the client application to read/write on it.
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
 *      -- USB Device HID Transfer Project xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *  -# When connecting USB cable to windows, the LED blinks.
 *     Then new "HID Transfer Device" appears in the
 *     hardware %device list.
 *  -# Then you can use the PC program !hidTest.exe to check the !device
 *     information and run tests.
 *  -# Find the HID Device whose VID is 03EB and PID is 6201, select item type
 *     and item to see its attributes.
 *  -# Type what you want to send in output edit box, use buttons on the right
 *     side to send. You can see data information in debug terminal.
 *  -# You can use the buttons above the input edit box to read data from
 *     !device of monitor the data, then the data and the status of the buttons
 *     on the board is read and the gray buttons is up or down based on the
 *     buttons status on the board.
 *
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_hid_transfer example
 */

/**
 *         Headers
 */

#include "board.h"

#include <HIDDTransferDriver.h>

#include <stdio.h>
#include <string.h>

/**
 *         Definitions
 */

/** Delay for pushbutton debouncing (ms) */
#define DEBOUNCE_TIME      10

/**
 *         External variables
 */

/** HID Transfer driver descriptors */
extern USBDDriverDescriptors hiddTransferDriverDescriptors;

/**
 *         Internal variables
 */

/** Pins for Buttons */
static Pin pinsButtons[] = { PINS_PUSHBUTTONS } ;

/**
 *  Remote wake-up support (optional)
 */

/** Button for Wake-UP the USB device. */
static const Pin pinWakeUp = PIN_PUSHBUTTON_1 ;

/** Debounce count (in ms) */
static uint32_t _dwDebounceCounter = DEBOUNCE_TIME ;

/**
 *  Interrupt service routine for the System Tick. Debounces the wake-up pin input.
 */
void SysTick_Handler( void )
{
    /* Button released */
    if ( PIO_Get( &pinWakeUp ) )
    {
        _dwDebounceCounter = DEBOUNCE_TIME ;
    }
    /* Button still pressed */
    else
    {
        _dwDebounceCounter-- ;
    }

    /* End of debounce time */
    if ( _dwDebounceCounter == 0 )
    {
        /* Disable debounce timer */
        NVIC_DisableIRQ( SysTick_IRQn ) ;

        _dwDebounceCounter = DEBOUNCE_TIME ;
        HIDDTransferDriver_RemoteWakeUp() ;
    }
}

/**
 *  Configures the System Tick to generate 1ms ticks.
 */
static void _ConfigureSysTick( void )
{
    /* Configure systick for 1 ms. */
    if ( SysTick_Config( BOARD_MCK/1000 ) )
    {
        printf("-F- Systick configuration error\n\r");
    }
}

/**
 *  Interrupt service routine for the remote wake-up pin. Starts the debouncing
 *  sequence.
 */
static void WakeUpHandler( const Pin *pin )
{
    TRACE_DEBUG( "Wake-up handler\n\r" ) ;

    /* Check current level on the remote wake-up pin */
    if ( !PIO_Get( &pinWakeUp ) )
    {
        _ConfigureSysTick() ;
    }
}

/**
 *  Configures the wake-up pin to generate interrupts.
 */
static void ConfigureWakeUp( void )
{
    TRACE_INFO( "Wake-up configuration\n\r" ) ;

    /* Configure PIO */
    PIO_Configure( &pinWakeUp, 1 ) ;
    PIO_ConfigureIt( &pinWakeUp, WakeUpHandler ) ;
    PIO_EnableIt( &pinWakeUp ) ;
}

/**
 *  VBus monitoring (optional)
 */

/** VBus pin instance */
static const Pin pinVbus = PIN_USB_VBUS ;

/**
 *  Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus( const Pin *pPin )
{
    TRACE_INFO( "VBUS " ) ;

    /* Check current level on VBus */
    if ( PIO_Get( &pinVbus ) )
    {
        TRACE_INFO( "conn\n\r" ) ;
        USBD_Connect() ;
    }
    else
    {
        TRACE_INFO( "discon\n\r" ) ;
        USBD_Disconnect() ;
    }
}

/**
 *  Configures the VBus pin to trigger an interrupt when the level on that pin
 *  changes.
 */
static void VBus_Configure( void )
{
    TRACE_INFO( "VBus configuration\n\r" ) ;

    /* Configure PIO */
    PIO_Configure( &pinVbus, 1 ) ;
    PIO_ConfigureIt( &pinVbus, ISR_Vbus ) ;
    PIO_EnableIt( &pinVbus ) ;

    /* Check current level on VBus */
    if ( PIO_Get( &pinVbus ) )
    {

        /* if VBUS present, force the connect */
        TRACE_INFO( "conn\n\r" ) ;
        USBD_Connect() ;
    }
    else
    {
        USBD_Disconnect() ;
    }
}

/*
 *         Internal Functions
 */

/**
 *  Display the buffer, 8 byte a line
 *
 *  \param buffer   Pointer to the data location
 *  \param dwLen    Size of the data
 */
static void _ShowBuffer( uint8_t* pucBuffer, uint32_t dwLen )
{
    uint32_t dw ;

    for ( dw = 0 ; dw < dwLen ; dw++ )
    {
        if ( (dw & 0x7) == 0 )
        {
            printf( "\n\r" ) ;
        }

        printf( " %02x", pucBuffer[dw] ) ;
    }

    printf( "\n\r" ) ;
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
    HIDDTransferDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    HIDDTransferDriver_ConfigurationChangedHandler(cfgnum);
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
 *  Main function
 */

/**
 *  Initializes the system and then monitors buttons, sending the
 *  corresponding character when one is pressed.
 *  \callgraph
 */
int main( void )
{
    uint32_t dwCnt=0 ;
    uint32_t dwLen ;
    uint8_t  iBuffer[64] ;
    uint8_t  oBuffer[64] ;
    uint8_t  bmLEDs=0 ;
    uint8_t  update ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    printf( "-- USB Device HID Transfer Project %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts( 0 ) ;

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    ConfigureWakeUp() ;

    /* If there is on board power, switch it off */

    /* Configure PINs for LEDs and Buttons */
    LED_Configure( LED_RED ) ;
    LED_Configure( LED_BLUE ) ;
    LED_Configure( LED_GREEN ) ;
    PIO_Configure( pinsButtons, PIO_LISTSIZE( pinsButtons ) ) ;

    /* HID driver initialization */
    HIDDTransferDriver_Initialize( &hiddTransferDriverDescriptors ) ;

    /* connect if needed */
    VBus_Configure() ;

    /* Infinite loop */
    while ( 1 )
    {
        if ( USBD_GetState() < USBD_STATE_CONFIGURED )
        {
            continue ;
        }

        update = 0 ;

        dwLen = HIDDTransferDriver_Read( iBuffer, 64 ) ;
        if ( dwLen )
        {
            printf( "Data In(%u):", (unsigned int)dwLen ) ;
            _ShowBuffer( iBuffer, dwLen ) ;

            bmLEDs = iBuffer[0] ;
            update = 1 ;
        }

        dwLen = HIDDTransferDriver_ReadReport( iBuffer, 64 ) ;
        if ( dwLen )
        {
            printf( "Report In(%u):", (unsigned int)dwLen ) ;
            _ShowBuffer( iBuffer, dwLen ) ;

            bmLEDs = iBuffer[0] ;
            update = 1 ;
        }

        /* Update the status of LEDs */
        if ( update && (0x80 & bmLEDs) )
        {
            /* LED1 */
            if ( bmLEDs & 0x01 )
            {
                LED_Set( LED_BLUE ) ;
            }
            else {

                LED_Clear( LED_BLUE ) ;
            }

            /* LED2 */
            if ( bmLEDs & 0x02 )
            {
                LED_Set( LED_GREEN ) ;
            }
            else
            {
                LED_Clear( LED_GREEN ) ;
            }
        }

        /* Update the status of the buttons */
        oBuffer[0] = 0x80 ;
        if ( PIO_Get( &pinsButtons[PUSHBUTTON_BP1]) == 0 )
        {
            oBuffer[0] |= 0x01 ;
        }
        if ( PIO_Get( &pinsButtons[PUSHBUTTON_BP2] ) == 0 )
        {
            oBuffer[0] |= 0x02 ;
        }

        sprintf( (char*)&oBuffer[5], ":%04x:%05u!", (unsigned int)dwCnt, (unsigned int)dwCnt ) ;
        oBuffer[1] = (uint8_t)(dwCnt) ;
        oBuffer[2] = (uint8_t)(dwCnt >> 8) ;
        oBuffer[3] = (uint8_t)(dwCnt >> 16) ;
        oBuffer[4] = (uint8_t)(dwCnt >> 24) ;
        if ( USBD_STATUS_SUCCESS == HIDDTransferDriver_Write( oBuffer, 64, 0, 0 ) )
        {
            dwCnt ++ ;
        }
    }
}
