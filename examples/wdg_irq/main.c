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
 * \page wdg_irq Watchdog with IRQ Interrupt Example
 *
 * \section Purpose
 *
 * This example demonstrates user to trigger a watchdog interrupt
 * if the software becomes trapped in a deadlock.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * When launched, this program reloads the watchdog at regular intervals
 * before the timer underflow occurs, a LED is blinked. User could press
 * button1 to make the program run in a infinite loop without
 * reloading the watchdog. So a watchdog interrupt will be triggered, and
 * "Enter watchdog interrupt." will print to terminal.
 *
 * \note
 * -# User can enable a watchdog reset instead of an interrupt by setting
 * WDRSTEN bit in WDT_MR register.
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
 *    \code
 *     -- Watchdog with IRQ Interrupt Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 *
 * The user could press the button1 to trigger a watchdog interrupt.
 *
 * \section References
 * - wdg_irq/main.c
 * - wdt.c
 * - wdt.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the wdg_irq example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** LED used in this program */
#define LED_ID    1

/** LED blink time, in ms */
#define BLINK_PERIOD        300

/** Watchdog period, in ms */
#define WDT_PERIOD           3000
/** Watchdog restart period, in ms */
#define WDT_RESTART_PERIOD   2000

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pushbutton \#1 pin instance. */
const Pin pinPB1 = PIN_PUSHBUTTON_1;

/** Pushbutton \#1 pin event flag. */
volatile bool button1Evt = false;

/** System Tick event flag. */
bool systickEvt = false;

/** System tick increased by 1 every millisecond */
volatile uint32_t gSystick = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Handler for Sytem Tick interrupt.
 *
 *  Set systick event flag & increased gSystick by 1.
 */
void SysTick_Handler( void )
{
    systickEvt = true ;
    gSystick++ ;
}

/**
 *  \brief Handler for Button 1 rising edge interrupt.
 *
 *  Set button1 event flag (button1Evt).
 */
static void _Button1_Handler( const Pin* pPin )
{
    if ( pPin == &pinPB1 )
    {
        button1Evt = true ;
    }
}

/**
 *  \brief Handler for watchdog interrupt.
 */
void WDT_IrqHandler( void )
{
    Wdt *pWdt = WDT ;
    volatile uint32_t dummy ;

    /* Clear status bit to acknowledge interrupt */
    dummy = pWdt->WDT_SR ;

    printf( "Enter watchdog interrupt.\n\r" ) ;
    WDT_Restart( WDT ) ;
    printf( "The watchdog timer was restarted.\n\r" ) ;
}

/**
 *  \brief Configure the Pushbuttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void _ConfigureButtons( void )
{
    /* Configure pios as inputs. */
    PIO_Configure( &pinPB1, 1 ) ;

    /* Adjust pio debounce filter patameters, uses 10 Hz filter. */
    PIO_SetDebounceFilter( &pinPB1, 10 ) ;

    /* Initialize pios interrupt handlers, see PIO definition in board.h. */
    PIO_ConfigureIt( &pinPB1, _Button1_Handler ) ; /* Interrupt on rising edge  */

    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ( (IRQn_Type)pinPB1.id ) ;

    /* Enable PIO line interrupts. */
    PIO_EnableIt( &pinPB1 ) ;
}

/**
 *  \brief Configure LEDs
 *
 *  Configures LED (cleared by default).
 */
static void _ConfigureLeds( void )
{
    LED_Configure( LED_ID ) ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for wdg_irq example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    uint32_t dwPeriod ;

    /* Output example information */
    printf( "-- Watchdog with IRQ Interrupt Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Sys tick configuration. */
    printf( "Configure sys tick to get 1ms tick period.\n\r" ) ;

    if ( SysTick_Config(BOARD_MCK / (1000)) )
    {
        printf( "-F- Systick configuration error\n\r" ) ;
    }

    /* PIO configuration for LEDs and Buttons. */
    PIO_InitializeInterrupts( 0 ) ;
    _ConfigureLeds() ;
    _ConfigureButtons() ;

    /* Configure WDT to trigger a interrupt (or reset) */
    printf( "Enable watchdog with %u millisecond period\n\r", (unsigned int)WDT_PERIOD ) ;
    dwPeriod = WDT_GetPeriod( WDT_PERIOD ) ;

#if 1 /* trigger a watchdog interrupt */
    WDT_Enable( WDT, WDT_MR_WDFIEN | WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT | (dwPeriod << 16) | dwPeriod ) ;
    NVIC_DisableIRQ( WDT_IRQn ) ;
    NVIC_ClearPendingIRQ( WDT_IRQn ) ;
    NVIC_SetPriority( WDT_IRQn, 0 ) ;
    NVIC_EnableIRQ( WDT_IRQn ) ;
#else /* trigger a watchdog reset */
    WDT_Enable( WDT, WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT | (dwPeriod << 16) | dwPeriod ) ;
#endif
    printf( "Press USRPB1 to simulate a deadlock loop.\n\r" ) ;

    while( 1 )
    {
        if ( systickEvt == true )
        {
            systickEvt = false ;

            /* Toggle led at given period */
            if ( (gSystick % BLINK_PERIOD) == 0 )
            {
                LED_Toggle( LED_ID ) ;
            }

            /* Restart watchdog at given period */
            if ( (gSystick % WDT_RESTART_PERIOD) == 0 )
            {
                WDT_Restart( WDT ) ;
            }
        }

        /* Simulate deadlock when button be pressed */
        if ( button1Evt == true )
        {
            printf( "Program enter infinite loop for triggering watchdog interrupt.\n\r" ) ;
            while( 1 ) ;
        }
    }
}

