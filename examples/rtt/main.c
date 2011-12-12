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
 * \page rtt RTT Example
 *
 * \section Purpose
 *
 * This example demonstrates the Real-Time Timer (RTT) provided on
 * SAM3S microcontrollers. It enables the user to set an alarm and watch
 * it being triggered when the timer reaches the corresponding value.
 *
 * You can configure the \ref rtt_module "RTT" by following steps
 * - SetPrescaler the RTT to 1s (32768)
 * - Initialize the ISR routine which refesh it when 1s is counted down
 * - Enable the RTT Interrtup of the vector
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * When launched, this program displays a timer count and a menu on the terminal,
 * enabling the user to choose between several options.
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
 *     -- RTT Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Time: 0
 *     Menu:
 *     r - Reset timer
 *     s - Set alarm
 *     Choice?
 *    \endcode
 *
 * The user can then choose any of the available options to perform
 * the described action.
 *
 * \section References
 * - rtt/main.c
 * - rtt.c
 * - rtt.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the rtt example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Device state: in the main menu. */
#define STATE_MAINMENU      0
/** Device state: user is setting an alarm time */
#define STATE_SETALARM      1

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

 /** Current device state. */
volatile unsigned char state;

/** New alarm time being currently entered. */
volatile unsigned int newAlarm;

/** Indicates if an alarm has occured but has not been cleared. */
volatile unsigned char alarmed;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Refresh display on terminal.
 *
 * Updates the terminal display to show the current menu and the current time
 * depending on the device state.
 */
static void _RefreshDisplay( void )
{
    printf("%c[2J\r", 27);
    printf("Time: %u\n\r", (unsigned int)RTT_GetTime( RTT ) ) ;

    /* Display alarm */
    if ( alarmed )
    {
        printf("!!! ALARM !!!\n\r");
    }

    /* Main menu */
    if ( state == STATE_MAINMENU )
    {
        printf("Menu:\n\r");
        printf(" r - Reset timer\n\r");
        printf(" s - Set alarm\n\r");
        if ( alarmed )
        {
            printf(" c - Clear alarm notification\n\r");
        }
        printf("\n\rChoice? ");
#if defined (  __GNUC__  )
        fflush(stdout);
#endif    }
    /* Set alarm */
    else
    {
        if (state == STATE_SETALARM)
        {
            printf("Enter alarm time: ");
            if ( newAlarm != 0 )
            {
                printf("%u", newAlarm);
#if defined (  __GNUC__  )
                fflush(stdout);
#endif
            }
        }
    }
}

/**
 * \brief Interrupt handler for the RTT.
 *
 * Displays the current time on the terminal.
 */
void RTT_IrqHandler( void )
{
    uint32_t status ;

    /* Get RTT status */
    status = RTT_GetStatus( RTT ) ;

    /* Time has changed, refresh display */
    if ((status & RTT_SR_RTTINC) == RTT_SR_RTTINC)
    {
        _RefreshDisplay();
    }

    /* Alarm */
    if ((status & RTT_SR_ALMS) == RTT_SR_ALMS)
    {
        alarmed = 1;
        _RefreshDisplay();
    }
}

/**
 * \brief RTT configuration function.
 *
 * Configures the RTT to generate a one second tick, which triggers the RTTINC
 * interrupt.
 */
static void _ConfigureRtt( void )
{
    uint32_t previousTime ;

    /* Configure RTT for a 1 second tick interrupt */
    RTT_SetPrescaler( RTT, 32768 ) ;
    previousTime = RTT_GetTime( RTT ) ;
    while ( previousTime == RTT_GetTime( RTT ) ) ;

    /* Enable RTT interrupt */
    NVIC_DisableIRQ( RTT_IRQn ) ;
    NVIC_ClearPendingIRQ( RTT_IRQn ) ;
    NVIC_SetPriority( RTT_IRQn, 0 ) ;
    NVIC_EnableIRQ( RTT_IRQn ) ;
    RTT_EnableIT( RTT, RTT_MR_RTTINCIEN ) ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for RTT example.
 *
 * Initializes the RTT, displays the current time and allows the user to
 * perform several actions: clear the timer, set an alarm, etc.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    unsigned char c ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- RTT Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure RTT */
    _ConfigureRtt() ;

    /* Initialize state machine */
    state = STATE_MAINMENU ;
    alarmed = 0;
    _RefreshDisplay() ;

    /* User input loop */
    while (1)
    {
        /* Wait for user input */
        c = UART_GetChar() ;

        /* Main menu mode */
        if ( state == STATE_MAINMENU )
        {
            /* Reset timer */
            if ( c == 'r')
            {
                _ConfigureRtt() ;
                _RefreshDisplay() ;
            }
            /* Set alarm */
            else
                if (c == 's')
            {
                state = STATE_SETALARM;
                newAlarm = 0;
                _RefreshDisplay();
            }
            /* Clear alarm */
            else
            {
                if ((c == 'c') && alarmed)
                {
                    alarmed = 0;
                    _RefreshDisplay();
                }
            }
        }
        /* Set alarm mode */
        else
        {
            if (state == STATE_SETALARM)
            {
                /* Number */
                if ((c >= '0') && (c <= '9'))
                {
                    newAlarm = newAlarm * 10 + c - '0';
                    _RefreshDisplay();
                }
                /* Backspace */
                else
                {
                    if ( c == 8 )
                    {
                        newAlarm /= 10;
                        _RefreshDisplay();
                    }
                    /* Enter key */
                    else
                    {
                        if ( c == 13 )
                        {
                            /* Avoid newAlarm = 0 case */
                            if (newAlarm != 0)
                            {
                                RTT_SetAlarm( RTT, newAlarm ) ;
                            }

                            state = STATE_MAINMENU;
                            _RefreshDisplay();
                        }
                    }
                }
            }
        }
    }
}

