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
 * \page spi_touchscreen SPI Touchscreen Example
 *
 * \section Purpose
 *
 * This example shows how to use SPI to control touchscreen controller (ADS7843).
 * It can also help you to get familiar with the touchscreen configurations
 * and usage, which can be used for fast implementation of your own touchscreen
 * driver and other applications related.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * This example first initialize LCD and touchscreen controller, then let
 * user do calibration for the touchscreen. After calibration is done, user may
 * touch the LCD and the pen position will be output on terminal.
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
 * -# Start the application
 * -# In HyperTerminal, it will show something like
 *     \code
 *     -- SPI_Touchscreen Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# Touch the dots on the LCD to calibrate the touchscreen, the calibration
 *    results will output on the terminal and LCD.
 * -# Touch the LCD, the pen position will output on the terminal
 *    if touchscreen is calibrated ok.
 *
 * \section References
 * - spi_touchscreen/main.c
 * - spi.c
 * - tsd_com.c
 * - tsd_ads7843.c
 * - ads7843.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi_touchscreen example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Handler for SysTick interrupt. Increments the timestamp counter.
 */
void SysTick_Handler( void )
{
    TimeTick_Increment() ;

    /* Call TSD_TimerHandler per 10ms */
    if ( (GetTickCount() % 10) == 0 )
    {
        TSD_TimerHandler() ;
    }
}

/**
 * \brief Callback called when the pen is pressed on the touchscreen.
 *
 * \param x horizontal position (in pixel if ts calibrated).
 * \param y vertical position (in pixel if ts calibrated).
 */
void TSD_PenPressed( uint32_t x, uint32_t y )
{
    printf( "Pen pressed at  (%03u, %03u)\n\r", (unsigned int)x, (unsigned int)y ) ;
}

/**
 * \brief Callback called when the pen is moved on the touchscreen.
 *
 * \param x horizontal position (in pixel if ts calibrated).
 * \param y vertical position (in pixel if ts calibrated).
 */
void TSD_PenMoved( uint32_t x, uint32_t y )
{
    printf("Pen moved at    (%03u, %03u)\n\r", (unsigned int)x, (unsigned int)y);
}

/**
 * \brief Callback called when the touchscreen is released on the touchscreen.
 *
 * \param x horizontal position (in pixel if ts calibrated).
 * \param y vertical position (in pixel if ts calibrated).
 */
void TSD_PenReleased( uint32_t x, uint32_t y )
{
    printf("Pen released at (%03u, %03u)\n\r", (unsigned int)x, (unsigned int)y);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for spi_touchscreen example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
    uint32_t bResult ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- SPI_Touchscreen Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure systick for 1 ms. */
    if ( TimeTick_Configure( BOARD_MCK ) != 0 )
    {
        printf( "-F- Systick configuration error\n\r" ) ;
    }

    /* Initialize LCD */
    LCDD_Initialize() ;
    LCDD_Fill( COLOR_WHITE ) ;

    /* Turn on LCD */
    LCDD_On() ;

    /* Initializes the PIO interrupt management for touchscreen driver */
    PIO_InitializeInterrupts( 0 ) ;

    /* Initialize touchscreen without calibration */
    TSD_Initialize( 0 ) ;

    while ( 1 )
    {
        bResult = TSD_Calibrate() ;
        if ( bResult )
        {
            printf( "-I- Calibration successful !\n\r" ) ;
            break ;
        }
        else
        {
            printf( "-E- Error too big ! Retry...\n\r" ) ;
        }
    }

    /* Infinite loop */
    while ( 1 ) ;
}
