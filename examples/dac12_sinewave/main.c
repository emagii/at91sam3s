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
 *  \page dac12_sinewave DAC12 Sinewave Example
 *
 *  \section Purpose
 *
 *  The dac12_sinewave example demonstrates how to use DACC peripheral.
 *
 *  \section Requirements
 *
 *  This package can only be used with sam3s-ek.
 *
 *  \section Description
 *
 *  This application is aimed to demonstrate how to use DACC in free running
 *  mode.
 *
 *  The example allows to configure the frequency and amplitude of output
 *  sinewave. The frequency could be set from 200Hz to 3KHz, and the peak amplitude
 *  could be set from 100 to 2047 in regard to 12bit resolution.
 *
 *  The output could be monitored by connecting PB13/DAC0 to one channel of an
 *  oscilloscope or heard by ear when plugging a headphone to J11 on the EK board.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or to the
 *     <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *     IAR EWARM User Guide</a>,
 *     depending on your chosen solution.
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- DAC12 Sinewave Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      -- 0: Set frequency(200Hz-3kHz).--
 *      -- 1: Set amplitude(100-2047).--
 *      -- i: Display present frequency and amplitude.--
 *      -- m: Display this menu.--
 *     \endcode
 *  -# Input command according to the menu.
 *
 *  \section References
 *  - dac12_sinewave/main.c
 *  - dacc.h
 */

/** \file
 *
 *  This file contains all the specific code for the dac12_sinewave.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Reference voltage for DACC,in mv*/
#define VOLT_REF   (3300)

/** The maximal digital value*/
#define MAX_DIGITAL (4095)

/** SAMPLES per cycle*/
#define SAMPLES (100)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** channel 0 */
uint8_t DACC_channel_sine = DACC_CHANNEL_0;

/** current index_sample */
uint8_t index_sample = 0;
/** frequency */
uint16_t frequency = 0;
/** amplitude */
uint16_t amplitude = 0;
/** amplitude is MAX_DIGITAL/2,SAMPLES is fixed at 100 */
const int16_t sine_data[SAMPLES]=
{
    0x0,   0x080, 0x100, 0x17f, 0x1fd, 0x278, 0x2f1, 0x367, 0x3da, 0x449,
    0x4b3, 0x519, 0x579, 0x5d4, 0x629, 0x678, 0x6c0, 0x702, 0x73c, 0x76f,
    0x79b, 0x7bf, 0x7db, 0x7ef, 0x7fb, 0x7ff, 0x7fb, 0x7ef, 0x7db, 0x7bf,
    0x79b, 0x76f, 0x73c, 0x702, 0x6c0, 0x678, 0x629, 0x5d4, 0x579, 0x519,
    0x4b3, 0x449, 0x3da, 0x367, 0x2f1, 0x278, 0x1fd, 0x17f, 0x100, 0x080,

    -0x0, -0x080, -0x100, -0x17f, -0x1fd, -0x278, -0x2f1, -0x367,  -0x3da, -0x449,
    -0x4b3, -0x519, -0x579, -0x5d4, -0x629, -0x678, -0x6c0, -0x702, -0x73c, -0x76f,
    -0x79b, -0x7bf, -0x7db, -0x7ef, -0x7fb, -0x7ff, -0x7fb, -0x7ef, -0x7db, -0x7bf,
    -0x79b, -0x76f, -0x73c, -0x702, -0x6c0, -0x678, -0x629, -0x5d4, -0x579, -0x519,
    -0x4b3, -0x449, -0x3da, -0x367, -0x2f1, -0x278, -0x1fd, -0x17f, -0x100, -0x080
};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Get input from user, the biggest 4-digit decimal number is allowed
 *
 * \param lower_limit the lower limit of input
 * \param upper_limit the upper limit of input
 */
static int16_t _GetInputValue( int16_t lower_limit, int16_t upper_limit )
{
    int8_t i = 0, c ;
    int16_t value = 0 ;
    int8_t length = 0 ;
    int8_t str_temp[5] = { 0 } ;

    while ( 1 )
    {
        c = UART_GetChar() ;
        if ( c == '\n' || c == '\r' )
        {
            printf( "\n\r" ) ;
            break ;
        }

        if ( '0' <= c && '9' >= c )
        {
            printf( "%c", c ) ;
            str_temp[i++] = c ;

            if ( i >= 4 )
            {
                break ;
            }
        }
    }

    str_temp[i] = '\0' ;
    /* input string length */
    length = i ;
    value = 0 ;

    /* convert string to integer */
    for ( i = 0 ; i < 4 ; i++ )
    {
        if ( str_temp[i] != '0' )
        {
            switch ( length - i - 1 )
            {
                case 0 :
                    value += (str_temp[i] - '0') ;
                break ;

                case 1 :
                    value += (str_temp[i] - '0') * 10 ;
                break ;

                case 2 :
                    value += (str_temp[i] - '0') * 100 ;
                break ;

                case 3 :
                    value += (str_temp[i] - '0') * 1000 ;
                break ;
            }

        }
    }

    if ( value > upper_limit || value < lower_limit )
    {
        printf( "\n\r-F- Input value is invalid!\n" ) ;

        return -1 ;
    }

    return value ;
}

/** Display main menu */
static void _DisplayMenuChoices( void )
{
    printf( "-- Menu Choices for this example--\n\r" ) ;
    printf( "-- 0: Set frequency(200Hz-3kHz).--\n\r" ) ;
    printf( "-- 1: Set amplitude(100-2047).--\n\r" ) ;
    printf( "-- i: Display present frequency and amplitude.--\n\r" ) ;
    printf( "-- m: Display this menu.--\n\r" ) ;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

void SysTick_Handler( void )
{
    uint32_t status ;

    status = DACC_GetStatus( DACC ) ;

    /* if conversion is done*/
    if ( (status & DACC_ISR_EOC) == DACC_ISR_EOC )
    {
        index_sample++ ;
        if ( index_sample >= SAMPLES )
        {
            index_sample = 0 ;
        }

        DACC_SetConversionData(DACC, sine_data[index_sample] * amplitude / (MAX_DIGITAL/2) + MAX_DIGITAL/2) ;
    }
}

/**
 *  \brief dac12_sinewave Application entry point.
 *
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
extern int main( void )
{
    uint8_t c ;
    int16_t freq, amp ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- DAC12 Sinewave Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* initialize amplitude and frequency */
    amplitude = MAX_DIGITAL / 2;
    frequency = 1000;

    /*10 us timer*/
    SysTick_Config( BOARD_MCK / (frequency * SAMPLES) ) ;

    /* Initialize DACC */
    DACC_Initialize( DACC,
                    ID_DACC,
                    0, /* Hardware triggers are disabled */
                    0, /* External trigger */
                    0, /* Half-Word Transfer */
                    0, /* Normal Mode (not sleep mode) */
                    BOARD_MCK,
                    8, /* refresh period */
                    0, /* Channel 0 selection */
                    0, /* Tag Selection Mode disabled */
                    16 /*  value of the start up time */);

    /*Enable  channel for potentiometer*/
    DACC_EnableChannel( DACC, DACC_channel_sine ) ;

    /*initialize the DACC_CDR*/
    DACC_SetConversionData( DACC,sine_data[index_sample]*amplitude/(MAX_DIGITAL/2)+MAX_DIGITAL/2);

    /* main menu*/
    _DisplayMenuChoices() ;

    while( 1 )
    {
        c = UART_GetChar() ;

        switch ( c )
        {
            case '0' :
                printf( "Frequency:" ) ;
                freq = _GetInputValue( 200, 3000 ) ;
                printf( "\n\r" ) ;
                printf( "Set frequency to:%dHz\n", freq ) ;

                if ( freq > 0 )
                {
                    SysTick_Config( BOARD_MCK / (freq*SAMPLES) ) ;
                    frequency = freq ;
                }
            break ;

            case '1' :
                printf( "Amplitude:" ) ;
                amp = _GetInputValue( 100, 2047 ) ;
                printf( "\n\r" ) ;
                printf( "Set amplitude to %d \n", amp ) ;
                if ( amp > 0 )
                {
                    amplitude = amp ;
                }
            break ;

            case 'i' :
            case 'I' :
                printf( "-I- Frequency:%d Hz Amplitude:%d\n\r", frequency, amplitude ) ;
			break ;

            case 'm' :
            case 'M' :
                _DisplayMenuChoices() ;
            break ;
		}

        printf( "Press \'m\' or \'M\' to display the main menu again!!\n\r" ) ;
    }
}

