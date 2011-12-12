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
 *  \page adc12_threshold_wakeup ADC12 Threshold Wakeup Example
 *
 *  \section Purpose
 *
 *  The adc12_threshold_wakeup example demonstrates how to use ADC with
 *  threshold wakeup.
 *
 *  \section Requirements
 *
 *  This package can be used with sam3s-ek. To enable full scale measurement
 *  of the potentiometer by default configuration. Close jumper jp18 on 1 and 2,
 *  the outer pins of it.
 *
 *  \section Description
 *  This example uses TIOA0 as external trigger instead of software trigger for
 *  ADC conversion. The TIOA0 is a 1ms period square wave. The rising edge
 *  during each period would trigger the ADC to start a conversion on channel 5
 *  which is connected to the potentiometer on the sam3s evaluation kit.This
 *  example shows a menu as below upon running,
 *  \code
 *  -- Menu Choices for this example--
 *  -- 0: Display voltage on potentiometer.--
 *  -- 1: Display thresholds.--
 *  -- 2: Modify low threshold.--
 *  -- 3: Modify high threshold.--
 *  -- 4: Choose comparison mode.--
 *  -- i: Display ADC information.--
 *  -- m: Display this main menu.--
 *  -- s: Enter sleep mode.--
 *  \endcode
 *  With the user interface, comparison window and mode could be set. The ADC
 *  supports 4 kinds of comparison events as following:
 *
 *  - lower than the lower threshold.
 *  - higher than the high threshold.
 *  - in the comparison window.
 *  - out of the comparison window.
 *
 *  If the target got an 'S' or 's' from user's input,the core fell in sleep
 *  by __WFI.Tuning the potentiometer to enable the ADC input fall into the
 *  comparison window and then generate a trigger event. The comparison event
 *  will wake the system up.
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
 *      -- ADC12 Threshold Wakeup Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      -- 0: Display voltage on potentiometer.--
 *      -- 1: Display thresholds.--
 *      -- 2: Modify low threshold.--
 *      -- 3: Modify high threshold.--
 *      -- 4: Choose comparison mode.--
 *      -- i: Display ADC information.--
 *      -- m: Display this main menu.--
 *      -- s: Enter sleep mode.--
 *     \endcode
 *  -# Input the command according to the menu.
 *
 *  \section References
 *  - adc12_threshold_wakeup/main.c
 *  - adc.h
 *  - tc.h
 */

/** \file
 *
 *  This file contains all the specific code for the adc12_threshold_wakeup.
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

/** ADC clock */
#define BOARD_ADC_FREQ (6000000)

/** Reference voltage for ADC,in mv*/
#define VOLT_REF   (3300)

/** The maximal digital value*/
#define MAX_DIGITAL (4095)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** channel connected to potentiometer*/
uint8_t ADC_channel_pm = ADC_CHANNEL_5;

/** time stamp */
uint32_t time_stamp = 0;

/** low threshold*/
uint16_t low_threshold = 0;
/** high threshold*/
uint16_t high_threshold = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**------------------------------------------------------------------------------
 * Interrupt handler for the ADC.
 *------------------------------------------------------------------------------*/
void ADC_IrqHandler( void )
{
    uint32_t status, mode ;
    uint16_t adc ;

    /* Disable Compare Interrupt*/
    ADC_DisableIt( ADC, ADC_IDR_COMPE ) ;

    status = ADC_GetStatus( ADC ) ;

    if ( (status & ADC_ISR_COMPE) == ADC_ISR_COMPE )
    {
        mode = ADC_GetCompareMode( ADC ) ;
        adc = ADC_GetConvertedData( ADC, ADC_channel_pm ) ;

        switch ( mode )
        {
            case 0:
                printf("-ISR-:Potentiometer voltage %d mv is below the low "
                       "threshold:%d mv!\n\r", adc * VOLT_REF / MAX_DIGITAL,
                       low_threshold * VOLT_REF / MAX_DIGITAL);
            break;

            case 1:
                printf("-ISR-:Potentiometer voltage %d mv is above the high "
                       "threshold:%d mv!\n\r", adc * VOLT_REF / MAX_DIGITAL,
                       high_threshold * VOLT_REF / MAX_DIGITAL);
            break;

            case 2:
                printf("-ISR-:Potentiometer voltage %d mv is in the comparison "
                       "window:%d mv-%d mv!\n\r", adc * VOLT_REF / MAX_DIGITAL,
                      low_threshold * VOLT_REF / MAX_DIGITAL, high_threshold * VOLT_REF / MAX_DIGITAL);
            break;

            case 3:
                printf("-ISR-:Potentiometer voltage %d mv is out of the comparison"
                       " window:%d mv-%d mv!\n\r", adc * VOLT_REF / MAX_DIGITAL,
                       low_threshold * VOLT_REF / MAX_DIGITAL, high_threshold * VOLT_REF / MAX_DIGITAL);
            break;
        }
    }
}

/**
 *  \brief TC0 configuration
 *
 * Configures Timer Counter 0 (TC0) to generate an interrupt every second. This
 * interrupt will be used to display the number of bytes received on the USART.
 */

static void _ConfigureTc0( void )
{
    /* Enable TC0 peripheral clock*/
    PMC_EnablePeripheral( ID_TC0 ) ;

    /* Configure TC for a 1s (= 1Hz) tick*/
    TC_Configure( TC0, 0, 0x4 | TC_CMR_ACPC_SET | TC_CMR_WAVE | TC_CMR_ACPA_CLEAR | (0x2 << 13) ) ;

    /* 50% duty, 1s frequency */
    TC0->TC_CHANNEL[0].TC_RA = 16384 ;
    TC0->TC_CHANNEL[0].TC_RC = 32768 ;

}
/** Display main menu */
static void _DisplayMenuChoices( void )
{
    printf("-- Menu Choices for this example--\n\r");
    printf("-- 0: Display voltage on potentiometer.--\n\r");
    printf("-- 1: Display thresholds.--\n\r");
    printf("-- 2: Modify low threshold.--\n\r");
    printf("-- 3: Modify high threshold.--\n\r");
    printf("-- 4: Choose comparison mode.--\n\r");
    printf("-- i: Display ADC information.--\n\r");
    printf("-- m: Display this main menu.--\n\r");
    printf("-- s: Enter sleep mode.--\n\r");
}

/** Display current information,including
 * voltage on potentiometer, thresholds and comparison mode.
 */
static void _DisplayInfo( void )
{
    uint32_t adc_value = ADC_GetConvertedData( ADC, ADC_channel_pm ) ;

    printf( "-I- Thresholds: %d mv - %d mv.\n\r", low_threshold * VOLT_REF / MAX_DIGITAL, high_threshold * VOLT_REF / MAX_DIGITAL ) ;
    printf( "-I- Voltage on potentiometer: %u mv.\n\r", (unsigned int)(adc_value * VOLT_REF / MAX_DIGITAL) ) ;
    printf( "-I- Comparison mode is %u\n\r.", (unsigned int)(ADC->ADC_EMR & ADC_EMR_CMPMODE_Msk) ) ;
}

/** Fall asleep by __WFI
 * Enable interrupt first, and disable it after wake up
 */
static void _Fallasleep( void )
{
    while ( 1 )
    {
        printf( "The device is going to fall in sleep!\n\r" ) ;
        /* clear status register*/
        ADC_GetStatus( ADC ) ;

        /* Enable Compare Interrupt*/
        ADC_EnableIt( ADC, ADC_IER_COMPE ) ;

        __WFI() ;

        /* every time waked-up ,break out of the loop*/
        break ;
    }
}

/**
 * Get comparison mode
 */
static uint8_t GetComparisonMode( void )
{
    uint8_t mode = (ADC->ADC_EMR & ADC_EMR_CMPMODE_Msk) ;
    uint8_t c ;

    while ( 1 )
    {
        c = UART_GetChar() ;
        switch ( c )
        {
            case 'a':
            case 'A':
            mode = 0x0;
                break;
            case 'b':
            case 'B':
            mode = 0x1;
                break;
            case 'c':
            case 'C':
            mode = 0x2;
                break;
            case 'd':
            case 'D':
            mode = 0x3;
                break;
            case 'q':
            case 'Q':
                break;
            default:
            continue;
        }
        return mode ;
    }
}

/**
 * \brief Get voltage from user input,the range
 * is 0~3300 (mv)
 */
static int16_t GetVoltage( void )
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
#if defined (  __GNUC__  )
    fflush(stdout);
#endif

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

    if ( value > VOLT_REF )
    {
        printf( "\n\r-F- Too big threshold!\n" ) ;
        return -1 ;
    }

    return value ;
}
/**
 *  \brief adc12_threshold_wakeup Application entry point.
 *
 *  Initialize adc to 12bit,enable channel 5
 *  , hardware trigger with TIOA0 every second
 *  and start conversion
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main( void )
{
    char choice;
    int16_t adc_value;
    int16_t threshold = 0;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- ADC12 Threshold Wakeup Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Initialize threshold */
    low_threshold = 0x0;
    high_threshold = MAX_DIGITAL;

    /* Initialize ADC */
    ADC_Initialize( ADC, ID_ADC );

    /* startup = 10:    640 periods of ADCClock
     * for prescal = 4
     *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
     *     ADC clock = 6.4 MHz
     */
    ADC_cfgFrequency( ADC, 10, 4 );

    ADC_check( ADC, BOARD_MCK );

    ADC_CfgTrigering( ADC, 1 /* HARDWARE trigger */, 0, 0 );

    /* hardware trigger TIOA0*/
    ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG1 ;
    /*Enable  channels for x,y and z*/
    ADC_EnableChannel( ADC, ADC_channel_pm ) ;

    /* Configure TC*/
    _ConfigureTc0() ;

    /*Channel 5 has to be compared*/
    ADC_SetCompareChannel( ADC, ADC_channel_pm ) ;
    /*Compare mode, in the window*/
    ADC_SetCompareMode( ADC, ADC_EMR_CMPMODE_IN ) ;

    /* Setup Threshold*/
    ADC_SetComparisonWindow( ADC, ((high_threshold<<16)|low_threshold) ) ;

    /* enable adc interrupt*/
    NVIC_EnableIRQ( ADC_IRQn ) ;

    /* Disable Compare Interrupt*/
    ADC_DisableIt( ADC, ADC_IDR_COMPE ) ;

    /* Start TC0 and hardware trigger*/
    TC_Start( TC0, 0 ) ;

    /*display main menu*/
    _DisplayMenuChoices() ;

    while( 1 )
    {
        choice = UART_GetChar() ;
        printf( "%c\r\n", choice ) ;

        switch( choice )
        {
            case '0':
                adc_value = ADC_GetConvertedData( ADC, ADC_channel_pm ) ;
                printf( "-I- Current voltage is %d mv, %d%% of ADVREF\n\r", (adc_value*VOLT_REF/MAX_DIGITAL),(adc_value*100/MAX_DIGITAL) ) ;
            break ;

            case '1':
                printf( "-I- Thresholds are 0x%x(%d%%) and 0x%x(%d%%).\n\r", low_threshold, low_threshold*100/MAX_DIGITAL, high_threshold, high_threshold*100/MAX_DIGITAL ) ;
            break ;

            case '2':
                printf( "Low threshold is set to(mv):" ) ;
                threshold = GetVoltage() ;
                printf( "\n\r" ) ;

                if ( threshold >= 0 )
                {
                    adc_value = threshold*MAX_DIGITAL/VOLT_REF ;
                    ADC_SetComparisonWindow( ADC, adc_value | (high_threshold<<16) ) ;
                    /* renew low threshold */
                    low_threshold = adc_value ;
                    printf( "Low threshold is set to 0x%x(%d%%)\n\r", low_threshold, low_threshold*100/MAX_DIGITAL ) ;
                }
            break ;

            case '3':
                printf( "High threshold is set to(mv):" ) ;
                threshold = GetVoltage() ;
                printf( "\n\r" ) ;

                if ( threshold >= 0 )
                {
                    adc_value = threshold*MAX_DIGITAL/VOLT_REF ;
                    ADC_SetComparisonWindow( ADC, low_threshold | (adc_value<<16) ) ;

                    /* renew high threshold */
                    high_threshold = adc_value ;
                    printf( "High threshold is set to 0x%x(%d%%)\n\r", high_threshold, high_threshold*100/MAX_DIGITAL ) ;
                }
            break ;
            case '4' :
                printf( "-a. Below low threshold.\n\r" \
                        "-b. Above high threshold.\n\r" \
                        "-c. In the comparison window.\n\r" \
                        "-d. Out of the comparison window.\n\r"\
                        "-q. Quit the setting.\n\r" ) ;
                choice = GetComparisonMode() ;
                ADC_SetCompareMode( ADC, choice ) ;
                printf( "Comparison mode is %c.\n\r", 'a'+choice ) ;
            break ;

            case 'm' :
            case 'M' :
                _DisplayMenuChoices() ;
            break ;

            case 'i' :
            case 'I' :
                _DisplayInfo() ;
            break ;

            case 's' :
            case 'S' :
                _Fallasleep() ;
            break ;
      }
      printf( "Press \'m\' or \'M\' to display the main menu again!!\n\r" ) ;
    }
}

