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
 *  \page adc12 ADC12 Example
 *
 *  \section Purpose
 *
 *  The adc12 example demonstrates how to use ADC peripheral.
 *
 *  \section Requirements
 *
 *  This package can be used with sam3s-ek. To enable full scale measurement
 *  of the potentiometer by default configuration. Close jumper jp18 on 1 and 2,
 *  the outer pins of it.
 *
 *  \section Description
 *
 *  This application is aimed to demonstrate how to use ADC with PDC.
 *  The example reserves buffers for transfer. It should start reading
 *  converted data first with PDC enabled. The PDC channel will be
 *  configured and then    conversion is started.
 *
 *  It works as a big size buffer for the ADC peripherals, the data
 *  will be stored immediately without interfering with the processor.
 *  The example will send the converted data to UART to indicate the
 *  conversion for several time is done.
 *
 *  Steps to be implemented:
 *  - Initialize ADC with expected parameters.
 *  - Configure and enable interrupt for ADC.
 *  - Enable PDC reception.
 *  - Handle RXBUFF or ENDRX status in ADC interrupt handler.
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
 *      -- ADC12 Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -#  The application will output converted value to hyperterminal per second.
 *
 *  \section References
 *  - adc12/main.c
 *  - adc.h
 */

/** \file
 *
 *  This file contains all the specific code for the adc12 example.
 *
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


/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE         1

/** Size of the pdc buffer. */
#define PDC_BUF_SIZE  (BUFFER_SIZE)

/** Reference voltage for ADC,in mv*/
#define VOLT_REF   (3300)

/** The maximal digital value*/
#define MAX_DIGITAL (4095)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** channel connected to potentiometer*/
uint8_t ADC_channel_pm = ADC_CHANNEL_5;

/** adc buffer*/
int16_t adc_values[BUFFER_SIZE] = { 0 } ;

/** time stamp */
uint32_t time_stamp = 0;

/** conversion and transfer done */
volatile bool conversionDone = false ;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
void SysTick_Handler( void )
{
    uint32_t status;
    time_stamp++;
    /* simply to get 1s interval*/
    if (time_stamp % 1000 == 0)
    {
        status = ADC_GetStatus( ADC ) ;

        /* if conversion is done*/
        if ( (status & ADC_ISR_EOC5) == ADC_ISR_EOC5 )
        {
            ADC_StartConversion( ADC ) ;
        }
    }
}
/**------------------------------------------------------------------------------
 * Interrupt handler for the ADC.
 *------------------------------------------------------------------------------*/
void ADC_IrqHandler(void)
{
    uint32_t status;

    status = ADC_GetStatus(ADC);

    if ( (status & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF )
    {
        conversionDone = true;

        ADC_ReadBuffer( ADC, adc_values, BUFFER_SIZE ) ;

    }
}
/**
 *  \brief adc12 Application entry point.
 *
 *  Initialize adc to 12bit,enable channel 5
 *  , pdc channel interrupt for potentiometer
 *  and start conversion
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main( void )
{
    uint32_t i ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- ADC12 Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /*1 ms timer*/
    SysTick_Config(BOARD_MCK / (1000));

    /* Initialize ADC */
    ADC_Initialize( ADC, ID_ADC );

    /*  startup = 15
     * for prescal = 4
     *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
     *     ADC clock = 6.4 MHz
     */
    ADC_cfgFrequency( ADC, 15, 4 );

    ADC_check( ADC, BOARD_MCK );


    /* Enable  channel for potentiometer */
    ADC_EnableChannel(ADC, ADC_channel_pm);
    /* Enable ADC interrupt */
    NVIC_EnableIRQ(ADC_IRQn);
    /* Start conversion */
    ADC_StartConversion(ADC);

    ADC_ReadBuffer(ADC,adc_values,BUFFER_SIZE);
    /* Enable PDC channel interrupt */
    ADC_EnableIt(ADC,ADC_IER_RXBUFF);

    while ( 1 )
    {
        while ( !conversionDone ) ;

        if ( conversionDone )
        {
            for ( i = 0 ; i < BUFFER_SIZE ; i++ )
            {
                printf( "Vol:%d mv\r\n", (adc_values[i] * VOLT_REF /MAX_DIGITAL) ) ;
            }
            conversionDone = false ;
        }
    }

}

