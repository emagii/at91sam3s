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
 *  \page adc12_temp_sensor ADC12 Temperature Sensor Example
 *
 *  \section Purpose
 *
 *  The adc12_temp_sensor example demonstrates how to use the temperature sensor
 *  feature inside the microcontroller.
 *
 *  \section Requirements
 *
 *  This package can be used with sam3s-ek. To enable full scale measurement
 *  ,Close jumper jp18 on 1 and 2,the outer pins of it.
 *
 *  \section Description
 *
 *  The adc12_temp_sensor is aimed to demonstrate the temperature sensor feature
 *  inside the device. To use this feature, the temperature sensor should be
 *  turned on by setting TSON bit in ADC_ACR. The channel 15 is connected to the
 *  sensor by default. With PDC support, the Interrupt Handler of ADC is designed
 *  to handle RXBUFF interrupt.
 *
 *  The temperature sensor provides an output voltage (VT) that is proportional
 *  to absolute temperature (PTAT). The relationship between measured voltage and
 *  actual temperature could be found in Electrical Characteristics part of the
 *  datasheet.
 *
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
 *      -- ADC12 Temperature Sensor xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -#  The application will output converted value to hyperterminal per second.
 *
 *  \section References
 *  - adc12_temp_sensor/main.c
 *  - adc.h
 */

/** \file
 *
 *  This file contains all the specific code for the adc12_temp_sensor.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** ADC clock */
#define BOARD_ADC_FREQ (6000000)

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE         (100)

/** Reference voltage for ADC,in mv */
#define VOLT_REF   (3300)

/** The maximal digital value */
#define MAX_DIGITAL (4095)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** channel connected to potentiometer */
uint8_t ADC_channel_temp = ADC_CHANNEL_15;

/** adc buffer */
int16_t adc_values[BUFFER_SIZE] = { 0 };

/** time stamp */
uint32_t time_stamp = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * Systick handler, start new conversion
 */
void SysTick_Handler(void)
{
    uint32_t status;
    time_stamp++;
    /* simply to get 10ms interval */
    if (time_stamp % 10 == 0) {

        status = ADC_GetStatus(ADC);

        if ((status & ADC_ISR_EOC15) == ADC_ISR_EOC15) {

            /* Start conversion */
            ADC_StartConversion(ADC);
        }

    }
}

/** \brief Simple function to replace with printf with float formatting
 *  1 decimal with rounding support
 */
static void _PrintTemp( float temp )
{
    int16_t integer1 = 0;
    int32_t integer2 = 0;

    assert( INT16_MAX > (temp * 100.0) && INT16_MIN < (temp * 100.0) ) ;

    /* cast to integer */
    integer1 = (int16_t) (temp * 100.0);

    /* rounding */
    integer2 = integer1 / 10;
    if ( ( integer1 - integer2 * 10) > 4 )
    {
        integer1 = integer2 + 1;
    }
    else
    {
        if ( ( integer1 - integer2 * 10 ) < -4 )
        {
            integer1 = integer2 - 1;
        }
        else
        {
            integer1 = integer2;
        }
    }

    /* quotient */
    integer2 = integer1 / 10;
    /* remainder */
    integer1 = integer1 - integer2 * 10;

    if (integer1 < 0) {
        printf("Temp:-%d.%d \n\r", (int16_t)((integer2) * (-1)),
                                   (int16_t)((integer1) * (-1)));
    } else {
        printf("Temp:%d.%d \n\r", (int16_t)integer2,(int16_t)integer1);
    }
}

/**------------------------------------------------------------------------------
 * Interrupt handler for the ADC.
 *------------------------------------------------------------------------------*/
void ADC_IrqHandler(void)
{
    uint32_t status, i;
    int32_t vol;
    float temp;
    uint32_t value = 0;
    uint32_t temp_value = 0;

    status = ADC_GetStatus(ADC);

    if ((status & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF) {

        /* multisample */
        for (i = 0; i < BUFFER_SIZE; i++) {
            value += adc_values[i];
        }
        /* averaging */
        temp_value = value / 10;
        value = value / 100;
        temp_value -= (value * 10);
        /* round for last decimal */
        if (temp_value > 4) {
            value++;
        }

        vol = value * VOLT_REF / MAX_DIGITAL;
        /* using multiply instead of dividing /2.65 */
        temp = (float) (vol - 800) * 0.37736 + 27.0;

        _PrintTemp( temp ) ;
        /* clear the buffer */
        memset( adc_values, 0x0, sizeof( adc_values ) ) ;
        /* start new pdc transfer */
        ADC_ReadBuffer( ADC, adc_values, BUFFER_SIZE ) ;

    }
}

/**
 *  \brief adc12_temp_sensor Application entry point.
 *
 *  Initialize adc to 12bit,enable channel 15,turn on
 *  temp sensor, pdc channel interrupt for temp sensor
 *  and start conversion
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main(void)
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- ADC12 Temperature Sensor %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* 1 ms timer */
    SysTick_Config(BOARD_MCK / (1000));

    /* Initialize ADC */
    ADC_Initialize( ADC, ID_ADC );

    /*  startup = 8:    512 periods of ADCClock
     * for prescal = 4
     *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
     *     ADC clock = 6.4 MHz
     */
    ADC_cfgFrequency( ADC, 8, 4 );

    ADC_check( ADC, BOARD_MCK );

    /* Enable  channel for potentiometer */
    ADC_EnableChannel( ADC, ADC_channel_temp ) ;

    /* Enable the temperature sensor */
    ADC_EnableTS( ADC, ADC_ACR_TSON ) ;

    /* Enable ADC interrupt */
    NVIC_EnableIRQ( ADC_IRQn ) ;
    /* Start conversion */
    ADC_StartConversion( ADC ) ;

    ADC_ReadBuffer( ADC, adc_values, BUFFER_SIZE ) ;
    /* Enable PDC channel interrupt */
    ADC_EnableIt( ADC, ADC_ISR_RXBUFF ) ;

    while ( 1 ) ;
}

