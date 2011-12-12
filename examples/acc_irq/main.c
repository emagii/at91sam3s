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
 *  \page acc_irq ACC IRQ Example
 *
 *  \section Purpose
 *
 *  The acc_irq example demonstrates how to use the ACC peripheral to
 *  detect comparison event on the input pair.
 *
 *  \section Requirements
 *
 *  This package can be used with sam3s-ek. The DAC0 uses 3.3v as reference
 *  voltage so that make sure to close JP2 on ADVREF and 3V3 on the board.
 *
 *  \section Description
 *
 *  The acc_irq is aimed to demonstrate the usage of ACC peripheral with
 *  interrupt support. The DAC0 and AD5 are selected as two inputs.
 *  The user can change the output voltage of DAC0 and change the voltage
 *  on AD5 by adjusting VR1.
 *
 *  The output voltage of DAC0 is ranged from (1/6)*ADVREF to (5/6)*ADVREF,and
 *  the input voltage of AD5 is ranged from 0 to ADVREF.
 *
 *  The comparison event would be generated if the voltage of one input is
 *  changed across the voltage of the other input. Both bigger and less events
 *  could be triggered by default.
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
 *      -- ACC IRQ Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      -- s: Set new DAC0 output voltage.--
 *      -- v: Get voltage on potentiometer.--
 *      -- m: Display this menu again.--
 *      \endcode
 *  -# Input command according to the menu.
 *
 *  \section References
 *  - acc_irq/main.c
 *  - dacc.h
 *  - acc.h
 *  - adc.h
 */

/** \file
 *
 *  This file contains all the specific code for the acc_irq
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Reference voltage for DACC,in mv */
#define VOLT_REF   (3300)

/** The maximal digital value*/
#define MAX_DIGITAL (4095)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** channel connected to potentiometer*/
uint8_t ADC_channel_pm = ADC_CHANNEL_5;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**------------------------------------------------------------------------------
 * Interrupt handler for the ACC.
 *------------------------------------------------------------------------------*/
void ACC_IrqHandler( void )
{
    uint32_t status;

    status = ACC_GetStatus(ACC);
    /*Compare Output Interrupt*/
    if ((status & ACC_IER_CE) == ACC_IER_CE) {

        if (ACC_GetComparisionResult(ACC, status)) {

            printf("-ISR- Voltage Comparison Result: AD5 > DAC0\n\r");

        } else {
            printf("-ISR- Voltage Comparison Result: AD5 < DAC0\n\r");

        }

    }

}

/** Display main menu*/
static void _DisplayMenuChoices( void )
{
    printf("-- Menu Choices for this example--\n\r");
    printf("-- s: Set new DAC0 output voltage.--\n\r");
    printf("-- v: Get voltage on potentiometer.--\n\r");
    printf("-- m: Display this menu again.--\n\r");
}

/**
 * \brief Get voltage from user input,the range
 * is (1/6)*ADVREF~(5/6)*ADVREF (mv)
 */
static int16_t _GetVoltage( void )
{
    int8_t i = 0, c;
    int16_t value = 0;
    int8_t length = 0;
    int8_t str_temp[5] = { 0 };

    while (1)
    {
        c = UART_GetChar() ;
        if (c == '\n' || c == '\r')
        {
            printf("\n\r");
            break;
        }

        if ('0' <= c && '9' >= c)
        {
            printf("%c", c);
#if defined (  __GNUC__  )
    fflush(stdout);
#endif
            str_temp[i++] = c;

            if (i >= 4)
                break;
        }
    }

    str_temp[i] = '\0';
    /*input string length*/
    length = i;
    value = 0;

    /*convert string to integer*/
    for (i = 0; i < 4; i++)
    {
        if (str_temp[i] != '0')
        {
            switch (length - i - 1)
            {
                case 0:
                    value += (str_temp[i] - '0');
                    break;
                case 1:
                    value += (str_temp[i] - '0') * 10;
                    break;
                case 2:
                    value += (str_temp[i] - '0') * 100;
                    break;
                case 3:
                    value += (str_temp[i] - '0') * 1000;
                    break;
            }

        }
    }

    if (value > (5 * VOLT_REF / 6) || value < (1 * VOLT_REF / 6))
    {
        printf("\n\r-F- Invalid voltage!\n");
        return -1;
    }

    return value;
}

/**
 *  \brief acc_irq Application entry point.
 *
 *  Initialize adc to 12bit,enable channel 15,turn on
 *  temp sensor, pdc channel interrupt for temp sensor
 *  and start conversion
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main( void )
{
    uint8_t c;
    int16_t volt = 0;
    uint32_t value = 0;
    volatile uint32_t status = 0x0;
    int32_t volt_dac0 = 0;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- ACC IRQ Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

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

    /* Enable  channel for potentiometer */
    DACC_EnableChannel(DACC, DACC_CHANNEL_0);

    /* Set DAC0 output at ADVREF/2 */
    DACC_SetConversionData(DACC,MAX_DIGITAL/2);

    /* Set voltage */
    volt_dac0 = (MAX_DIGITAL/2) * (2*VOLT_REF/3)/MAX_DIGITAL + VOLT_REF /6;

    /* Initialize ADC */
    ADC_Initialize( ADC, ID_ADC ) ;

    /*  startup = 8:    512 periods of ADCClock
     * for prescal = 4
     *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
     *     ADC clock = 6.4 MHz
     */
    ADC_cfgFrequency( ADC, 8, 4 ) ;
    ADC_check( ADC, BOARD_MCK ) ;

    /* Channel 5 has to be compared */
    ADC_EnableChannel( ADC, ADC_channel_pm ) ;

    /* Initialize ACC */
    ACC_Configure( ACC,
                   ID_ACC,
                   ACC_SELPLUS_AD12B5,
                   ACC_SELMINUS_DAC0,
                   1, /* Analog Comparator Enabled */
                   2, /* any edge of comparator output */
                   0  /* Analog Comparator output is directly processed */ );
    /* select comparison pair */
    ACC_SetComparisionPair( ACC, ACC_SELPLUS_AD12B5, ACC_SELMINUS_DAC0 );

    /* Enable ACC interrupt */
    NVIC_EnableIRQ( ACC_IRQn ) ;

    /* Enable */
    ACC_EnableIt( ACC, ACC_IER_CE ) ;

    _DisplayMenuChoices() ;

    while( 1 )
    {
        c = UART_GetChar() ;
        printf( "input: %c\r\n", c ) ;

        switch ( c )
        {
            case 's':
            case 'S':
                printf("DAC0 output voltage is set to: ");
                volt = _GetVoltage() ;
                printf("\n\r");

                if ( volt > 0 )
                {
                    volt_dac0 = volt;
                    value = ((volt - (VOLT_REF / 6)) *(MAX_DIGITAL * 6)/4)/ VOLT_REF;
                    DACC_SetConversionData( DACC, value ) ;
                }
                else
                {
                    printf("-I- Input voltage is invalid\n\r");
                }
                break;
            case 'v':
            case 'V':
                /* Start conversion */
                ADC_StartConversion(ADC);
                status = ADC_GetStatus(ADC);
                while( (status & ADC_ISR_EOC5) != ADC_ISR_EOC5 )
                {
                    status = ADC_GetStatus(ADC);
                }
                /* Conversion is done */
                value =  ADC_GetConvertedData(ADC,ADC_channel_pm);

                volt = ( value * VOLT_REF ) / MAX_DIGITAL;
                printf("-I- Voltage on potentiometer(AD5) is %d mv\n\r",volt);
                printf("-I- Voltage on DAC0 is %d mv \n\r", (int)volt_dac0);

                break;
            case 'm':
            case 'M':
                _DisplayMenuChoices() ;
                break;
        }
    }
}

