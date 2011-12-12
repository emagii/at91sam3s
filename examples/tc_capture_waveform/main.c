/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
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
 * \page tc_capture_waveform TC Capture Waveform Example
 *
 * \section Purpose
 *
 * This example indicate how to use TC in capture mode to measure the pulse frequency
 * and count the total pulse number of an external signal injected on TIOA pin.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 * It generates a waveform on TC0 channel 1 TIOA1 {PIO_PA15), and it capture wave
 * from channel 2 TIOA2 (PIO_PA26). These 2 pins could be found on board extension
 * header. To measure the wavefrom on TIOA1, you shall connect PIO_PA15 to PIO_PA26,
 * configure PIO_PA15 as output pin and PIO_PA26 as input pin.
 *
 * \section Descriptions
 *
 * This example shows how to configure TC in waveform and capture mode.
 * In capture mode, pulse signal (from TIOA1) is set as an input to TIOA2, RA and RB will be
 * loaded when programmed event occurs. When TC interrupt happens, we could read RA and RB
 * value for calculating pulse frequency and pulse number be increased. The current pulse frequency
 * and total pulse number are output on UART.
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li>Select pre-defined waveform frequency and duty cycle to be generated.
 * <li>Configure TC channel 1 as waveform output.
 * <li>Configure TC channel 2 as capture input.
 * <li>Configure capture Register A be loaded when rising edge of TIOA occurs.
 * <li>Configure capture Register B be loaded when failing edge of TIOA occurs.
 * <li>Configure an interrupt for TC and enable the RB load interrupt.
 * <li> 'c' start capture.
 * <li> 's' will stop capture,and dump the informations what have been captured.
 * </ul>
 *
 * \section Usage
 *
 * -# Compile the application.
 * -# Connect the DBGU port of the evaluation board to the computer and open
 * it in a terminal.
 *    - Settings: 115200 bauds, 8 bits, 1 stop bit, no parity, no flow control.
 * -# Download the program inside the evaluation board and run it. Please refer to
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Upon startup, the application will output the following line on the DBGU:
 *    \code
 *     -- TC capture waveform example  xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Choose the item in the following menu to test.
 *    \code
 *     Menu :
 *     ------
 *       Output waveform property:
 *       0: Set Frequency =  120 Hz, Duty Cycle = 30%
 *       1: Set Frequency =  375 Hz, Duty Cycle = 50%
 *       2: Set Frequency =  800 Hz, Duty Cycle = 75%
 *       3: Set Frequency = 1000 Hz, Duty Cycle = 80%
 *       4: Set Frequency = 4000 Hz, Duty Cycle = 55%
 *       -------------------------------------------
 *       c: Capture waveform from TC 0 channel 2
 *       s: Stop capture and display informations what have been captured
 *       h: Display menu
 *     ------
 *    \endcode
 *
 * \section References
 * - tc_capture_waveform/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the tc capture waveform example.
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define PIN_TC0_TIOA1    {PIO_PA15, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_TC0_TIOA2    {PIO_PA26, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/
/** Describes a possible Timer configuration as waveform mode */
struct WaveformConfiguration {
    /** Internal clock signals selection. */
    uint32_t clockSelection;
    /** Waveform frequency (in Hz). */
    uint16_t frequency;
    /** Duty cycle in percent (positive)*/
    uint16_t dutyCycle;
};

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIOs for TC0 */
static const Pin pTcPins[] = {PIN_TC0_TIOA1, PIN_TC0_TIOA2};

/** TC waveform configurations */
static const struct WaveformConfiguration waveformConfigurations[] = {
    {TC_CMR_TCCLKS_TIMER_CLOCK4, 178, 30},
    {TC_CMR_TCCLKS_TIMER_CLOCK3, 375, 50},
    {TC_CMR_TCCLKS_TIMER_CLOCK3, 800, 75},
    {TC_CMR_TCCLKS_TIMER_CLOCK2, 1000, 80},
    {TC_CMR_TCCLKS_TIMER_CLOCK1, 4000, 55}
};

/** Current wave configuration*/
static uint8_t configuration = 0;

/** Number of available wave configurations */
const uint8_t numConfigurations = sizeof(waveformConfigurations)
                                        / sizeof(struct WaveformConfiguration);
/** Capture status*/
static uint32_t _dwCaptured_pulses;
static uint32_t _dwCaptured_ra ;
static uint32_t _dwCaptured_rb ;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu(void)
{
    uint8_t i;
    printf("\n\rMenu :\n\r");
    printf("------\n\r");

    printf("  Output waveform property:\n\r");
    for (i = 0; i < numConfigurations; i++) {
        printf("  %d: Set Frequency = %4u Hz, Duty Cycle = %2u%%\n\r",
                   i,
                   (unsigned int)waveformConfigurations[i].frequency,
                   (unsigned int)waveformConfigurations[i].dutyCycle);
    }
    printf("  -------------------------------------------\n\r");
    printf("  c: Capture waveform from TC 0 channel 2\n\r");
    printf("  s: Stop capture and display informations what have been captured \n\r");
    printf("  h: Display menu \n\r");
    printf("------\n\r\n\r");
}


/**
 * \brief Interrupt handler for the TC0 channel 2.
 */
void TC2_IrqHandler( void )
{
    uint32_t status ;
    status = REG_TC0_SR2 ;

    if ( (status & TC_SR_LDRBS) == TC_SR_LDRBS )
    {
        _dwCaptured_pulses++ ;
        _dwCaptured_ra = REG_TC0_RA2 ;
        _dwCaptured_rb = REG_TC0_RB2 ;
    }
}

/**
 * \brief Configure clock, frequency and dutyclcle in wave mode.
 */
static void TcWaveformConfigure(void)
{
    const uint32_t divisors[5] = {2, 8, 32, 128, BOARD_MCK / 32768};
    uint32_t ra, rc;
    /*  Set channel 1 as waveform mode*/
    REG_TC0_CMR1 = waveformConfigurations[configuration].clockSelection  /* Waveform Clock Selection */
                   | TC_CMR_WAVE                                        /* Waveform mode is enabled */
                   | TC_CMR_ACPA_SET                                    /* RA Compare Effect: set */
                   | TC_CMR_ACPC_CLEAR                                  /* RC Compare Effect: clear */
                   | TC_CMR_CPCTRG;                                     /* UP mode with automatic trigger on RC Compare */
    rc = (BOARD_MCK / divisors[waveformConfigurations[configuration].clockSelection]) / waveformConfigurations[configuration].frequency;
    REG_TC0_RC1 = rc;
    ra = (100 - waveformConfigurations[configuration].dutyCycle) * rc / 100;
    REG_TC0_RA1 = ra;
}

/**
 * \brief Configure TC0 channel 1 as waveform operating mode.
 */
static void TcWaveformInitialize(void)
{
    volatile uint32_t dummy;
    /* Configure the PMC to enable the Timer Counter clock for TC0 channel 1. */
    PMC_EnablePeripheral(ID_TC1);
    /*  Disable TC clock */
    REG_TC0_CCR1 = TC_CCR_CLKDIS;
    /*  Disable interrupts */
    REG_TC0_IDR1 = 0xFFFFFFFF;
    /*  Clear status register */
    dummy = REG_TC0_SR1;
    /* Configure waveform frequency and duty cycle */
    TcWaveformConfigure();
    /* Enable TC0 channel 1 */
    REG_TC0_CCR1 =  TC_CCR_CLKEN | TC_CCR_SWTRG ;
    printf ("Start waveform: Frequency = %d Hz,Duty Cycle = %2d%%\n\r",
            waveformConfigurations[configuration].frequency,
            waveformConfigurations[configuration].dutyCycle);
}

/**
 * \brief Configure TC0 channel 2 as capture operating mode.
 */
static void TcCaptureInitialize(void)
{
    volatile uint32_t dummy;
    /* Configure the PMC to enable the Timer Counter clock TC0 channel 2.*/
    PMC_EnablePeripheral(ID_TC2);
    /*  Disable TC clock */
    REG_TC0_CCR2 = TC_CCR_CLKDIS;
    /*  Disable interrupts */
    REG_TC0_IDR2 = 0xFFFFFFFF;
    /*  Clear status register */
    dummy = REG_TC0_SR2;
    /*  Set channel 2 as capture mode */
    REG_TC0_CMR2 = (TC_CMR_TCCLKS_TIMER_CLOCK2    /* Clock Selection */
                   | TC_CMR_LDRA_RISING           /* RA Loading Selection: rising edge of TIOA */
                   | TC_CMR_LDRB_FALLING          /* RB Loading Selection: falling edge of TIOA */
                   | TC_CMR_ABETRG                /* External Trigger Selection: TIOA */
                   | TC_CMR_ETRGEDG_FALLING );    /* External Trigger Edge Selection: Falling edge */
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for tc_capture_waveform example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    uint8_t ucKey ;
    uint16_t frequence, dutyCycle ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- TC capture waveform example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure PIO Pins for TC0 */
    PIO_Configure( pTcPins, PIO_LISTSIZE( pTcPins ) ) ;

    /* Configure TC0 channel 1 as waveform operating mode */
    printf("Configure TC0 channel 1 as waveform operating mode \n\r");
    TcWaveformInitialize() ;
    /* Configure TC0 channel 2 as capture operating mode */
    printf("Configure TC0 channel 2 as capture operating mode \n\r");
    TcCaptureInitialize() ;

    printf("Please Connect PA15 to PA26 on SAM3S-EK for wave capture test.\n\r");

    /* Configure TC interrupts for TC0 channel 2 only */
    NVIC_DisableIRQ( TC2_IRQn ) ;
    NVIC_ClearPendingIRQ( TC2_IRQn ) ;
    NVIC_SetPriority( TC2_IRQn, 0 ) ;
    NVIC_EnableIRQ( TC2_IRQn ) ;

    /* Display menu */
    DisplayMenu() ;

    while ( 1 )
    {
        ucKey = UART_GetChar() ;

        switch ( ucKey )
        {
            case 'h' :
                DisplayMenu() ;
            break ;

            case 's' :
                if ( _dwCaptured_pulses )
                {
                    REG_TC0_IDR2 = TC_IER_LDRBS ;
                    printf( "Captured %u pulses from TC0 channel 2, RA = %u, RB = %u \n\r",
                            (unsigned int)_dwCaptured_pulses, (unsigned int)_dwCaptured_ra, (unsigned int)_dwCaptured_rb ) ;

                    frequence = (BOARD_MCK / 8) / _dwCaptured_rb;
                    dutyCycle = (_dwCaptured_rb - _dwCaptured_ra) * 100 / _dwCaptured_rb;
                    printf( "Captured wave frequency = %d Hz, Duty cycle = %d%% \n\r", frequence, dutyCycle ) ;

                    _dwCaptured_pulses = 0 ;
                    _dwCaptured_ra = 0 ;
                    _dwCaptured_rb = 0 ;
                }
                else
                {
                    printf( "No waveform has been captured\n\r" ) ;
                }
                printf( "\n\rPress 'h' to display menu\n\r" ) ;
            break ;

            case 'c' :
                printf ("Start capture, press 's' to stop \n\r");
                REG_TC0_IER2 = TC_IER_LDRBS;
                /* Reset and enable the tiimer counter for TC0 channel 2 */
                REG_TC0_CCR2 =  TC_CCR_CLKEN | TC_CCR_SWTRG;
            break ;
            default :
                /* Set waveform configuration #n */
                if ( (ucKey >= '0') && (ucKey <= ('0' + numConfigurations - 1)) )
                {
                    if ( !_dwCaptured_pulses )
                    {
                        configuration = ucKey - '0' ;
                        TcWaveformInitialize() ;
                    }
                    else
                    {
                        printf( "In capturing ... , press 's' to stop capture first \n\r" ) ;
                    }
                }
            break ;
        }
    }
}
