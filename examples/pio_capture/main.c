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
 *  \page pio_capture PIO Parallel Capture Example
 *
 *  \section Purpose
 *
 *  The PIO Parallel Capture demonstrates how to read data from a CMOS digital image
 *  sensor, a high-speed parallel ADC, a DSP synchronous port in synchronous
 *  mode, etc.... using the PIO Controller.
 *
 *  \section Requirements
 *
 *  This package can only be used with sam3s-ek.
 *
 *  \section Description
 *
 *  The PIO Controller integrates an interface able to read data from a CMOS
 *  digital image sensor, a high-speed parallel ADC, a DSP synchronous port in
 *  synchronous mode, etc....
 *
 *  The application is ocmposed of 2 softwares:
 *  - one for use the PIO Parallel Capture in send mode.
 *  - one for use the PIO Parallel Capture in receive mode.
 *
 *  The use of 2 boards is required. Connect one to the other, and put one board
 *  in send mode and the second in receive mode.
 *  Different choice can be selected for use the data enable pins, or not, and
 *  for sample all data, or only one time out of two.
 *  Pins to be connected beetween the 2 boards:<br />
 *    PA15 PIODCEN1<br />
 *    PA16 PIODCEN2<br />
 *    PA17 AD0<br />
 *    PA18 AD1<br />
 *    PA19 AD2/WKUP9<br />
 *    PA20 AD3/WKUP10<br />
 *    PA21 AD8<br />
 *    PA22 AD9<br />
 *    PA23 PIODCCLK<br />
 *    PA24 PIODC0<br />
 *    PA25 PIODC1<br />
 *    PA26 PIODC2<br />
 *    PA27 PIODC3<br />
 *    PA28 PIODC4<br />
 *    PA29 PIODC5<br />
 *    PA30 PIODC6<br />
 *    PA31 PIODC7<br />
 *    And, of course: GND<br />
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
 *  -# Connect the first board to the second board by connecting:
 *     PA15, PA16, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31 and GND.
 *  -# Start application of the first board.
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- PIO Parallel Capture example x.x --
 *      -- SAM3S-EK
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      Frequency: 64 MHz
 *        Press r to Receive data on PPIO Parallel Capture
 *        Press s to Send data on PIO Parallel Capture
 *      ** SEND mode **
 *      This is for debug purpose only !
 *      Frequency of PIO controller clock must be strictly superior to 2 times
 *      the frequency of the clock of the device which generates the parallel data.
 *
 *      Please, connect the second board, and put it in reception mode
 *        Press y to send data without data enables pins
 *        Press n to send data with data enables pins
 *      We send data without data enables pins
 *        Press a key
 *      We send data without data enables pins
 *        Press a key
 *     \endcode
 *  -# Start application of the second board.
 *  -# Put the software in send mode
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- PIO Parallel Capture example x.x --
 *      -- SAM3S-EK
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      Frequency: 64 MHz
 *        Press r to Receive data on PIO Parallel Capture
 *        Press s to Send data on PIO Parallel Capture
 *      ** RECEIVE mode **
 *        Press y to samples the data when both data enables are active
 *        Press n to samples the data whatever the data enables are
 *      We receive data whatever the data enables are
 *        Press y to samples all the data
 *        Press n to samples the data only one time out of two
 *      Whatever the data enables are, sample all the data
 *      wait
 *      IT ENdOfBufferFull Callback
 *      0x3020100 0x7060504 0xB0A0908 0xF0E0D0C
 *      0x13121110 0x17161514 0x1B1A1918 0x1F1E1D1C
 *      0x23222120 0x27262524 0x2B2A2928 0x2F2E2D2C
 *      0x33323130 0x37363534 0x3B3A3938 0x3F3E3D3C
 *      Whatever the data enables are, sample all the data
 *      wait
 *     \endcode
 *
 *  \section References
 *  - pio_capture/main.c
 *  - pio.h, pio_it.h, pio_capture.c, pio_capture.h
 */

/** \file
 *
 *  This file contains all the specific code for the pio_capture_function.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** PIO PARALLEL CAPTURE */
/** Parallel Capture Mode Data Enable1 */
#define DBG_PIN_PIODCEN1    {PIO_PA15, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/** Parallel Capture Mode Data Enable2 */
#define DBG_PIN_PIODCEN2    {PIO_PA16, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/** Parallel Capture Mode Clock */
#define DBG_PIN_PIODCCLK    {PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
/** Parallel Capture Mode Data */
#define DBG_PIN_PIODC0      {PIO_PA24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC1      {PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC2      {PIO_PA26, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC3      {PIO_PA27, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC4      {PIO_PA28, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC5      {PIO_PA29, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC6      {PIO_PA30, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PIN_PIODC7      {PIO_PA31, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define DBG_PINS_PIODC_DATA DBG_PIN_PIODC0, DBG_PIN_PIODC1, DBG_PIN_PIODC2, \
                            DBG_PIN_PIODC3, DBG_PIN_PIODC4, DBG_PIN_PIODC5, \
                            DBG_PIN_PIODC6, DBG_PIN_PIODC7

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** PIO Parallel Capture pins instance */
static const Pin _pinsPIOCD[] = {DBG_PINS_PIODC_DATA, DBG_PIN_PIODCCLK, \
                                 DBG_PIN_PIODCEN1,    DBG_PIN_PIODCEN2};
static const Pin _pinPIODCEN1 = DBG_PIN_PIODCEN1;
static const Pin _pinPIODCEN2 = DBG_PIN_PIODCEN2;
static const Pin _pinPIODCCLK = DBG_PIN_PIODCCLK;

#define SIZE_BUFF_RECEPT 16
static uint32_t _adwPIO_mes_rx[SIZE_BUFF_RECEPT];

/** Key pressed */
static uint8_t _ucKey;

/** API for PIO Parallel Capture */
static SpioCaptureInit _PioCapture;

/** Test if a callback is received */
static volatile uint8_t ucCbkReceived = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
 * \brief Callback on end of reception transfer
 * \param pParam : for the receive data
 */
/*----------------------------------------------------------------------------*/
static void OnEndOfReceptionTransfer( SpioCaptureInit* pParam )
{
    uint8_t i;

    printf("IT ENdOfRecept Callback\n\r");
    for(i=0; i<SIZE_BUFF_RECEPT; i++)
    {
        printf("0x%X ",(unsigned int)pParam->pData[i]);
    }
    printf("\n\r");
    ucCbkReceived = 1;
    PIO_CaptureDisableIt(PIO_PCISR_RXBUFF|PIO_PCISR_ENDRX);
}

/*----------------------------------------------------------------------------*/
/**
 * \brief Callback on buffer full
 * \param pParam : for the receive data
 */
/*----------------------------------------------------------------------------*/
static void OnReceptionBufferFull( SpioCaptureInit* pParam )
{
    uint8_t i;

    printf("IT ENdOfBufferFull Callback\n\r");
    for(i=0; i<SIZE_BUFF_RECEPT; i++)
    {
        printf("0x%X ",(unsigned int)pParam->pData[i]);
    }
    printf("\n\r");
    ucCbkReceived = 1;
    PIO_CaptureDisableIt(PIO_PCISR_RXBUFF|PIO_PCISR_ENDRX);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
 *  \brief pio_capture Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
/*----------------------------------------------------------------------------*/
extern int main( void )
{
    uint8_t i;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- PIO Parallel Capture example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    printf( "Frequency: %d MHz\n\r", BOARD_MCK/1000000 ) ;

    printf( "  Press r to Receive data on PIO Parallel Capture\n\r" ) ;
    printf( "  Press s to Send data on PIO Parallel Capture\n\r" ) ;
    _ucKey = 0 ;
    while( (_ucKey != 'r') && (_ucKey != 's') )
    {
        _ucKey = UART_GetChar() ;
    }
    if( _ucKey == 'r')
    {
        printf("** RECEIVE mode **\n\r");
        PIO_InitializeInterrupts(0);
        /* Clear Recept buffer */
        for(i=0; i<SIZE_BUFF_RECEPT; i++)
        {
            _adwPIO_mes_rx[i]=0;
        }
        /* Init calback */
        /* PIO_PCRHR register is a WORD */
        _PioCapture.dsize = 2;
        _PioCapture.dPDCsize = 16;
        /* A word, 16x, so we wait 64 bytes */
        /* Buffer for received data */
        _PioCapture.pData = _adwPIO_mes_rx;

        printf("  Press y to samples the data when both data enables are active\n\r");
        printf("  Press n to samples the data whatever the data enables are\n\r");
        _ucKey = 0;
        while( (_ucKey != 'y') && (_ucKey != 'n') )
        {
            _ucKey = UART_GetChar();
        }
        if( _ucKey == 'y')
        {
            /* The parallel capture mode samples the data when both data enables are active. */
            _PioCapture.alwaysSampling = 0;
            printf("We receive data when both data enables are active\n\r");
        }
        else
        {
            /* The parallel capture mode samples the data whatever the data enables are.*/
            _PioCapture.alwaysSampling = 1;
            printf("We receive data whatever the data enables are\n\r");
        }
        printf("  Press y to samples all the data\n\r");
        printf("  Press n to samples the data only one time out of two\n\r");
        _ucKey = 0;
        while( (_ucKey != 'y') && (_ucKey != 'n') )
        {
            _ucKey = UART_GetChar();
        }
        if( _ucKey == 'y')
        {
            /* The parallel capture mode samples all the data */
            _PioCapture.halfSampling = 0;
        }
        else
        {
            /* The parallel capture mode samples the data only one time out of two */
            _PioCapture.halfSampling = 1;
            printf("Data with an even index are sampled\n\r");
        }
        /* Only if halfSampling is set, data with an even index are sampled. */
        _PioCapture.modeFirstSample = 0;
        /* No callback for Data Ready */
        _PioCapture.CbkDataReady = NULL;
        /* No callback for Overrun */
        _PioCapture.CbkOverrun = NULL;
        /* Callback for end of reception */
        _PioCapture.CbkEndReception = OnEndOfReceptionTransfer;
        /* Callback for Reception Buffer Full */
        _PioCapture.CbkBuffFull = OnReceptionBufferFull;
        /* Custom parameter not used in this application */
        _PioCapture.pParam = NULL;

        while(1)
        {
            printf("\n\r");
            if( _PioCapture.alwaysSampling == 0 )
            {
                printf("Data enables are active");
            }
            else
            {
                printf("Whatever the data enables are");
            }
            if( _PioCapture.halfSampling == 0 )
            {
                printf(", sample all the data\n\r");
            }
            else
            {
                printf(" only one time out of two, with an even index\n\r");
            }
            /* Init PIO Parallel Capture */
            ucCbkReceived = 0;
            PIO_CaptureInit( &_PioCapture );
            PIO_CaptureEnable();
            printf("wait\n\r");
            while(ucCbkReceived == 0);
        }
    }
    else if( _ucKey == 's')
    {
        printf("** SEND mode **\n\r");
        printf("This is for debug purpose only !\n\r");
        printf("Frequency of PIO controller clock must be strictly superior to");
        printf(" 2 times the frequency of the clock of the device which");
        printf(" generates the parallel data.\n\r");
        printf("\n\rPlease, connect the second board, and put it in reception mode\n\r");
        PMC_EnablePeripheral( ID_PIOA );

        /* Configure PIO Parrallel Capture pins */
        PIO_Configure(_pinsPIOCD, PIO_LISTSIZE(_pinsPIOCD));

        /* Define PIO used for PIO DC */
        PIOA->PIO_OWER = PIO_OWER_P24 | PIO_OWER_P25 | PIO_OWER_P26 | PIO_OWER_P27
                       | PIO_OWER_P28 | PIO_OWER_P29 | PIO_OWER_P30 | PIO_OWER_P31;

        PIOA->PIO_ODSR = 0x00000000;
        PIO_Clear( &_pinPIODCCLK );

        printf("  Press y to send data without data enables pins\n\r");
        printf("  Press n to send data with data enables pins\n\r");
        _ucKey = 0;
        while( (_ucKey != 'y') && (_ucKey != 'n') )
        {
            _ucKey = UART_GetChar();
        }
        if( _ucKey == 'y')
        {
            while(1)
            {
                printf("\n\rWe send data without data enables pins\n\r");
                /* 0x3020100 0x7060504 0xB0A0908 0xF0E0D0C 0x13121110
                 * 0x17161514 0x1B1A1918 0x1F1E1D1C 0x23222120 0x27262524
                 * 0x2B2A2928 0x2F2E2D2C 0x33323130 0x37363534 0x3B3A3938
                 * 0x3F3E3D3C */
                for( i=0; i<64; i++)
                {
                    /* Set Data */
                    PIOA->PIO_ODSR = (i<<24);
                    /* set Clock */
                    PIO_Set( &_pinPIODCCLK );
                    PIOA->PIO_ODSR = (i<<24);
                    /* Clear clock */
                    PIO_Clear( &_pinPIODCCLK );
                    /* Clear data */
                    PIOA->PIO_ODSR = 0x00000000;
                }
                printf("  Press a key\n\r");
                while( UART_GetChar() == 0 );
            }
        }
        else
        {
            while(1)
            {
                printf("\n\rWe send data with data enables pins\n\r");
                /* 0x3020100 0x7060504 0xB0A0908 0xF0E0D0C 0x13121110
                 * 0x17161514 0x1B1A1918 0x1F1E1D1C 0x23222120 0x27262524
                 * 0x2B2A2928 0x2F2E2D2C 0x33323130 0x37363534 0x3B3A3938
                 * 0x3F3E3D3C */
                for( i=0; i<64; i++)
                {
                    /* set Enable */
                    PIO_Set( &_pinPIODCEN1 );
                    PIO_Set( &_pinPIODCEN2 );
                    /* Set Data */
                    //PIOA->PIO_ODSR = 0xA5000000;
                    PIOA->PIO_ODSR = (i<<24);
                    /* set Clock */
                    PIO_Set( &_pinPIODCCLK );
                    PIOA->PIO_ODSR = (i<<24);
                    /* Clear clock */
                    PIO_Clear( &_pinPIODCCLK );
                    PIOA->PIO_ODSR = 0x00000000;
                    /* Clear Enable */
                    PIO_Clear( &_pinPIODCEN1 );
                    PIO_Clear( &_pinPIODCEN2 );
                }
                printf("  Press a key\n\r");
                while( UART_GetChar() == 0 );
            }
        }
    }

    return 0 ;
}
