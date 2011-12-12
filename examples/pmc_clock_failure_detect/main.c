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
 *  \page pmc_clock_failure_detect PMC Clock Failure Detect Example
 *
 *  \section Purpose
 *
 *  This example shows how to use the Clock Failure Detection feature of the
 *  SAM3S chip when the external oscillator fails.
 *
 *  \section Requirements
 *
 *  This package can only be used with sam3s-ek.
 *
 *  \section Description
 *   The example requires the chip configured to run on external oscillator,
 *   which is normally the case when using the standard lowlevel init functions. *
 *   When the external oscillator is activated, the XIN pin is automatically set
 *   in XIN mode, and no more in PIO mode.
 *
 *   After startup,it enables Clock Failure Detector Enable feature
 *   (CFDEN in PMC_MOR register),and also the interrupt
 *   of Clock Failure Detector Event(CFDEV through PMC_IER).Then ask the user to
 *   force a oscillator failure by shorting XIN or XOUT pin to ground.The chip
 *   switches automatically MAINCK on the 4/8/12 MHz Fast RC Oscillator clock
 *   and also trigger the CFDEV interrupt.
 *
 *   In the interrupt handler, the example lighting a LED ON to show this
 *   feature works.
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
 *  -# On the computer, open and configure a terminal application for each board
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start application from two boards in sequence.
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- PMC Clock Failure Detect Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# The consequent output message through UART should be:
 *     \code
 *     -I- Short XIN or XOUT to ground to force a clock failure.
 *     \endcode
 *     The user should short XIN or XOUT (97 and 96 of 100-Lead LQFP Pinout) to
 *     ground to force a clock failure. If the failure is detected, the LED D2
 *     would be lighted on in the event handler.
 *  \section References
 *  - pmc_clock_failure_detect/main.c
 *  - pmc.h
 */

/** \file
 *
 *  This file contains all the specific code for the pmc_clock_failure_detect.
 *  example.
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
 *        Local Definitions
 *----------------------------------------------------------------------------*/
#define KEY_CKGR_MOR             (0x37)
#define PMC_ENABLE_IT(mode)      (PMC->PMC_IER |= mode)
#define PMC_CKGR_SETMODE(mode)   (PMC->CKGR_MOR = (mode | CKGR_MOR_KEY(KEY_CKGR_MOR)))

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Handler for PMC interrupt.
 *
 *  Toggle led to indicate the event.
 */
void PMC_IrqHandler(void)
{

  if(PMC->PMC_SR & PMC_SR_CFDEV)
  {
    /*indicate Clock failure detect*/
    LED_Set(0);
  }
}

/**
 *  \brief pmc_clock_failure_detect Application entry point.
 *
 *  Enable Clock Failure Detect funciton in PMC, toggle LED
 *  0 to indicate the event detected
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */

int main(void)
{
    uint32_t ckgr_mor;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- PMC Clock Failure Detect Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* configure led0, D2 on sam3s-ek*/
    LED_Configure(0);

    /* get current mode */
    ckgr_mor = PMC->CKGR_MOR;

    /* Enable Clock Failure Detector Enable (CFDEN in PMC_MOR register) */
    PMC_CKGR_SETMODE( CKGR_MOR_CFDEN | ckgr_mor );

    /* Enable interrupt,Clock Failure Detector Event */
    PMC_ENABLE_IT( PMC_IMR_CFDEV );
    /* Enable PMC interrupt */
    NVIC_EnableIRQ( PMC_IRQn );
    /* turn off led */
    LED_Clear(0);

    printf("-I- Short XIN or XOUT to ground to force a clock failure.\n\r");

    /* infinite loop */
    while(1);

}



