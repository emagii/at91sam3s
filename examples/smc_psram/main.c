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
 * \page smc_psram SMC Psram Example
 *
 * \section Purpose
 *
 * This example shows how to configure the Static Memory Controller (SMC)
 * for PSRAM, and do write and read operations to check the configuration.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * An accurate one-to-one comparison is necessary between Psram and SMC waveforms for
 * a complete SMC configuration.
 * * The required steps are:
 * <ul>
 * <li> Configure EBI I/O for psram connection </li>
 * <li> Configure PMC to enable the SMC clock.</li>
 * <li> Refer to the "AC Characteristics" section of the customer psram datasheet.</li>
 * <li> Configure SMC timing to fix the characteristics</li>
 * <li>	Configure SMC mode register for page mode if any.</li>
 * <li>	Access the psram for write or read.</li>
 * </ul>
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
 *     -- SMC Psram Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *
 *     -I- Configure EBI I/O for psram connection.
 *     -I- Configure PMC to enable the SMC clock.
 *     -I- Configure SMC timing and mode.
 *     -I- SMC Setup Register 0xxxxxxxxx.
 *     -I- SMC Pulse Register 0xxxxxxxxx.
 *     -I- SMC Cycle Register 0xxxxxxxxx.
 *     -I- SMC MODE  Register 0xxxxxxxxx.
 *     -I- CTest external PSRAM access.
 *    \endcode
 *
 * \section References
 * - smc_psram/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the smc_psram example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/** Base address of chip select */
#define PSRAM_BASE_ADDRESS         (0x61000000)

/** 8-bits Data bus on EBI */
#define PIN_DATA_BUS_ON_EBI         {0x000000ff, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Read Signal (output) */
#define PIN_NRD_ON_EBI              {1 << 11, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Write Enable Signal (output) */
#define PIN_NWE_ON_EBI              {1 << 8,  PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Static Memory Controller Chip Select Lines */
#define PIN_NCS1_ON_EBI             {1 << 15, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Address Bus A0-A20 */
#define PIN_ADDR_BUS_ON_EBI         {0xfffc0000, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP},\
                                    {0x019c0003, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP}

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIOs configuration for EBI connect to Psram. */
const Pin pinPsram[] = {PIN_DATA_BUS_ON_EBI, \
                        PIN_NRD_ON_EBI, \
                        PIN_NWE_ON_EBI, \
                        PIN_NCS1_ON_EBI,\
                        PIN_ADDR_BUS_ON_EBI
                        };

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Go/No-Go test of the first 10K-Bytes of external PSRAM access.
   \return 0 if test is failed else 1.
 */

static uint8_t AccessPsramTest(void)
{
    uint32_t i;
    uint32_t *ptr = (uint32_t *) PSRAM_BASE_ADDRESS;

    for (i = 0; i < 10 * 1024; ++i) {
        if (i & 1) {
            ptr[i] = 0x55AA55AA | (1 << i);
        }
        else {
            ptr[i] = 0xAA55AA55 | (1 << i);
        }
    }
    for (i = 0; i < 10 * 1024; ++i) {
        if (i & 1) {
            if (ptr[i] != (0x55AA55AA | (1 << i))) {
                return 0;
            }
        }
        else {
            if (ptr[i] != (0xAA55AA55 | (1 << i))) {
                return 0;
            }
        }
    }
    return 1;
}


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for smc_psram example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    /* Disable watchdog */
    WDT_Disable( WDT );

    /* Output example information */
    printf("\n\r\n\r\n\r");
    printf("-- SMC Psram Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure EBI I/O for psram connection*/
    printf("-I- Configure EBI I/O for psram connection.\n\r");
    PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

    /* complete SMC configuration between PSRAM and SMC waveforms.*/
    BOARD_ConfigurePSRAM( SMC ) ;

    /* Test external PSRAM access */
    printf("-I- CTest external PSRAM access. \n\r");
    if (AccessPsramTest()) {
        printf("-I- Access Psram successful.\n\r");
    }
    else {
        printf("-I- Access Psram failed.\n\r");
    }
    return 0;
}
