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
 * \file
 *
 * Implementation of memories configuration on board.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Configures the EBI to initialize and configure the PSRAM
 */
extern void BOARD_ConfigurePsram( void )
{
    const Pin pinPsram[] = {BOARD_PSRAM_PINS} ;
    uint32_t dwTmp ;

    /* Enable peripheral clock */
    PMC_EnablePeripheral( ID_SMC ) ;

    /* Configure I/O */
    PIO_Configure( pinPsram, PIO_LISTSIZE( pinPsram ) ) ;

    //PSRAM IS66WV51216BLL
    // 55 ns Access time
    // tdoe = 25 ns max
    // SMC1 (timing SAM3S read mode SMC) = 21 ns of setup
    // 21 + 55 = 76 ns => at least 5 cycles at 64 MHz
    // Write pulse width minimum = 45 ns (PSRAM)
    SMC->SMC_CS_NUMBER[0].SMC_SETUP = SMC_SETUP_NWE_SETUP( 1 )
                        | SMC_SETUP_NCS_WR_SETUP( 0 )
                        | SMC_SETUP_NRD_SETUP( 2 )
                        | SMC_SETUP_NCS_RD_SETUP( 0 )
                        ;
    SMC->SMC_CS_NUMBER[0].SMC_PULSE = SMC_PULSE_NWE_PULSE( 3 )
                        | SMC_PULSE_NCS_WR_PULSE( 4 )
                        | SMC_PULSE_NRD_PULSE( 3 )
                        | SMC_PULSE_NCS_RD_PULSE( 5 )
                        ;
    SMC->SMC_CS_NUMBER[0].SMC_CYCLE = SMC_CYCLE_NWE_CYCLE( 4 )
                        | SMC_CYCLE_NRD_CYCLE( 5 )
                        ;

    dwTmp = SMC->SMC_CS_NUMBER[0].SMC_MODE & ~(SMC_MODE_DBW_Msk) ;
    SMC->SMC_CS_NUMBER[0].SMC_MODE = dwTmp
                        | SMC_MODE_READ_MODE
                        | SMC_MODE_WRITE_MODE
                        | SMC_MODE_DBW_8_BIT ;
}

