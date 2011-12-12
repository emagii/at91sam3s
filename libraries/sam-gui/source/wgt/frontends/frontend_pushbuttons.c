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

#include "libsam_gui.h"

/**
 * \addtogroup SAMGUI_WGT_CORE_FRONTEND WGT Core Frontend
 * @{
 *   \addtogroup SAMGUI_WGT_frontend_Pushbuttons SAM-GUI WGT Pushbuttons Frontend
 *   @{
 */

/** VBus pin instance. */
static const Pin _gPinButton1 = PIN_PUSHBUTTON_1 ;
static const Pin _gPinButton2 = PIN_PUSHBUTTON_2 ;

/**
 * \brief Handles Button2 interrupt.
 */
static void _WFE_PushButtons_IRQHandler( const Pin* pPin )
{
    uint32_t dwKey ;

    if ( pPin == &_gPinButton1 )
    {
        dwKey=WGT_KEY_PB1 ;
    }
    else
    {
        if ( pPin == &_gPinButton2 )
        {
            dwKey=WGT_KEY_PB2 ;
        }
        else
        {
            return ;
        }
    }

    if ( PIO_Get( pPin ) )
    {
        WGT_SendMessageISR( WGT_MSG_KEY_RELEASED, dwKey, 0 ) ;
    }
    else
    {
        WGT_SendMessageISR( WGT_MSG_KEY_PRESSED, dwKey, 0 ) ;
    }
}

static uint32_t _WFE_PushButtons_Initialize( void )
{
    /* Configure PIO */
    PIO_Configure( &_gPinButton1, 1 ) ;
    PIO_Configure( &_gPinButton2, 1 ) ;

    PIO_ConfigureIt( &_gPinButton1, _WFE_PushButtons_IRQHandler ) ;
    PIO_ConfigureIt( &_gPinButton2, _WFE_PushButtons_IRQHandler ) ;

    /* Enable PIO interrupt */
    PIO_EnableIt( &_gPinButton1 ) ;
    PIO_EnableIt( &_gPinButton2 ) ;

    return SAMGUI_E_OK ;
}

static uint32_t _WFE_PushButtons_IOCtl( uint32_t dwCommand, uint32_t* pdwValue, uint32_t* pdwValueLength )
{
    return SAMGUI_E_OK ;
}

const SWGTFrontend sWGT_Frontend_PushButtons=
{
    .sData={ .dwId=WGT_FRONTEND_PUSHBUTTONS },

    .Initialize=_WFE_PushButtons_Initialize,

    .IOCtl=_WFE_PushButtons_IOCtl
} ;

/**
 *   @}
 * @}
 */
