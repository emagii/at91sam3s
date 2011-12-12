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

#include <stdlib.h>
#include <stdio.h>

//#define TRACE( x ) x
#define TRACE( x )

/**
 * \addtogroup SAMGUI
 * @{
 *   \addtogroup SAMGUI_DISP
 *   @{
 *
 * \brief DISP API
 * This API allow connection to a graphical backend and access to drawing primitives
 *
 */

static SDISPBackend* _pBackendBase=NULL ;

extern SDISPBackend* DISP_GetBackend( void )
{
    return _pBackendBase ;
}

/**
 * Allow to get specific backend handle
 */
extern uint32_t DISP_AddBackend( SDISPBackend* pBackend )
{
    // Check pointers
    if ( pBackend == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    if ( _pBackendBase != NULL )
    {
        pBackend->pNext=_pBackendBase ;
        _pBackendBase=pBackend ;
    }
    else
    {
        _pBackendBase=pBackend ;
    }

    TRACE( printf( "DISP_GetBackend - %x %x\r\n", _aDISP_Backends[dwIndex], *ppBackend ) ; )

    return SAMGUI_E_OK ;
}

/**
 *   @}
 * @}
 */
