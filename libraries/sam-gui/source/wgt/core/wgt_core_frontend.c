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
#include <string.h>

//#define TRACE( x ) x
#define TRACE( x )

/**
 * \addtogroup SAMGUI
 * @{
 *   \addtogroup SAMGUI_WGT
 *   @{
 *     \addtogroup SAMGUI_WGT_CORE
 *     @{
 *       \addtogroup SAMGUI_WGT_CORE_FRONTEND WGT Core Frontend
 *       @{
 */

//static __no_init SWGTFrontend* _aWGT_Frontends[WGT_FRONTEND_MAX] ;
static SWGTFrontend* _pFrontendBase=NULL ;


//extern uint32_t WGT_Frontend_Initialize( void )
//{
//    return SAMGUI_E_OK ;
//}

extern uint32_t WGT_Frontend_AddFrontend( SWGTFrontend* pFrontend )
{
    // Check pointers
    if ( pFrontend == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    if ( _pFrontendBase != NULL )
    {
        pFrontend->pNext=_pFrontendBase ;
        _pFrontendBase=pFrontend ;
    }
    else
    {
        _pFrontendBase=pFrontend ;
    }

//    TRACE( printf( "WGT_Frontend_GetFrontend - %x %x\r\n", _aWGT_Frontends[dwIndex], *ppFrontend ) ; )

    return SAMGUI_E_OK ;
}


/** @}
 * @}
 * @}
 * @} */
