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

#include <string.h>
#include <stdio.h>

/**
 * \addtogroup SAMGUI
 * @{
 *   \addtogroup SAMGUI_WGT
 *   @{
 *     \addtogroup SAMGUI_WGT_CORE
 *     @{
 *       \addtogroup SAMGUI_WGT_CORE_WIDGETS WGT Core Widgets
 *       @{
 *
 * \brief WGT Widgets.
 */

/**
 * Initializes a widget structure
 */
extern SWGT_Widget* WGT_CreateWidget( WGT_Type eType, uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight )
{
    SWGT_Widget* pWidget ;
//    if ( pWidget == NULL )
//    {
//        return SAMGUI_E_BAD_POINTER ;
//    }

    pWidget=SAMGUI_Malloc( sizeof( SWGT_Widget ) ) ;
    if ( pWidget )
    {
        memset( pWidget, 0, sizeof( SWGT_Widget ) ) ;
        pWidget->dwType=eType ;
        pWidget->dwX=dwX ;
        pWidget->dwY=dwY ;
        pWidget->dwWidth=dwWidth ;
        pWidget->dwHeight=dwHeight ;
    }

    return pWidget ;
}

/**
 * Initializes a widget structure
 */
extern void WGT_DestroyWidget( SWGT_Widget* pWidget )
{
    if ( pWidget != NULL )
    {
        SAMGUI_Free( pWidget ) ;
    }
}

/**
 * Sets a widget background color
 */
extern uint32_t WGT_SetBkgndColor( SWGT_Widget* pWidget, uint32_t dwColor )
{
    if ( pWidget == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    pWidget->dwClrBackground=dwColor ;

    return SAMGUI_E_OK ;
}

/**
 * Sets a widget text color
 */
extern uint32_t WGT_SetTextColor( SWGT_Widget* pWidget, uint32_t dwColor )
{
    if ( pWidget == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    pWidget->dwClrText=dwColor ;

    return SAMGUI_E_OK ;
}

/**
 * Sets a widget text
 */
extern uint32_t WGT_SetText( SWGT_Widget* pWidget, char* pszText )
{
    if ( (pWidget == NULL) || (pszText == NULL) )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    pWidget->pvText=pszText ;

    return SAMGUI_E_OK ;
}

/**
 * Sets a widget bitmap
 */
extern uint32_t WGT_SetBitmap( SWGT_Widget* pWidget, uint8_t* pucBitmap )
{
    if ( (pWidget == NULL) || (pucBitmap == NULL) )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    pWidget->pvBitmap=pucBitmap ;

    return SAMGUI_E_OK ;
}

/**
 * Draws a widget
 */
extern uint32_t WGT_Draw( SWGT_Widget* pWidget, SDISPBackend* pBE )
{
    SGUIColor clr ;

    if ( (pWidget == NULL) || (pBE == NULL) )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    // if bitmap declared, draw it
    if ( pWidget->pvBitmap != NULL )
    {
        pBE->DrawBitmap( pWidget->dwX, pWidget->dwY, pWidget->dwWidth, pWidget->dwHeight, pWidget->pvBitmap ) ;
    }
    // else if colors, draw background color filled rectangle
    else
    {
        clr.u.dwRGBA=pWidget->dwClrBackground ;

        pBE->DrawFilledRectangle( pWidget->dwX, pWidget->dwY, pWidget->dwX+pWidget->dwWidth, pWidget->dwY+pWidget->dwHeight, NULL, &clr ) ;
    }

    // if label declared, draw text
    if ( (pWidget->pvText != NULL) && (pWidget->dwType == WGT_TYPE_TEXT) )
    {
        clr.u.dwRGBA=pWidget->dwClrText ;

        pBE->DrawText( pWidget->dwX, pWidget->dwY, pWidget->pvText, &clr, (void*)&g_Font10x14, 0 ) ;
    }

    return SAMGUI_E_OK ;
}

/** @}
 * @}
 * @}
 * @} */
