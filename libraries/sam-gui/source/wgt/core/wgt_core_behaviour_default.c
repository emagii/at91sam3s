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

#include <stdio.h>

/**
 * \addtogroup SAMGUI
 * @{
 *   \addtogroup SAMGUI_WGT
 *   @{
 *     \addtogroup SAMGUI_WGT_CORE
 *     @{
 *
 * \brief WGT Core default behaviour using inputs from implemented frontends.
 */

extern uint32_t PreProcessMessage_Default( struct _SWGTScreen* pScreen, SWGTCoreMessage* pMsg )
{
    SWGT_Widget* pWidget ;
    SGUIColor clrBlack={ .u.dwRGBA=GUICLR_BLACK } ;
//    static uint32_t dwPointerTimeStampOld=0 ;
//    static uint32_t dwPointerTimeStamp=0 ;

    // Intercept special message for code factorization
    switch ( pMsg->dwID )
    {
        case WGT_MSG_POINTER :
            // Draw point
            g_WGT_CoreData.pBE->DrawPixel( pMsg->dwParam1, pMsg->dwParam2, &clrBlack ) ;

            if ( WGT_Screen_GetPointedWidget( g_WGT_CoreData.pCurrentScreen, pMsg->dwParam1, pMsg->dwParam2, &pWidget ) == SAMGUI_E_OK )
            {
                if ( WGT_Screen_SetSelectedWidget( g_WGT_CoreData.pCurrentScreen, pWidget ) == SAMGUI_E_OK )
                {
                }
            }
        break ;

        case WGT_MSG_POINTER_RELEASED :

            if ( WGT_Screen_GetPointedWidget( g_WGT_CoreData.pCurrentScreen, pMsg->dwParam1, pMsg->dwParam2, &pWidget ) == SAMGUI_E_OK )
            {
                if ( WGT_Screen_SetSelectedWidget( g_WGT_CoreData.pCurrentScreen, pWidget ) == SAMGUI_E_OK )
                {
                    WGT_PostMessage( WGT_MSG_WIDGET_SELECTED, (uint32_t)pWidget, 0 ) ;
                }
            }
            // pointer has been released, repaint last selected widget
//            if ( pScreen->pWidgetCurrent != NULL )
//            {
//                WGT_Draw( pScreen->pWidgetCurrent, g_WGT_CoreData.pBE ) ;
//                pScreen->pWidgetOld=pScreen->pWidgetCurrent=NULL ;
//            }
        break ;

        case WGT_MSG_INIT :
            // Reset selected widget
            pScreen->pWidgetOld=pScreen->pWidgetCurrent=NULL ;
            if ( g_WGT_CoreData.pCurrentScreen->OnInitialize )
            {
                g_WGT_CoreData.pCurrentScreen->OnInitialize( g_WGT_CoreData.pCurrentScreen ) ;
            }
        break ;

        case WGT_MSG_ERASE_BKGND :
            // Call 'before' hook
            if ( g_WGT_CoreData.pCurrentScreen->HkBeforeEraseBackground )
            {
                g_WGT_CoreData.pCurrentScreen->HkBeforeEraseBackground( g_WGT_CoreData.pCurrentScreen ) ;
            }

            WGT_Screen_OnEraseBackground( g_WGT_CoreData.pCurrentScreen ) ;

            if ( g_WGT_CoreData.pCurrentScreen->OnEraseBackground )
            {
                g_WGT_CoreData.pCurrentScreen->OnEraseBackground( g_WGT_CoreData.pCurrentScreen ) ;
            }

            // Call 'after' hook
            if ( g_WGT_CoreData.pCurrentScreen->HkAfterEraseBackground )
            {
                g_WGT_CoreData.pCurrentScreen->HkAfterEraseBackground( g_WGT_CoreData.pCurrentScreen ) ;
            }
        break ;

        case WGT_MSG_PAINT :
            // Call 'before' hook
            if ( g_WGT_CoreData.pCurrentScreen->HkBeforePaint )
            {
                g_WGT_CoreData.pCurrentScreen->HkBeforePaint( g_WGT_CoreData.pCurrentScreen ) ;
            }

            WGT_Screen_OnPaint( g_WGT_CoreData.pCurrentScreen ) ;

            if ( g_WGT_CoreData.pCurrentScreen->OnPaint )
            {
                g_WGT_CoreData.pCurrentScreen->OnPaint( g_WGT_CoreData.pCurrentScreen ) ;
            }

            // Call 'after' hook
            if ( g_WGT_CoreData.pCurrentScreen->HkAfterPaint )
            {
                g_WGT_CoreData.pCurrentScreen->HkAfterPaint( g_WGT_CoreData.pCurrentScreen ) ;
            }
        break ;
    }

    // Send message to screen main callbacks
    if ( g_WGT_CoreData.pCurrentScreen->ProcessMessage != NULL )
    {
        if ( g_WGT_CoreData.pCurrentScreen->ProcessMessage( g_WGT_CoreData.pCurrentScreen, pMsg ) == SAMGUI_E_OK )
        {
        }
    }

    return SAMGUI_E_OK ;
}

/** @}
 * @}
 * @} */
