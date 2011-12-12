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

#include "board.h"

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
 */

#define WGT_CORE_MSG_QUEUE_SIZE    32

NO_INIT SWGTCoreData g_WGT_CoreData ;
NO_INIT SAMGUI_TaskHandle hGUITask ;

/**
 * Core SAM-GUI task handling Message Queue and Message dispatching.
 */
static void _WGT_TaskMessageLoop( void* pParameter )
{
    SWGTCoreData* pData=(SWGTCoreData*)pParameter ;
    SWGTCoreMessage xMessage ;

    WGT_Start() ;

    for ( ; ; )
    {
        // Wait for a widget message

//        if ( SAMGUI_QueueReceive( pData->hMessagesQueue, &xMessage, portMAX_DELAY ) != SAMGUI_E_OK )
        if ( SAMGUI_QueueReceive( pData->hMessagesQueue, &xMessage, g_WGT_CoreData.dwTimerDelay ) == SAMGUI_E_OK )
        {
            if ( g_WGT_CoreData.pCurrentScreen != NULL )
            {
                PreProcessMessage_Default( g_WGT_CoreData.pCurrentScreen, &xMessage ) ;
            }
        }
//        else // delay, send timer
//        {
//            if ( g_WGT_CoreData.pCurrentScreen != NULL )
//            {
//                xMessage.dwID=WGT_MSG_TIMER ;
//                if ( g_WGT_CoreData.pCurrentScreen->ProcessMessage( g_WGT_CoreData.pCurrentScreen, &xMessage ) == SAMGUI_E_OK )
//                {
//                }
//            }
//        }
    }
}

/**
 * Initialize SAM-GUI core data and task.
 */
extern uint32_t WGT_Initialize( void )
{
    uint32_t dwError ;

    /* Set Timer Delay to 1s. */
    g_WGT_CoreData.dwTimerDelay=1000 ;

	/* Create the queue used by the Messages task. */
	g_WGT_CoreData.hMessagesQueue=SAMGUI_QueueCreate( WGT_CORE_MSG_QUEUE_SIZE, sizeof( SWGTCoreMessage ) ) ;

	/* Create the Core Messages task. */
	if ( SAMGUI_TaskCreate( _WGT_TaskMessageLoop, "WGT_ML", NULL, 512, 1, &g_WGT_CoreData, &hGUITask ) != SAMGUI_E_OK )
    {
        printf( "Failed to create WGT_ML task\r\n" ) ;
        return SAMGUI_E_OS_TASK_CREATE_FAILED ;
    }

    /* Get DISP backend. */
    g_WGT_CoreData.pBE=DISP_GetBackend() ;
    g_WGT_CoreData.pCurrentScreen=NULL ;

    if ( g_WGT_CoreData.pBE == NULL )
    {
        printf( "Invalid backend\n" ) ;

        return SAMGUI_E_BAD_POINTER ;
    }

    return SAMGUI_E_OK ;
}

/**
 * Initialize backends and frontends, once done, SAM-GUI is ready to work.
 */
extern uint32_t WGT_Start( void )
{
    SGUIColor clrWhite={ .u.dwRGBA=COLOR_WHITE } ;

    /* Initialize backend. */
    g_WGT_CoreData.pBE->IOCtl( DISP_BACKEND_IOCTL_POWER_OFF, NULL, NULL ) ;
    g_WGT_CoreData.pBE->Initialize() ;

//    g_WGT_CoreData.pBE->IOCtl( DISP_BACKEND_IOCTL_SET_BACKLIGHT, (uint32_t*)1, NULL ) ;

	g_WGT_CoreData.pBE->DrawFilledRectangle( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT, NULL, &clrWhite ) ;
    g_WGT_CoreData.pBE->IOCtl( DISP_BACKEND_IOCTL_POWER_ON, NULL, NULL ) ;

    return SAMGUI_E_OK ;
}

/**
 * Allow to set the delay between WM_TIMER messages.
 */
extern uint32_t WGT_SetTimerPeriod( uint32_t dwDelay )
{
    uint32_t dw ;

    dw=g_WGT_CoreData.dwTimerDelay ;
    g_WGT_CoreData.dwTimerDelay=dwDelay ;

    return dw ;
}

/**
 * Return the current delay between WM_TIMER messages.
 */
extern uint32_t WGT_GetTimerPeriod( void )
{
    return g_WGT_CoreData.dwTimerDelay ;
}

/**
 * Allow to set the current screen.
 */
extern uint32_t WGT_SetCurrentScreen( SWGTScreen* pScreen )
{
    SWGTCoreMessage xMessage ;

    if ( pScreen == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    if ( g_WGT_CoreData.pCurrentScreen != NULL )
    {
        if ( g_WGT_CoreData.pCurrentScreen->HkExit )
        {
            g_WGT_CoreData.pCurrentScreen->HkExit( g_WGT_CoreData.pCurrentScreen ) ;
        }
    }

    /* Destroy all data needed by this screen */
    if ( g_WGT_CoreData.pCurrentScreen != NULL )
    {
        WGT_Screen_Exit( g_WGT_CoreData.pCurrentScreen ) ;
    }

    /* Flush Message Queue. */
    while ( SAMGUI_QueueReceive( g_WGT_CoreData.hMessagesQueue, &xMessage, 0 ) == SAMGUI_E_OK )
    {
    }

    g_WGT_CoreData.pCurrentScreen=pScreen ;

    WGT_PostMessage( WGT_MSG_INIT, 0, 0 ) ;
    WGT_PostMessage( WGT_MSG_ERASE_BKGND, 0, 0 ) ;
    WGT_PostMessage( WGT_MSG_PAINT, 0, 0 ) ;

    return SAMGUI_E_OK ;
}

/** @}
 * @}
 * @} */
