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

#ifndef _SAM_GUI_WGT_FRONTEND_
#define _SAM_GUI_WGT_FRONTEND_

#include <stdint.h>

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

typedef struct _SWGTFrontendData
{
    uint32_t dwId ;
} sWGTFrontendData ;

typedef struct _SWGTFrontend
{
    struct _SWGTFrontend *pNext ;
    sWGTFrontendData sData ;

    uint32_t (*Initialize)( void ) ;

    uint32_t (*IOCtl)( uint32_t dwCommand, uint32_t* pdwValue, uint32_t* pdwValueLength ) ;
} SWGTFrontend ;

#define WGT_FRONTEND_IOCTL_SET_DATA              0x01L

extern uint32_t WGT_Frontend_AddFrontend( SWGTFrontend* pFrontend ) ;

/** @}
 * @}
 * @}
 * @} */

#endif // _SAM_GUI_WGT_FRONTEND_
