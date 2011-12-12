/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
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

#ifndef _LIB_JPEG_
#define _LIB_JPEG_


/**
 * \addtogroup libjpeg Independant JPEG Group library v0.08a
 *
 * This library allows JPEG images manipulation.
 * @{
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "include/jpeglib.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
typedef enum _EJpegInput
{
  /** error/unspecified */
	JPG_DATA_UNKNOWN,

  /** monochrome */
	JPG_DATA_GRAYSCALE,

  /** red/green/blue */
	JPG_DATA_RGB,

  /** Y/Cb/Cr (also known as YUV) */
	JPG_DATA_YCbCr,

  /** C/M/Y/K */
	JPG_DATA_CMYK,

  /** Y/Cb/Cr/K */
	JPG_DATA_YCCK
} EJpegInput ;

typedef enum _EJpegMethod
{
  /** slow but accurate integer algorithm */
	JPG_METHOD_ISLOW,
  
  /** faster, less accurate integer method */
	JPG_METHOD_IFAST,
  
  /** floating-point: accurate, fast on fast HW */
	JPG_METHOD_FLOAT
} EJpegMethod ;

typedef struct _SJPEGData
{
    // Source
    uint8_t* pucSrc;
    uint32_t dwSrcLength ;

    // Destination
    uint8_t* pucDst ;
    uint32_t dwDstLength ;

    // Dimensions
    uint32_t dwHeight ;
    uint32_t dwWidth ;
    uint32_t dwBPP ;

    // JPEG algo parameters
    uint32_t dwQuality ;
    EJpegInput eInput ;
    EJpegMethod eMethod ;

    uint32_t (*cbk)( uint8_t*, uint32_t ) ;
} SJpegData ;

/*------------------------------------------------------------------------------
 *         Exported functions
 *------------------------------------------------------------------------------*/

extern uint32_t JpegData_Init( SJpegData* pData ) ;
extern uint32_t JpegData_SetSource( SJpegData* pData, uint8_t* pucSrc, uint32_t dwSrcLength ) ;


extern uint32_t JpegData_SetDestination( SJpegData* pData, uint8_t* pucDst, uint32_t dwDstLength ) ;
extern uint32_t JpegData_SetDimensions( SJpegData* pData, uint32_t dwWidth, uint32_t dwHeight, uint32_t dwBPP ) ;
extern uint32_t JpegData_SetParameters( SJpegData* pData, uint32_t dwQuality, EJpegInput eInput, EJpegMethod eMethod ) ;
extern uint32_t JpegData_SetCallback( SJpegData* pData, uint32_t (*cbk)( uint8_t*, uint32_t ) ) ;

extern uint32_t ijg_compress( SJpegData* pData ) ;

extern uint32_t ijg_compress_raw_no_padding( SJpegData* pData );
extern uint32_t ijg_decompress( SJpegData* pData ) ;

/** @} */

#endif // _LIB_JPEG_
