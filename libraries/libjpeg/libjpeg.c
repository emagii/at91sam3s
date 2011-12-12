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

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "libjpeg.h"
#include "include/cdjpeg.h"		/* Common decls for cjpeg/djpeg applications */

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/**
 * \addtogroup libjpeg Independant JPEG Group library v0.08a
 * @{
 * This library allows JPEG images manipulation.
 * This is a port of IJG open source JPEG library.
 * The original package can be obtained here: http://www.ijg.org/
 *
 */
 
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief allocate memory for mcu row from heap
 * \param image pointer to mcu row
 * \param cinfo compress infomation especially components info
 */
static void _alloc_sarray_imcu_row(JSAMPIMAGE *image,j_compress_ptr cinfo)
{
    int ci;
    jpeg_component_info *compptr;
    assert( image != NULL ) ;
    *image = malloc( cinfo->num_components * sizeof(JSAMPARRAY));
    /* Allocate a strip buffer for each component */
    for (ci = 0, compptr = cinfo->comp_info; ci < cinfo->num_components;
	 ci++, compptr++) {
      (*image)[ci] = (*cinfo->mem->alloc_sarray)
	((j_common_ptr) cinfo, JPOOL_IMAGE,
	 compptr->width_in_blocks * compptr->DCT_h_scaled_size,
	 (JDIMENSION) (compptr->v_samp_factor * compptr->DCT_v_scaled_size));
    }
}

/**
  * \brief initial mcu rows with yuv420 data
  * \param image pointer to mcu row
  * \param src pointer of source with yuv422 format
  * \param y_pos the start position of the mcu rows in a whole image
  * \param width width of the image
  * \param rows total number of concerned rows in the mcu
  */
static void _init_sarray_imcu_row(JSAMPIMAGE *image,JSAMPLE *src,uint32_t y_pos,uint32_t width,uint32_t rows)
{
    JSAMPLE *r0;
    JSAMPLE *r1;
    JSAMPARRAY ay = (*image)[0];
    JSAMPARRAY au = (*image)[1];
    JSAMPARRAY av = (*image)[2];
    JSAMPROW ry0,ru,rv,ry1;    
    
    int i,j;
    for( i = 0; i < rows/2 ;i++)
    {
        r0 = & src[ (i * 2 + y_pos) * width * 2];
        r1 = & src[ (i * 2 + y_pos + 1) * width * 2];
        
        ry0 = ay[i*2];
        ry1 = ay[i*2+1];
        
        ru = au[i];
        rv = av[i];
        
        for(j = 0; j < width /2; j++)
        {
            ry0[j * 2] = r0[j*4];
            ry1[j * 2] = r1[j*4];
            ry0[j * 2 + 1] = r0[j*4+2];
            ry1[j * 2 +1] = r1[j*4+2];
            
            ru[j] = (r0[ j * 4 + 1] + r1[j*4+1])/2;
            rv[j] = (r0[ j * 4 + 3] + r1[j*4+3])/2;
            
        }
        
        
    }
}


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief do comperession from raw yuv420 data
 * \param pData pointer to jpeg information given by main application
 */
extern uint32_t ijg_compress_raw_no_padding( SJpegData* pData )
{
    struct jpeg_compress_struct cinfo ;
    struct jpeg_error_mgr       jerr ;
    
    JSAMPIMAGE  raw_image;          /* pointer to raw pointer */
    
    uint32_t lines;

    assert( pData != NULL ) ;

    cinfo.err = jpeg_std_error( &jerr ) ;
    jpeg_create_compress( &cinfo ) ;
    jpeg_mem_dest( &cinfo, &(pData->pucDst), (unsigned long*)&(pData->dwDstLength) ) ;

    cinfo.image_width      = pData->dwWidth ;
    cinfo.image_height     = pData->dwHeight ;
    cinfo.input_components = pData->dwBPP ;
    cinfo.in_color_space   = JCS_YCbCr ;

    jpeg_set_defaults( &cinfo ) ;
    /* set this for raw data*/
    cinfo.raw_data_in = TRUE;
    cinfo.do_fancy_downsampling = FALSE;
    /* color space is yuv*/
    jpeg_set_colorspace(&cinfo,JCS_YCbCr);
    /* factors for 3 components horizontally and vertically*/
    cinfo.comp_info[0].h_samp_factor = 2;
    cinfo.comp_info[0].v_samp_factor = 2;
    cinfo.comp_info[1].h_samp_factor = 1;
    cinfo.comp_info[1].v_samp_factor = 1;
    cinfo.comp_info[2].h_samp_factor = 1;
    cinfo.comp_info[2].v_samp_factor = 1;
    
    cinfo.dct_method = pData->eMethod ;
    jpeg_set_quality( &cinfo, pData->dwQuality, true ) ;
    jpeg_start_compress( &cinfo, true ) ;
    
    lines = cinfo.max_v_samp_factor * DCTSIZE;
    
    /* allocate memory for raw_image*/
    _alloc_sarray_imcu_row(&raw_image,&cinfo);

    while ( cinfo.next_scanline < cinfo.image_height )
    {  
        _init_sarray_imcu_row(&raw_image,pData->pucSrc,cinfo.next_scanline,cinfo.image_width,lines);
        jpeg_write_raw_data( &cinfo, raw_image, lines ) ;
        
    }
    
    /* free allocated memory*/
    free(raw_image);
    jpeg_finish_compress( &cinfo ) ;
    jpeg_destroy_compress(&cinfo);

    return 0 ;
}
/**
 * \brief ijg compress of recommened method with pre-processing
 *
 * \param pData pointer to jpeg information given by main application
 */
extern uint32_t ijg_compress( SJpegData* pData )
{
    struct jpeg_compress_struct cinfo ;
    struct jpeg_error_mgr       jerr ;
    JSAMPROW row_pointer ;          /* pointer to a single row */

    assert( pData != NULL ) ;

    cinfo.err = jpeg_std_error( &jerr ) ;
    jpeg_create_compress( &cinfo ) ;
    jpeg_mem_dest( &cinfo, &(pData->pucDst), (unsigned long*)&(pData->dwDstLength) ) ;

    cinfo.image_width      = pData->dwWidth ;
    cinfo.image_height     = pData->dwHeight ;
    cinfo.input_components = pData->dwBPP ;
    cinfo.in_color_space   = pData->eInput ;

    jpeg_set_defaults( &cinfo ) ;
    cinfo.dct_method = pData->eMethod ;
    jpeg_set_quality( &cinfo, pData->dwQuality, true ) ;
    jpeg_start_compress( &cinfo, true ) ;

    while ( cinfo.next_scanline < cinfo.image_height )
    {
        row_pointer = (JSAMPROW) &pData->pucSrc[cinfo.next_scanline*cinfo.image_width*cinfo.input_components] ;
        jpeg_write_scanlines( &cinfo, &row_pointer, 1 ) ;
    }

    jpeg_finish_compress( &cinfo ) ;
    jpeg_destroy_compress(&cinfo);

    return 0 ;
}

/** 
 * \brief Entry for decompression
 *
 * \param pData Pointer to jpeg information given by main application
 */
extern uint32_t ijg_decompress( SJpegData* pData )
{
//        Allocate and initialize a JPEG decompression object
//        Specify the source of the compressed data (eg, a file)
//        Call jpeg_read_header() to obtain image info
//        Set parameters for decompression
//        jpeg_start_decompress(...);
//        while (scan lines remain to be read)
//                jpeg_read_scanlines(...);
//        jpeg_finish_decompress(...);
//        Release the JPEG decompression object

    struct jpeg_decompress_struct cinfo ;
    struct jpeg_error_mgr jerr ;
    uint32_t dwSourceLength ;
    JSAMPROW aLines[2]={ pData->pucDst, pData->pucDst+pData->dwWidth*pData->dwBPP } ;

    assert( pData != NULL ) ;

    cinfo.err=jpeg_std_error( &jerr ) ;

    jpeg_create_decompress( &cinfo ) ;
    dwSourceLength=pData->dwHeight*pData->dwWidth*pData->dwBPP ;
    jpeg_mem_src( &cinfo, (uint8_t*)pData->pucSrc, dwSourceLength ) ;
    jpeg_read_header( &cinfo, TRUE ) ;
    cinfo.dct_method = pData->eMethod ;
    jpeg_start_decompress( &cinfo ) ;

//    dwLines=dwDstLength/(dwWidth*3) ;
    for ( ; cinfo.output_scanline < cinfo.image_height ; )
    {
        jpeg_read_scanlines( &cinfo, aLines, 1 ) ;
//        jpeg_read_scanlines( &cinfo, aLines, dwLines ) ;

        if ( pData->cbk )
        {
            if ( pData->cbk( aLines[0], pData->dwWidth*pData->dwBPP ) != 0 )
            {
                break ;
            }
        }

    }

    jpeg_finish_decompress( &cinfo ) ;
    jpeg_destroy_decompress( &cinfo ) ;

    return 0 ;
}

/**
 * \brief Initialization of jpeg information
 *
 * \param pData Pointer to jpeg information given by main application
 */
extern uint32_t JpegData_Init( SJpegData* pData )
{
    assert( pData != NULL ) ;

    memset( pData, 0, sizeof( SJpegData ) ) ;

    return 0 ;
}

/** 
 * \brief Set the source data of the jpeg compression
 *
 * \param pData pointer to jpeg information given by main application
 */
extern uint32_t JpegData_SetSource( SJpegData* pData, uint8_t* pucSrc, uint32_t dwSrcLength )
{
    assert( pData != NULL ) ;

    pData->pucSrc=pucSrc ;
    pData->dwSrcLength=dwSrcLength ;

    return 0 ;
}

/** 
 * \brief Set destination for jpeg data
 *
 * \param pData Pointer to jpeg information given by main application
 * \param pucDst Pointer on destination buffer
 * \param dwDstLength Destination buffer length
 */
extern uint32_t JpegData_SetDestination( SJpegData* pData, uint8_t* pucDst, uint32_t dwDstLength )
{
    assert( pData != NULL ) ;

    pData->pucDst=pucDst ;
    pData->dwDstLength=dwDstLength ;

    return 0 ;
}

/** 
 * \brief Set dimensions of image
 *
 * \param pData pointer to jpeg information given by main application
 * \param dwWidth Image width
 * \param dwHeight Image height
 * \param dwBPP Bits per pixel
 */
extern uint32_t JpegData_SetDimensions( SJpegData* pData, uint32_t dwWidth, uint32_t dwHeight, uint32_t dwBPP )
{
    assert( pData != NULL ) ;

    pData->dwWidth=dwWidth ;
    pData->dwHeight=dwHeight ;
    pData->dwBPP=dwBPP ;

    return 0 ;
}

/** 
 * \brief Set parameters like quality, compression method and input format
 *
 * \param pData pointer to jpeg information given by main application
 */
extern uint32_t JpegData_SetParameters( SJpegData* pData, uint32_t dwQuality, EJpegInput eInput, EJpegMethod eMethod )
{
    assert( pData != NULL ) ;

    pData->dwQuality=dwQuality ;
    pData->eInput=eInput ;
    pData->eMethod=eMethod ;

    return 0 ;
}

/** 
 * \brief Set callback function for using in main application
 * 
 * \param pData pointer to jpeg information given by main application
 */
extern uint32_t JpegData_SetCallback( SJpegData* pData, uint32_t (*cbk)( uint8_t*, uint32_t ) )
{
    assert( pData != NULL ) ;

    pData->cbk=cbk ;

    return 0 ;
}

/** @} */
