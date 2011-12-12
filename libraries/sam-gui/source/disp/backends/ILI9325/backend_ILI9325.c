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
 * \section Purpose
 *   Implementation of ILI9325 driver.
 *
 * \section Usage
 * The ILI9325 LCD Controller can be accessed through the \ref SDISPBackend DISP Backend interface
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include "libsam_gui.h"

#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

#ifdef __ICCARM__          // IAR
#pragma pack(1)            // IAR
#define __attribute__(...) // IAR
#endif                     // IAR

// BMP (Windows) Header Format
struct _SBMPHeader
{
    uint16_t wType ; // signature, must be 4D42 hex
    uint32_t dwFileSize ; // size of BMP file in bytes (unreliable)
    uint16_t wReserved1 ; // reserved, must be zero
    uint16_t wReserved2 ; // reserved, must be zero
    uint32_t dwOffset ; // offset to start of image data in bytes
    uint32_t dwHeaderSize ; // size of BITMAPINFOHEADER structure, must be 40
    uint32_t dwWidth ; // image width in pixels
    uint32_t dwHeight ; // image height in pixels
    uint16_t wPlanes ; // number of planes in the image, must be 1
    uint16_t wBits ; // number of bits per pixel (1, 4, 8, 16, 24, 32)
    uint32_t dwCompression ; // compression type (0=none, 1=RLE-8, 2=RLE-4)
    uint32_t dwImageSize ; // size of image data in bytes (including padding)
    uint32_t dwHResolution ; // horizontal resolution in pixels per meter (unreliable)
    uint32_t dwVResolution ; // vertical resolution in pixels per meter (unreliable)
    uint32_t dwColours ; // number of colors in image, or zero
    uint32_t dwImportantcolours ; // number of important colors, or zero
} __attribute__ ((packed)); // GCC

#ifdef __ICCARM__          // IAR
#pragma pack()             // IAR
#endif                     // IAR

typedef struct _SBMPHeader SBMPHeader ;

typedef volatile uint8_t vByte ;

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

/** LCD index register address */
#define ILI9325_IR (*((vByte *)(BOARD_LCD_BASE)))
/** LCD status register address */
#define ILI9325_SR (*((vByte *)(BOARD_LCD_BASE)))
/** LCD data address */
#define ILI9325_D  (*((vByte *)((uint32_t)(BOARD_LCD_BASE) + BOARD_LCD_RS)))

#define ILI9325_WIDTH        240
#define ILI9325_HEIGTH       320

//#define ILI9325_BGR_MODE
#define ILI9325_RGB_MODE

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/* ILI9325 ID code */
#define ILI9325_DEVICE_CODE    0x9325

/* ILI9325 LCD Registers */
#define TS_INS_START_OSC			0x00 /* Driver Code Read - data read at this instruction should be 0x0789 */
#define TS_INS_DRIV_OUT_CTRL		0x01 /* Driver Output Control 1          */
#define TS_INS_DRIV_WAV_CTRL		0x02 /* LCD Driving Control              */
#define TS_INS_ENTRY_MOD			0x03 /* Entry Mode                       */
#define TS_INS_RESIZE_CTRL			0x04 /* Resize Control                   */

#define TS_INS_DISP_CTRL1			0x07 /* Display Control 1                */
#define TS_INS_DISP_CTRL2			0x08 /* Display Control 2                */
#define TS_INS_DISP_CTRL3			0x09 /* Display Control 3                */
#define TS_INS_DISP_CTRL4			0x0A /* Display Control 4                */

#define TS_INS_RGB_DISP_IF_CTRL1	0x0C /* RGB Display Interface Control 1  */
#define TS_INS_FRM_MARKER_POS		0x0D /* Frame Maker Position             */
#define TS_INS_RGB_DISP_IF_CTRL2	0x0F /* RGB Display Interface Control 2  */

#define TS_INS_POW_CTRL1			0x10 /* Power Control 1 */
#define TS_INS_POW_CTRL2			0x11 /* Power Control 2 */
#define TS_INS_POW_CTRL3			0x12 /* Power Control 3 */
#define TS_INS_POW_CTRL4			0x13 /* Power Control 4 */

#define TS_INS_GRAM_HOR_AD			0x20 /* Horizontal GRAM Address Set  */
#define TS_INS_GRAM_VER_AD			0x21 /* Vertical  GRAM Address Set   */
#define TS_INS_RW_GRAM				0x22 /* Write Data to GRAM           */
#define TS_INS_POW_CTRL7			0x29 /* Power Control 7              */
#define TS_INS_FRM_RATE_COL_CTRL	0x2B /* Frame Rate and Color Control */

#define TS_INS_GAMMA_CTRL1			0x30 /* Gamma Control 1  */
#define TS_INS_GAMMA_CTRL2			0x31 /* Gamma Control 2  */
#define TS_INS_GAMMA_CTRL3			0x32 /* Gamma Control 3  */
#define TS_INS_GAMMA_CTRL4			0x35 /* Gamma Control 4  */
#define TS_INS_GAMMA_CTRL5			0x36 /* Gamma Control 5  */
#define TS_INS_GAMMA_CTRL6			0x37 /* Gamma Control 6  */
#define TS_INS_GAMMA_CTRL7			0x38 /* Gamma Control 7  */
#define TS_INS_GAMMA_CTRL8			0x39 /* Gamma Control 8  */
#define TS_INS_GAMMA_CTRL9			0x3C /* Gamma Control 9  */
#define TS_INS_GAMMA_CTRL10			0x3D /* Gamma Control 10 */

#define TS_INS_HOR_START_AD			0x50 /* Horizontal Address Start Position */
#define TS_INS_HOR_END_AD			0x51 /* Horizontal Address End Position   */
#define TS_INS_VER_START_AD			0x52 /* Vertical Address Start Position   */
#define TS_INS_VER_END_AD			0x53 /* Vertical Address End Position     */

#define TS_INS_GATE_SCAN_CTRL1		0x60 /* Driver Output Control 2    */
#define TS_INS_GATE_SCAN_CTRL2		0x61 /* Base Image Display Control */
#define TS_INS_GATE_SCAN_CTRL3		0x6A /* Vertical Scroll Control    */

#define TS_INS_PART_IMG1_DISP_POS	0x80 /* Partial Image 1 Display Position  */
#define TS_INS_PART_IMG1_START_AD	0x81 /* Partial Image 1 Area (Start Line) */
#define TS_INS_PART_IMG1_END_AD		0x82 /* Partial Image 1 Area (End Line)   */
#define TS_INS_PART_IMG2_DISP_POS	0x83 /* Partial Image 2 Display Position  */
#define TS_INS_PART_IMG2_START_AD	0x84 /* Partial Image 2 Area (Start Line) */
#define TS_INS_PART_IMG2_END_AD		0x85 /* Partial Image 2 Area (End Line)   */

#define TS_INS_PANEL_IF_CTRL1		0x90 /* Panel Interface Control 1 */
#define TS_INS_PANEL_IF_CTRL2		0x92 /* Panel Interface Control 2 */
#define TS_INS_PANEL_IF_CTRL3		0x93
#define TS_INS_PANEL_IF_CTRL4		0x95 /* Panel Interface Control 4 */
#define TS_INS_PANEL_IF_CTRL5		0x97
#define TS_INS_PANEL_IF_CTRL6		0x98

#define TS_INS_PANEL_IF_OTP_PC      0xA1    /* OTP VCM Programming Control */
#define TS_INS_PANEL_IF_OTP_STATUS  0xA2    /* OTP VCM Status and Enable   */
#define TS_INS_PANEL_IF_OTP_KEY     0xA5    /* OTP Programming ID Key      */

//touch screen LCD configuration
#define TS_ORN_PORTRAIT

#ifdef TS_ORN_PORTRAIT
#    define TS_SIZE_X					240
#    define TS_SIZE_Y					320
#    define TS_VAL_ENTRY_MOD			0x0030
#    define TS_INS_GRAM_ADX				TS_INS_GRAM_HOR_AD
#    define TS_INS_GRAM_ADY				TS_INS_GRAM_VER_AD
#    define TS_INS_START_ADX   			TS_INS_HOR_START_AD
#    define TS_INS_END_ADX   			TS_INS_HOR_END_AD
#    define TS_INS_START_ADY   			TS_INS_VER_START_AD
#    define TS_INS_END_ADY   			TS_INS_VER_END_AD
#else
#    define TS_SIZE_X					320
#    define TS_SIZE_Y					240
#    define TS_VAL_ENTRY_MOD			0x0028
#    define TS_INS_GRAM_ADX				TS_INS_GRAM_VER_AD
#    define TS_INS_GRAM_ADY				TS_INS_GRAM_HOR_AD
#    define TS_INS_START_ADX   			TS_INS_VER_START_AD
#    define TS_INS_END_ADX   			TS_INS_VER_END_AD
#    define TS_INS_START_ADY   			TS_INS_HOR_START_AD
#    define TS_INS_END_ADY   			TS_INS_HOR_END_AD
#endif

/*----------------------------------------------------------------------------
 *        Statics
 *----------------------------------------------------------------------------*/

/** static buffer for file operations */
static uint8_t _DBE_ILI9325_aucFileData[1024*2] ;

/**
 * \brief Write data to LCD Register.
 *
 * \param ucReg   Register address.
 * \param wData  Data to be written.
 */
static void _DBE_ILI9325_WriteReg( uint8_t ucReg, uint16_t wData )
{
    ILI9325_IR = 0 ;
    ILI9325_IR = ucReg ;
    ILI9325_D  = (wData >> 8) & 0xFF ;
    ILI9325_D  = wData & 0xFF ;
}

/**
 * \brief Read data from LCD Register.
 *
 * \param ucReg   Register address.
 *
 * \return      Readed data.
 */
static uint16_t _DBE_ILI9325_ReadReg( uint8_t ucReg )
{
    uint16_t wValue ;

    ILI9325_IR = 0 ;
    ILI9325_IR = ucReg ;

    wValue = ILI9325_D ;
    wValue = (wValue << 8) | ILI9325_D ;

    return wValue ;
}

/**
 * \brief Read LCD status Register.
 *
 * \return status data.
 */
static uint16_t _DBE_ILI9325_ReadStatus( void )
{
    return ILI9325_SR ;
}

/**
 * \brief Prepare to access GRAM data.
 */
static inline void _DBE_ILI9325_RAMAccess_Prepare( void )
{
    ILI9325_IR=0 ;
    ILI9325_IR=TS_INS_RW_GRAM ;
}

/**
 * \brief Write data to LCD GRAM.
 *
 * \param dwColor  24-bits RGB color.
 */
static void _DBE_ILI9325_WriteRAM( uint32_t dwColor )
{
#ifdef ILI9325_BGR_MODE
    ILI9325_D = (dwColor & 0xff) ;
    ILI9325_D = ((dwColor >> 8) & 0xff) ;
    ILI9325_D = ((dwColor >> 16) & 0xff) ;
#endif // ILI9325_BGR_MODE

#ifdef ILI9325_RGB_MODE
    ILI9325_D = ((dwColor >> 16) & 0xff) ;
    ILI9325_D = ((dwColor >> 8) & 0xff) ;
    ILI9325_D = (dwColor & 0xff) ;
#endif // ILI9325_RGB_MODE
}

/**
 * \brief Read data to LCD GRAM.
 *
 * \note Because pixel data LCD GRAM is 18-bits, so conversion to RGB 24-bits
 * will cause low color bit lose.
 *
 * \return SAMGUI_E_OK.
 */
static uint32_t _DBE_ILI9325_ReadRAM( SGUIColor* pclrResult )
{
    uint8_t ucValue ;

    ucValue=ILI9325_D ; // dummy read
    ucValue=ILI9325_D ; // dummy read
    ucValue=ILI9325_D ; // data upper byte
    pclrResult->u.RGBA.ucR=ucValue & 0xf8 ;
    pclrResult->u.RGBA.ucG=(ucValue & 0x07) << 13 ;

    ucValue=ILI9325_D ; // data lower byte
    pclrResult->u.RGBA.ucG|=(ucValue & 0xe0) << 5 ;
    pclrResult->u.RGBA.ucB=(ucValue & 0x1f) << 19 ;

    return SAMGUI_E_OK ;
}

/**
 * \brief Set cursor of LCD srceen.
 *
 * \param wX  X-coordinate of cursor.
 * \param wY  Y-coordinate of cursor.
 */
static inline void _DBE_ILI9325_SetCursor( uint16_t wX, uint16_t wY )
{
    _DBE_ILI9325_WriteReg( TS_INS_GRAM_HOR_AD, wX ) ; // column
    _DBE_ILI9325_WriteReg( TS_INS_GRAM_VER_AD, wY ) ; // row
}

/**
 * \brief Turn on the LCD.
 */
static inline void _DBE_ILI9325_LCD_On( void )
{
    _DBE_ILI9325_WriteReg( TS_INS_DISP_CTRL1, 0x133 ) ;
}

/**
 * \brief Turn off the LCD.
 */
static inline void _DBE_ILI9325_LCD_Off( void )
{
    _DBE_ILI9325_WriteReg( TS_INS_DISP_CTRL1, 0x00 ) ;
}

static uint32_t _DBE_ILI9325_Reset( void )
{
    return SAMGUI_E_OK ;
}

/**
 * \brief Set the backlight of the LCD.
 *
 * \param dwLevel Backlight brightness level [1..16], 1 means maximum brightness.
 */
static void _DBE_ILI9325_SetBacklight( uint32_t dwLevel )
{
    uint32_t i ;
    const Pin pPins[]={ BOARD_BACKLIGHT_PIN } ;

//    printf( "_DBE_ILI9325_SetBacklight - %u\r\n", dwLevel ) ;

    // Ensure valid level
    if ( dwLevel < 1 )
    {
        dwLevel=1 ;
    }
    else
    {
        if ( dwLevel > 16 )
        {
            dwLevel=16 ;
        }
    }

    // Switch off backlight
    PIO_Clear( pPins ) ;
    SAMGUI_TaskDelay( 2 ) ;

    // Set new backlight level
    for ( i=17 ; i > dwLevel ; i-- )
    {
        PIO_Clear( pPins ) ;
        PIO_Clear( pPins ) ;
        PIO_Clear( pPins ) ;

        PIO_Set( pPins ) ;
        PIO_Set( pPins ) ;
        PIO_Set( pPins ) ;
    }
}

static void _DBE_ILI9325_SetOrientation( uint32_t dwOrientation )
{
//DISP_BACKEND_IOCTL_SET_MODE_PORTRAIT
//DISP_BACKEND_IOCTL_SET_MODE_LANDSCAPE
}

static inline void _DBE_ILI9325_SetBGRMode( void )
{
    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xc030 ) ; /* set GRAM write direction and BGR=1. */
}

static inline void _DBE_ILI9325_SetRGBMode( void )
{
    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xd030 ) ; /* set GRAM write direction and BGR=1. */
}

static void _DBE_ILI9325_SetWindow( uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight )
{
    /* Set Horizontal Address Start Position */
   _DBE_ILI9325_WriteReg( TS_INS_HOR_START_AD, (uint16_t)dwX ) ;

   /* Set Horizontal Address End Position */
   _DBE_ILI9325_WriteReg( TS_INS_HOR_END_AD, (uint16_t)dwX+dwWidth-1 ) ;

   /* Set Vertical Address Start Position */
   _DBE_ILI9325_WriteReg( TS_INS_VER_START_AD, (uint16_t)dwY ) ;

   /* Set Vertical Address End Position */
   _DBE_ILI9325_WriteReg( TS_INS_VER_END_AD, (uint16_t)dwY+dwHeight-1 ) ;
}

/**
 * \brief Initialize the LCD controller.
 */
static uint32_t _DBE_ILI9325_Initialize( void )
{
    const Pin pPins[]={ BOARD_LCD_PINS, BOARD_BACKLIGHT_PIN } ;
    Smc *pSmc=SMC ;
    uint16_t wChipID ;

    // Enable pins
    PIO_Configure( pPins, PIO_LISTSIZE( pPins ) ) ;

    // Enable peripheral clock
    PMC_EnablePeripheral( ID_SMC ) ;

    // EBI SMC Configuration
    pSmc->SMC_CS_NUMBER[1].SMC_SETUP = 0
                | ((2 <<  0) & SMC_SETUP1_NWE_SETUP)
                | ((2 <<  8) & SMC_SETUP1_NCS_WR_SETUP)
                | ((2 << 16) & SMC_SETUP1_NRD_SETUP)
                | ((2 << 24) & SMC_SETUP1_NCS_RD_SETUP)
                ;

    pSmc->SMC_CS_NUMBER[1].SMC_PULSE = 0
                | ((4  <<  0) & SMC_PULSE1_NWE_PULSE)
                | ((4  <<  8) & SMC_PULSE1_NCS_WR_PULSE)
                | ((10 << 16) & SMC_PULSE1_NRD_PULSE)
                | ((10 << 24) & SMC_PULSE1_NCS_RD_PULSE)
                ;

    pSmc->SMC_CS_NUMBER[1].SMC_CYCLE = 0
                | ((7 <<  0) & SMC_CYCLE1_NWE_CYCLE)
                | ((20 << 16) & SMC_CYCLE1_NRD_CYCLE)
//                | ((10 <<  0) & SMC_CYCLE1_NWE_CYCLE)
//                | ((22 << 16) & SMC_CYCLE1_NRD_CYCLE)
                ;

    pSmc->SMC_CS_NUMBER[1].SMC_MODE = 0
                | (SMC_MODE1_READ_MODE)
                | (SMC_MODE1_WRITE_MODE)
                | (0) /* Set 8 bit width. TODO: replace with definition in device header file */
                ;

    // Check ILI9325 chipid
//    wChipID=_DBE_ILI9325_ReadReg( TS_INS_START_OSC ) ;
//    if ( wChipID != ILI9325_DEVICE_CODE )
//    {
//        printf( "Read ILI9325 chip ID (%x) error, skip initialization.\r\n", wChipID ) ;
//        return SAMGUI_E_WRONG_COMPONENT ;
//    }

    // Turn off LCD
    _DBE_ILI9325_LCD_Off() ;

    // Start initial sequence
    _DBE_ILI9325_WriteReg( TS_INS_POW_CTRL1, 0x0000 ) ; /* DSTB = LP = STB = 0 */
    _DBE_ILI9325_WriteReg( TS_INS_START_OSC, 0x0001 ) ; /* start internal OSC */
    _DBE_ILI9325_WriteReg( TS_INS_DRIV_OUT_CTRL, 0x0100 ) ; /* set SS and SM bit */
    _DBE_ILI9325_WriteReg( TS_INS_DRIV_WAV_CTRL, 0x0700 ) ; /* set 1 line inversion */
#ifdef ILI9325_BGR_MODE
//    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xc030 ) ; /* set GRAM write direction and BGR=1. */
    _DBE_ILI9325_SetBGRMode() ;
#endif // ILI9325_BGR_MODE
#ifdef ILI9325_RGB_MODE
//    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xD030 ) ; /* set GRAM write direction and BGR=1. */
    _DBE_ILI9325_SetRGBMode() ;

#endif // ILI9325_RGB_MODE
    _DBE_ILI9325_WriteReg( TS_INS_RESIZE_CTRL, 0x0000 ) ; /* Resize register */
//    wChipID=_DBE_ILI9325_ReadReg( TS_INS_DISP_CTRL1 ) ;
//    printf( "TS_INS_DISP_CTRL1 %x\r\n", wChipID ) ;
//    _DBE_ILI9325_WriteReg( TS_INS_DISP_CTRL1, 0x0100 ) ;
    _DBE_ILI9325_WriteReg( TS_INS_DISP_CTRL2, 0x0207 ) ; /* set the back porch and front porch */
    _DBE_ILI9325_WriteReg( TS_INS_DISP_CTRL3, 0x0000 ) ; /* set non-display area refresh cycle ISC[3:0] */
    _DBE_ILI9325_WriteReg(TS_INS_DISP_CTRL4, 0x0000); /* FMARK function */
    _DBE_ILI9325_WriteReg(TS_INS_RGB_DISP_IF_CTRL1, 0x0000); /* RGB interface setting */
    _DBE_ILI9325_WriteReg(TS_INS_FRM_MARKER_POS, 0x0000); /* Frame marker Position */
    _DBE_ILI9325_WriteReg(TS_INS_RGB_DISP_IF_CTRL2, 0x0000); /* RGB interface polarity */

    /* Power on sequence */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL1, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL2, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL3, 0x0000); /* VREG1OUT voltage */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL4, 0x0000); /* VDV[4:0] for VCOM amplitude */

    SAMGUI_TaskDelay( 200 ) ;           /* Dis-charge capacitor power voltage */

    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL1, 0x1290); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL2, 0x0227); /* DC1[2:0], DC0[2:0], VC[2:0] */

    SAMGUI_TaskDelay( 50 ) ;

    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL3, 0x001B); /* Internal reference voltage= Vci; */

    SAMGUI_TaskDelay( 50 ) ;

    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL4, 0x1100); /* Set VDV[4:0] for VCOM amplitude */
    _DBE_ILI9325_WriteReg(TS_INS_POW_CTRL7, 0x0019); /* Set VCM[5:0] for VCOMH */
    _DBE_ILI9325_WriteReg(TS_INS_FRM_RATE_COL_CTRL, 0x000D); /* Set Frame Rate */

    SAMGUI_TaskDelay( 50 ) ;

    _DBE_ILI9325_SetCursor( 0, 0 ) ;

    // Adjust the Gamma Curve
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL1, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL2, 0x0204);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL3, 0x0200);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL4, 0x0007);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL5, 0x1404);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL6, 0x0705);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL7, 0x0305);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL8, 0x0707);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL9, 0x0701);
    _DBE_ILI9325_WriteReg(TS_INS_GAMMA_CTRL10, 0x000e);

    // Set GRAM area (240x320)
    _DBE_ILI9325_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;

    _DBE_ILI9325_WriteReg(TS_INS_GATE_SCAN_CTRL1, 0xa700);
    _DBE_ILI9325_WriteReg(TS_INS_GATE_SCAN_CTRL2, 0x0001);
    _DBE_ILI9325_WriteReg(TS_INS_GATE_SCAN_CTRL3, 0x0000);

    // Partial Display Control
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG1_DISP_POS, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG1_START_AD, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG1_END_AD, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG2_DISP_POS, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG2_START_AD, 0x0000);
    _DBE_ILI9325_WriteReg(TS_INS_PART_IMG2_END_AD, 0x0000);

    // Panel Control
    _DBE_ILI9325_WriteReg(TS_INS_PANEL_IF_CTRL1, 0x0010);
    _DBE_ILI9325_WriteReg(TS_INS_PANEL_IF_CTRL2, 0x0600);
    _DBE_ILI9325_WriteReg(TS_INS_PANEL_IF_CTRL4, 0x0110);

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_Color666( uint32_t dwColor888 )
{
    return ((dwColor888&0xfc)<<2)|((dwColor888&0xfc00)<<2)|((dwColor888&0xfc0000)<<2) ;
}

static uint32_t _DBE_ILI9325_GetPixel( uint32_t dwX, uint32_t dwY, SGUIColor* pclrResult )
{
    if ( pclrResult == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    _DBE_ILI9325_SetCursor( dwX, dwY ) ;
    _DBE_ILI9325_RAMAccess_Prepare() ;
//    pclrResult->u.dwRGBA=_DBE_ILI9325_ReadRAM() ;
    _DBE_ILI9325_ReadRAM( pclrResult ) ;

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawPixel( uint32_t dwX, uint32_t dwY, SGUIColor* pclrIn )
{
    if ( dwX >= BOARD_LCD_WIDTH )
    {
        dwX=BOARD_LCD_WIDTH-1 ;
    }

    if ( dwY >= BOARD_LCD_HEIGHT )
    {
        dwY=BOARD_LCD_HEIGHT-1 ;
    }

    _DBE_ILI9325_SetCursor( dwX, dwY ) ;
    _DBE_ILI9325_RAMAccess_Prepare() ;
#ifdef ILI9325_RGB_MODE
    ILI9325_D=((pclrIn->u.dwRGBA >> 16) & 0xff) ;
    ILI9325_D=((pclrIn->u.dwRGBA >> 8) & 0xff) ;
    ILI9325_D=(pclrIn->u.dwRGBA & 0xff) ;
#endif // ILI9325_RGB_MODE

#ifdef ILI9325_BGR_MODE
//    _DBE_ILI9325_WriteRAM( /*_DBE_ILI9325_Color666*/( pclrIn->u.dwRGBA ) ) ;
    ILI9325_D=(pclrIn->u.dwRGBA & 0xff) ;
    ILI9325_D=((pclrIn->u.dwRGBA >> 8) & 0xff) ;
    ILI9325_D=((pclrIn->u.dwRGBA >> 16) & 0xff) ;
#endif // ILI9325_BGR_MODE

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawLineBresenham( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, SGUIColor* pclrIn )
{
    int dx, dy ;
    int i ;
    int xinc, yinc, cumul ;
    int x, y ;

    x = dwX1 ;
    y = dwY1 ;
    dx = dwX2 - dwX1 ;
    dy = dwY2 - dwY1 ;

    xinc = ( dx > 0 ) ? 1 : -1 ;
    yinc = ( dy > 0 ) ? 1 : -1 ;
    dx = ( dx > 0 ) ? dx : -dx ;
    dy = ( dy > 0 ) ? dy : -dy ;

    _DBE_ILI9325_DrawPixel( x, y, pclrIn ) ;

    if ( dx > dy )
    {
      cumul = dx / 2 ;
      for ( i = 1 ; i <= dx ; i++ )
      {
        x += xinc ;
        cumul += dy ;

        if ( cumul >= dx )
        {
          cumul -= dx ;
          y += yinc ;
        }
        _DBE_ILI9325_DrawPixel( x, y, pclrIn ) ;
      }
    }
    else
    {
        cumul = dy / 2 ;
        for ( i = 1 ; i <= dy ; i++ )
        {
            y += yinc ;
            cumul += dx ;

            if ( cumul >= dy )
            {
                cumul -= dy ;
                x += xinc ;
            }

            _DBE_ILI9325_DrawPixel( x, y, pclrIn ) ;
        }
    }

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawLine( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, SGUIColor* pclrIn )
{
    uint32_t dw=0 ;

    if ( dwX1 >= BOARD_LCD_WIDTH )
    {
        dwX1=BOARD_LCD_WIDTH-1 ;
    }

    if ( dwX2 >= BOARD_LCD_WIDTH )
    {
        dwX2=BOARD_LCD_WIDTH-1 ;
    }

    if ( dwY1 >= BOARD_LCD_HEIGHT )
    {
        dwY1=BOARD_LCD_HEIGHT-1 ;
    }

    if ( dwY2 >= BOARD_LCD_HEIGHT )
    {
        dwY2=BOARD_LCD_HEIGHT-1 ;
    }

    _DBE_ILI9325_SetCursor( dwX1, dwY1 ) ;

    // Horizontal line
    if ( dwY1 == dwY2 )
    {
        _DBE_ILI9325_RAMAccess_Prepare() ;

        if ( dwX1 > dwX2 )
        {
            dw=dwX1 ;
            dwX1=dwX2 ;
            dwX2=dw ;
        }

        for ( dw=dwX1 ; dw <= dwX2 ; dw++ )
        {
            _DBE_ILI9325_WriteRAM( /*_DBE_ILI9325_Color666*/( pclrIn->u.dwRGBA ) ) ;
        }
    }
    else
    {
        // Vertical line
        if ( dwX1 == dwX2 )
        {
            if ( dwY1 > dwY2 )
            {
                dw=dwY1 ;
                dwY1=dwY2 ;
                dwY2=dw ;
            }

            for ( dw=dwY1 ; dw <= dwY2 ; dw++ )
            {
                _DBE_ILI9325_RAMAccess_Prepare() ;
                _DBE_ILI9325_WriteRAM( /*_DBE_ILI9325_Color666*/( pclrIn->u.dwRGBA ) ) ;
                _DBE_ILI9325_SetCursor( dwX1, dw ) ;
            }
        }
        else // Bresenham
        {
            return _DBE_ILI9325_DrawLineBresenham( dwX1, dwY1, dwX2, dwY2, pclrIn ) ;
        }
    }

    return SAMGUI_E_OK ;
}

#if 1
static uint32_t _DBE_ILI9325_DrawCircle( uint32_t dwX, uint32_t dwY, uint32_t dwRadius, SGUIColor* pclrBorder )
{
    signed int d ; // Decision Variable
    uint32_t dwCurX ; // Current X Value
    uint32_t dwCurY ; // Current Y Value

    if ( pclrBorder == NULL )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    if ( dwRadius < 2 )
    {
        return SAMGUI_E_BAD_PARAMETER ;
    }

    d = 3 - (dwRadius << 1) ;
    dwCurX = 0 ;
    dwCurY = dwRadius ;

    while ( dwCurX <= dwCurY )
    {
        _DBE_ILI9325_DrawPixel( dwX + dwCurX, dwY + dwCurY, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX + dwCurX, dwY - dwCurY, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX - dwCurX, dwY + dwCurY, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX - dwCurX, dwY - dwCurY, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX + dwCurY, dwY + dwCurX, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX + dwCurY, dwY - dwCurX, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX - dwCurY, dwY + dwCurX, pclrBorder ) ;
        _DBE_ILI9325_DrawPixel( dwX - dwCurY, dwY - dwCurX, pclrBorder ) ;

        if ( d < 0 )
        {
            d += (dwCurX << 2) + 6 ;
        }
        else
        {
            d += ((dwCurX - dwCurY) << 2) + 10;
            dwCurY-- ;
        }

        dwCurX++ ;
    }

    return SAMGUI_E_OK ;
}
#else
static void _DBE_ILI9325_Plot4points( uint32_t dwX, uint32_t dwY, int iX, int iY, SGUIColor* pclrBorder )
{
  _DBE_ILI9325_DrawPixel( dwX+iX, dwY+iY, pclrBorder ) ;
  if ( iX != 0 ) _DBE_ILI9325_DrawPixel( dwX-iX, dwY+iY, pclrBorder ) ;
  if ( iY != 0 ) _DBE_ILI9325_DrawPixel( dwX+iX, dwY-iY, pclrBorder ) ;
  if ( iX != 0 && iY != 0 ) _DBE_ILI9325_DrawPixel( dwX-iX, dwY-iY, pclrBorder ) ;
}

static void _DBE_ILI9325_Plot8points( uint32_t dwX, uint32_t dwY, int x, int iY, SGUIColor* pclrBorder )
{
  _DBE_ILI9325_Plot4points( dwX, dwY, iX, iY, pclrBorder ) ;
  if ( iX != iY ) _DBE_ILI9325_Plot4points( dwX, dwY, iY, iX, pclrBorder ) ;
}

static uint32_t _DBE_ILI9325_DrawCircle( uint32_t dwX, uint32_t dwY, uint32_t dwRadius, SGUIColor* pclrBorder )
{
  int iError=-dwRadius ;
  int iX=dwRadius ;
  int iY=0 ;

  while ( iX >= iY )
  {
    plot8points( dwX, dwY, iX, iY, pclrBorder ) ;

    iError+=iY ;
    ++iY ;
    iError+=iY ;

    if ( iError >= 0 )
    {
      --iX ;
      iError-=iX ;
      iError-=iX ;
    }
  }
}
#endif

static uint32_t _DBE_ILI9325_DrawFilledCircle( uint32_t dwX, uint32_t dwY, uint32_t dwRadius, SGUIColor* pclrBorder, SGUIColor* pclrInside )
{
    signed int d ; // Decision Variable
    uint32_t dwCurX ; // Current X Value
    uint32_t dwCurY ; // Current Y Value

//    if ( pclrBorder == NULL )
//    {
//        return SAMGUI_E_BAD_POINTER ;
//    }

    if ( dwRadius < 2 )
    {
        return SAMGUI_E_BAD_PARAMETER ;
    }

    d = 3 - (dwRadius << 1) ;
    dwCurX = 0 ;
    dwCurY = dwRadius ;

    while ( dwCurX <= dwCurY )
    {
        _DBE_ILI9325_DrawLine( dwX-dwCurX, dwY-dwCurY, dwX+dwCurX, dwY-dwCurY, pclrInside ) ;
        _DBE_ILI9325_DrawLine( dwX-dwCurX, dwY+dwCurY, dwX+dwCurX, dwY+dwCurY, pclrInside ) ;
        _DBE_ILI9325_DrawLine( dwX-dwCurY, dwY-dwCurX, dwX+dwCurY, dwY-dwCurX, pclrInside ) ;
        _DBE_ILI9325_DrawLine( dwX-dwCurY, dwY+dwCurX, dwX+dwCurY, dwY+dwCurX, pclrInside ) ;

        if ( d < 0 )
        {
            d += (dwCurX << 2) + 6 ;
        }
        else
        {
            d += ((dwCurX - dwCurY) << 2) + 10;
            dwCurY-- ;
        }

        dwCurX++ ;
    }

    if ( pclrBorder != NULL )
    {
        _DBE_ILI9325_DrawCircle( dwX, dwY, dwRadius, pclrBorder ) ;
    }

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, SGUIColor* pclrFrame )
{
    uint32_t dw ;

    // Check coordonates
    if ( dwX1 > dwX2 )
    {
        dw=dwX1 ;
        dwX1=dwX2 ;
        dwX2=dw ;
    }

    if ( dwY1 > dwY2 )
    {
        dw=dwY1 ;
        dwY1=dwY2 ;
        dwY2=dw ;
    }

    _DBE_ILI9325_DrawLine( dwX1, dwY1, dwX2, dwY1, pclrFrame ) ;
    _DBE_ILI9325_DrawLine( dwX1, dwY2, dwX2, dwY2, pclrFrame ) ;

    _DBE_ILI9325_DrawLine( dwX1, dwY1, dwX1, dwY2, pclrFrame ) ;
    _DBE_ILI9325_DrawLine( dwX2, dwY1, dwX2, dwY2, pclrFrame ) ;

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawFilledRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, SGUIColor* pclrFrame, SGUIColor* pclrInside )
{
    uint32_t dw ;
    uint32_t dwLength ;

    // Check coordonates
    if ( dwX1 > dwX2 )
    {
        dw=dwX1 ;
        dwX1=dwX2 ;
        dwX2=dw ;
    }

    if ( dwY1 > dwY2 )
    {
        dw=dwY1 ;
        dwY1=dwY2 ;
        dwY2=dw ;
    }

    _DBE_ILI9325_SetCursor( dwX1, dwY1 ) ;
    _DBE_ILI9325_SetWindow( dwX1, dwY1, dwX2-dwX1+1, dwY2-dwY1+1 ) ;
    _DBE_ILI9325_RAMAccess_Prepare() ;

    dwLength=(dwX2-dwX1+1)*(dwY2-dwY1+1) ;
    for ( dw=0 ; dw < dwLength ; dw++ )
    {
//            _DBE_ILI9325_WriteRAM( pclrInside->u.dwRGBA ) ;
#ifdef ILI9325_BGR_MODE
        ILI9325_D=(pclrInside->u.dwRGBA & 0xff) ;
        ILI9325_D=((pclrInside->u.dwRGBA >> 8) & 0xff) ;
        ILI9325_D=((pclrInside->u.dwRGBA >> 16) & 0xff) ;
#endif // ILI9325_BGR_MODE

#ifdef ILI9325_RGB_MODE
        ILI9325_D=((pclrInside->u.dwRGBA >> 16) & 0xff) ;
        ILI9325_D=((pclrInside->u.dwRGBA >> 8) & 0xff) ;
        ILI9325_D=(pclrInside->u.dwRGBA & 0xff) ;
#endif // ILI9325_RGB_MODE
    }

    _DBE_ILI9325_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawBitmapBMP( uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight, uint8_t* pucData )
{
    uint32_t dwRow ;
    uint32_t dwCol ;
    uint32_t dwMinWidth ;
    uint32_t dwMinHeight ;
    SBMPHeader* pHeader ;
#ifdef ILI9325_RGB_MODE
    uint8_t ucR ;
    uint8_t ucG ;
    uint8_t ucB ;
#endif // ILI9325_RGB_MODE
    uint8_t* pucImage ;
    uint8_t* pucLine ;
    uint32_t dwScanLineBits ;
    uint32_t dwScanLineBytes ;
    SGUIColor clr={ .u.dwRGBA=0xff0000 } ;

    // Read header information
    pHeader=(SBMPHeader*)pucData ;

//    printf( "_DBE_ILI9325_DrawBitmapBMP\n\r" ) ;

    dwScanLineBits=pHeader->dwWidth*pHeader->wBits ;
    dwScanLineBytes=((dwScanLineBits+31)/32)*4 ;

    // Check that parameters match
    if ( (pHeader->dwCompression != 0) || (pHeader->dwWidth != dwWidth) || (pHeader->dwHeight != dwHeight) )
    {
        printf( "BMP_Decode: File format not supported\n\r" ) ;
        printf( " -> .compression = %u\n\r", pHeader->dwCompression ) ;
        printf( " -> .width = %u %u\n\r", pHeader->dwWidth, dwWidth ) ;
        printf( " -> .height = %u %u\n\r", pHeader->dwHeight, dwHeight ) ;
        printf( " -> .bits = %u\n\r", pHeader->wBits ) ;
        printf( " -> .scanline_bits = %u\n\r", dwScanLineBits ) ;
        printf( " -> .scanline_bytes = %u\n\r", dwScanLineBytes ) ;

        _DBE_ILI9325_DrawFilledRectangle( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, NULL, &clr ) ;
        clr.u.dwRGBA=0x000000 ;
        _DBE_ILI9325_DrawLine( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, &clr ) ;
        _DBE_ILI9325_DrawLine( dwX, dwY+dwHeight-1, dwX+dwWidth-1, dwY, &clr ) ;

        return 2 ;
    }

    // Get image data
    pucImage=pucData+pHeader->dwOffset ;
    dwMinWidth=(dwWidth+dwX < ILI9325_WIDTH)?dwWidth:ILI9325_WIDTH ;
    dwMinHeight=(dwHeight+dwY < ILI9325_HEIGTH)?dwHeight:ILI9325_HEIGTH ;

    switch ( pHeader->wBits )
    {
        case 24:
//            _DBE_ILI9325_WriteReg(ILI9325_R03H, _DBE_ILI9325_ReadReg( ILI9325_R03H )~(1<<12) ); /* set GRAM write direction and BGR=1. */
            _DBE_ILI9325_SetCursor( dwX, dwY ) ;
            _DBE_ILI9325_SetWindow( dwX, dwY, dwWidth, dwHeight ) ;
            _DBE_ILI9325_RAMAccess_Prepare() ;

            // Send image data to ILI9325 (swapping red & blue)
            for ( dwRow=0 ; dwRow < dwMinHeight ; dwRow++ )
            {
                pucLine=pucImage+((dwHeight-dwRow-1)*dwScanLineBytes) ;
//                printf( "_DBE_ILI9325_DrawBitmapBMP - Line %u/%u\n\r", dwRow, ((dwHeight-dwRow-1)*(dwWidth*3)) ) ;

                for ( dwCol=0 ; dwCol < dwMinWidth ; dwCol++ )
                {
    #ifdef ILI9325_BGR_MODE
                    ILI9325_D=(*pucLine/*<<2*/)++ ;
                    ILI9325_D=(*pucLine/*<<2*/)++ ;
                    ILI9325_D=(*pucLine/*<<2*/)++ ;
    #endif // ILI9325_BGR_MODE

    #ifdef ILI9325_RGB_MODE
                    ucB=*pucLine++ ;
                    ucG=*pucLine++ ;
                    ucR=*pucLine++ ;
                    ILI9325_D=ucR/*<<2*/ ;
                    ILI9325_D=ucG/*<<2*/ ;
                    ILI9325_D=ucB/*<<2*/ ;
    #endif // ILI9325_RGB_MODE
                }
            }

            _DBE_ILI9325_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;

//            _DBE_ILI9325_WriteReg(ILI9325_R03H, _DBE_ILI9325_ReadReg( ILI9325_R03H )|(1<<12) ); /* set GRAM write direction and BGR=1. */
        break ;

        case 8:
//            // Retrieve palette
//            struct BMPPaletteEntry palette[256];
//            memcpy(palette,
//                   (unsigned char *) ((unsigned int) file + sizeof(struct BMPHeader)),
//                   header->offset - sizeof(struct BMPHeader));
//
            // Decode image (reversing row order)
            // Send image data to ILI9325 (swapping red & blue)
            for ( dwRow=0 ; dwRow < dwMinHeight ; dwRow++ )
            {
                _DBE_ILI9325_SetCursor( dwX, dwY+dwRow ) ;
                _DBE_ILI9325_RAMAccess_Prepare() ;
                pucLine=pucImage+((dwHeight-dwRow-1)*dwWidth) ;
//                printf( "_DBE_ILI9325_DrawBitmapBMP - Line %u/%u/%u\n\r", dwRow, dwMinWidth, ((dwHeight-dwRow-1)*dwWidth) ) ;

                for ( dwCol=0 ; dwCol < dwMinWidth ; dwCol++ )
                {
                    ILI9325_D=(*pucLine++)/*<<2*/ ;
                    ILI9325_D=(*pucLine++)/*<<2*/ ;
                    ILI9325_D=(*pucLine++)/*<<2*/ ;
                }
            }
        break ;

        default:
            printf( "BMP_Decode: Input resolution not supported\n\r" ) ;
            printf( "header->bits 0x%X \n\r", pHeader->wBits ) ;

            _DBE_ILI9325_DrawFilledRectangle( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, NULL, &clr ) ;
            clr.u.dwRGBA=0x000000 ;
            _DBE_ILI9325_DrawLine( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, &clr ) ;
            _DBE_ILI9325_DrawLine( dwX, dwY+dwHeight-1, dwX+dwWidth-1, dwY, &clr ) ;

            return 4 ;
//        break ;
    }

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawBitmapBMPFile( uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight, uint8_t* pucData )
{
    uint32_t dwMin ;
    FIL fp ;
    SBMPHeader sHeader ;
    uint32_t dwLineLength ;
    uint32_t dwLength ;
    uint32_t dwTemp ;
    uint32_t dwIndex ;
    uint32_t dw ;
    uint8_t* pucLineData ;

    // Read first 256 bytes to obtain header
    if ( f_open( &fp, (const char*)pucData, FA_OPEN_EXISTING|FA_READ ) == FR_OK )
    {
        if ( f_read( &fp, &sHeader, sizeof( sHeader ), &dwLength ) == FR_OK )
        {
//            printf( "read %c%c\r\n", (char)(sHeader.wType&0xff), (char)(sHeader.wType>>8) ) ;
//            printf( " -> .dwCompression = %u\n\r", sHeader.dwCompression ) ;
//            printf( " -> .dwWidth = %u\n\r", sHeader.dwWidth ) ;
//            printf( " -> .dwHeight = %u\n\r", sHeader.dwHeight ) ;
//            printf( " -> .wBits = %u\n\r", sHeader.wBits ) ;
//            printf( " -> .dwOffset %u\r\n", sHeader.dwOffset ) ;
//            printf( " -> .dwColours %u\r\n", sHeader.dwColours ) ;
//            printf( " -> .dw %u\r\n", sHeader.dwColours*4+54 ) ;

            dwLineLength=(((sHeader.dwWidth*sHeader.wBits)+31)/32)*4 ;

            switch ( sHeader.wBits )
            {
                case 24:
                    // if header is Windows BMP 24bpp, alloc line size bytes on HEAP
                    // Move file pointer to begin of BMP data
//                    f_lseek( &fp, sHeader.dwOffset ) ;

//                    dwX2=dwX ;
//                    dwY+=sHeader.height ;

                    /* set GRAM write direction and BGR=1. */
                    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xc010 ) ;
                    /* Set window */
                    _DBE_ILI9325_SetWindow( dwX, dwY, dwWidth, dwHeight ) ;
//                _DBE_ILI9325_SetCursor( dwX, dwY ) ; //+dwHeight-1 ) ;
                    _DBE_ILI9325_SetCursor( dwX, dwY+dwHeight-1 ) ;


    ////                dwX2=_DBE_ILI9325_ReadReg( ILI9325_R03H ) ;
    //////                dwX2|=(1<<5) ; // I/D1=1
    ////                dwX2&=~(1<<3) ; // I/D1=0
    //////                dwX2&=~(1<<3) ; // AM=0
    //////                dwX2|=(1<<3) ; // AM=1
    ////                _DBE_ILI9325_WriteReg( ILI9325_R03H, (uint16_t)dwX2 ) ;

                    ILI9325_IR=0 ;
                    ILI9325_IR=TS_INS_RW_GRAM ;

                    // Read buffer and write it on backend
                    dwIndex=0 ;
                    do
                    {
                        if ( f_read( &fp, _DBE_ILI9325_aucFileData, sizeof( _DBE_ILI9325_aucFileData ), &dwLength ) == FR_OK )
                        {
//                            printf( "Read %u bytes\r\n", dwLength ) ;
                            // Get Min value from read length and buffer length
//                            dwMin=(dwLength < sizeof( _DBE_ILI9325_aucFileData ) )?dwLength:sizeof( _DBE_ILI9325_aucFileData ) ;

//                            printf( "writing %u bytes\r\n", dwMin ) ;
                            // Draw whole buffer data to backend
                            dwMin=dwWidth*3 ;
                            if ( dwLineLength != dwMin )
                            {
                                for ( dwTemp=0 ; dwTemp < dwLength ; dwTemp++, dwIndex++ )
                                {
                                    if ( (dwIndex >= dwMin) && (dwIndex < dwLineLength) )
                                    {
                                    }
                                    else
                                    {
                                        if ( dwIndex == dwLineLength )
                                        {
                                            dwIndex=0 ;
                                        }
                                        ILI9325_D=_DBE_ILI9325_aucFileData[dwTemp] ;
                                    }
                                }
                            }
                            else
                            {
                                for ( dwTemp=0 ; dwTemp < dwLength ; dwTemp++ )
                                {
                                    ILI9325_D=_DBE_ILI9325_aucFileData[dwTemp] ;
                                }
                            }
                        }
                    } while ( dwLength == sizeof( _DBE_ILI9325_aucFileData ) ) ;

#if defined ILI9325_RGB_MODE
                    _DBE_ILI9325_SetRGBMode() ;
#endif // defined ILI9325_RGB_MODE
#if defined ILI9325_BGR_MODE
                    _DBE_ILI9325_SetBGRMode() ;
#endif // defined ILI9325_BGR_MODE
                    // Restore whole window
                    _DBE_ILI9325_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;
                break ;

                case 8:
                    // Move file pointer to begin of BMP data
//                    f_lseek( &fp, 54 ) ;

                    // Get LUT
                    if ( f_read( &fp, _DBE_ILI9325_aucFileData, sHeader.dwColours<<2, &dwLength ) == FR_OK )
                    {
//                        printf( "Read %u bytes\r\n", dwLength ) ;
                    }

//                    f_lseek( &fp, sHeader.dwOffset ) ;

                    /* set GRAM write direction and BGR=1. */
                    _DBE_ILI9325_WriteReg( TS_INS_ENTRY_MOD, 0xc010 ) ;
                    /* Set window */
                    _DBE_ILI9325_SetWindow( dwX, dwY, dwWidth, dwHeight ) ;
                    /* Set cursor */
                    _DBE_ILI9325_SetCursor( dwX, dwY+dwHeight-1 ) ;
                    ILI9325_IR=0 ;
                    ILI9325_IR=TS_INS_RW_GRAM ;

                    // Read buffer and write it on backend
                    dwIndex=0 ;
                    pucLineData=_DBE_ILI9325_aucFileData+1024/*(sHeader.dwColours<<2)*/ ;
                    do
                    {
                        if ( f_read( &fp, pucLineData, sizeof( _DBE_ILI9325_aucFileData )-1024/*sHeader.dwColours<<2*/, &dwLength ) == FR_OK )
                        {
                            // Draw whole buffer data to backend
                            dwMin=dwWidth/**3*/ ;
                            if ( dwLineLength != dwMin )
                            {
                                for ( dwTemp=0 ; dwTemp < dwLength ; dwTemp++, dwIndex++ )
                                {
                                    if ( (dwIndex >= dwMin) && (dwIndex < dwLineLength) )
                                    {
                                    }
                                    else
                                    {
                                        if ( dwIndex == dwLineLength )
                                        {
                                            dwIndex=0 ;
                                        }
                                        /* Get value */
                                        dw=pucLineData[dwTemp]<<2 ;
                                        /* Draw pixel */
                                        ILI9325_D=_DBE_ILI9325_aucFileData[dw] ;
                                        ILI9325_D=_DBE_ILI9325_aucFileData[dw+1] ;
                                        ILI9325_D=_DBE_ILI9325_aucFileData[dw+2] ;
                                    }
                                }
                            }
                            else
                            {
                                for ( dwTemp=0 ; dwTemp < dwLength ; dwTemp++ )
                                {
                                    /* Get value */
                                    dw=pucLineData[dwTemp]<<2 ;
                                    /* Draw pixel */
                                    ILI9325_D=_DBE_ILI9325_aucFileData[dw] ;
                                    ILI9325_D=_DBE_ILI9325_aucFileData[dw+1] ;
                                    ILI9325_D=_DBE_ILI9325_aucFileData[dw+2] ;
                                }
                            }
                        }
                    } while ( dwLength == sizeof( _DBE_ILI9325_aucFileData )-1024/*(sHeader.dwColours<<2)*/ ) ;

#if defined ILI9325_RGB_MODE
                    _DBE_ILI9325_SetRGBMode() ;
#endif // defined ILI9325_RGB_MODE
#if defined ILI9325_BGR_MODE
                    _DBE_ILI9325_SetBGRMode() ;
#endif // defined ILI9325_BGR_MODE
                    // Restore whole window
                    _DBE_ILI9325_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;
                break ;

                default:
                    printf( "failed to read\r\n" ) ;
                break ;
            }
        }
        else // failed read
        {
            printf( "failed to read\r\n" ) ;
        }

        f_close( &fp ) ;

        return SAMGUI_E_OK ;
    }
    else
    {
        SGUIColor clr={ .u.dwRGBA=0xff0000 } ;

        printf( "failed to open %s\r\n", (const char*)pucData ) ;

        _DBE_ILI9325_DrawFilledRectangle( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, NULL, &clr ) ;
        clr.u.dwRGBA=0x000000 ;

        _DBE_ILI9325_DrawLine( dwX, dwY, dwX+dwWidth-1, dwY+dwHeight-1, &clr ) ;
        _DBE_ILI9325_DrawLine( dwX, dwY+dwHeight-1, dwX+dwWidth-1, dwY, &clr ) ;
    }

    return SAMGUI_E_FILE_OPEN ;
}

static uint32_t _DBE_ILI9325_DrawBitmap( uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight, uint8_t* pucData )
{
    uint32_t dwCol ;
//#ifdef ILI9325_BGR_MODE
//    uint8_t ucB ;
//    uint8_t ucG ;
//    uint8_t ucR ;
//#endif // ILI9325_BGR_MODE

    // Check if bitmap is Microsoft BMP
    if ( (pucData[0] == 'B') && (pucData[1] == 'M') )
    {
        return _DBE_ILI9325_DrawBitmapBMP( dwX, dwY, dwWidth, dwHeight, pucData ) ;
    }

    if ( pucData[0] == '/' )
    {
        return _DBE_ILI9325_DrawBitmapBMPFile( dwX, dwY, dwWidth, dwHeight, pucData ) ;
    }

    // Draw raw RGB bitmap
    _DBE_ILI9325_SetWindow( dwX, dwY, dwWidth, dwHeight ) ;
    _DBE_ILI9325_SetCursor( dwX, dwY ) ;

    ILI9325_IR=0 ;
    ILI9325_IR=TS_INS_RW_GRAM ;

#ifdef ILI9325_BGR_MODE
//    _DBE_ILI9325_SetBGRMode() ;
#endif // ILI9325_RGB_MODE

#ifdef ILI9325_RGB_MODE
//    _DBE_ILI9325_SetBGRMode() ;
#endif // ILI9325_RGB_MODE

    for ( dwCol=0 ; dwCol < dwWidth*dwHeight ; dwCol++ )
    {
#ifdef ILI9325_BGR_MODE
        ucB=((*pucData++)/*<<2*/) ;
        ucG=((*pucData++)/*<<2*/) ;
        ucR=((*pucData++)/*<<2*/) ;
        ILI9325_D=ucR/*<<2*/ ;
        ILI9325_D=ucG/*<<2*/ ;
        ILI9325_D=ucB/*<<2*/ ;
#endif // ILI9325_BGR_MODE

#ifdef ILI9325_RGB_MODE
        ILI9325_D = ((*pucData++)/*<<2*/) ;
        ILI9325_D = ((*pucData++)/*<<2*/) ;
        ILI9325_D = ((*pucData++)/*<<2*/) ;
#endif // ILI9325_RGB_MODE
    }

#ifdef ILI9325_BGR_MODE
//    _DBE_ILI9325_SetBGRMode() ;
#endif // ILI9325_RGB_MODE

#ifdef ILI9325_RGB_MODE
//    _DBE_ILI9325_SetRGBMode() ;
#endif // ILI9325_RGB_MODE

    _DBE_ILI9325_SetWindow( dwX, dwY, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawChar( uint32_t dwX, uint32_t dwY, uint8_t ucChar, SGUIColor* pclrText, SGUIFont* pFont, uint32_t dwSize )
{
    uint32_t dwRow ;
    uint32_t dwCol ;

//    assert( (ucChar >= 0x20) && (ucChar <= 0x7F) ) ;

    for ( dwCol=0 ; dwCol < 10 ; dwCol++ )
    {
        for ( dwRow=0 ; dwRow < 8 ; dwRow++ )
        {
            if ( (aucFont10x14[((ucChar - 0x20) * 20) + dwCol * 2] >> (7 - dwRow)) & 0x1)
            {
                _DBE_ILI9325_DrawPixel( dwX+dwCol, dwY+dwRow, pclrText ) ;
            }
        }

        for ( dwRow=0 ; dwRow < 6 ; dwRow++ )
        {
            if ( (aucFont10x14[((ucChar - 0x20) * 20) + dwCol * 2 + 1] >> (7 - dwRow)) & 0x1)
            {
                _DBE_ILI9325_DrawPixel( dwX+dwCol, dwY+dwRow+8, pclrText ) ;
            }
        }
    }

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_DrawText( uint32_t dwX, uint32_t dwY, char* pszText, SGUIColor* pclrText, SGUIFont* pFont, uint32_t dwSize )
{
    uint32_t dwXOrg=dwX ;

    if ( (pszText == NULL) || (pclrText == NULL) || (pFont == NULL) )
    {
        return SAMGUI_E_BAD_POINTER ;
    }

    while ( *pszText != 0 )
    {
        if ( *pszText == '\n' )
        {
            dwY+=pFont->dwHeight+2 ;
            dwX=dwXOrg ;
        }
        else
        {
            _DBE_ILI9325_DrawChar( dwX, dwY, *pszText, pclrText, pFont, dwSize ) ;
            dwX+=pFont->dwWidth+2 ;
        }
        pszText++ ;
    }

    return SAMGUI_E_OK ;
}

static uint32_t _DBE_ILI9325_IOCtl( uint32_t dwCommand, uint32_t* pdwValue, uint32_t* pdwValueLength )
{
    // Check pointers
//    if ( (pdwValue == NULL) || (pdwValueLength == NULL) )
//    {
//        return SAMGUI_E_BAD_POINTER ;
//    }

    switch ( dwCommand )
    {
        case DISP_BACKEND_IOCTL_POWER_ON :
            _DBE_ILI9325_LCD_On() ;
        break ;

        case DISP_BACKEND_IOCTL_POWER_OFF :
            _DBE_ILI9325_LCD_Off() ;
        break ;

        case DISP_BACKEND_IOCTL_SET_BACKLIGHT :
            _DBE_ILI9325_SetBacklight( (uint32_t)pdwValue ) ;
        break ;

        case DISP_BACKEND_IOCTL_SET_MODE_PORTRAIT :
            _DBE_ILI9325_SetOrientation( DISP_BACKEND_IOCTL_SET_MODE_PORTRAIT ) ;
        break ;

        case DISP_BACKEND_IOCTL_SET_MODE_LANDSCAPE :
            _DBE_ILI9325_SetOrientation( DISP_BACKEND_IOCTL_SET_MODE_LANDSCAPE ) ;
        break ;

        default :
            printf( "_DBE_ILI9325_IOCtl - Bad IOCtl index (%x)\r\n", dwCommand ) ;
        break ;
    }

    return SAMGUI_E_OK ;
}

/*----------------------------------------------------------------------------
 *        Exported interface
 *----------------------------------------------------------------------------*/
SDISPBackend sDISP_Backend_ILI9325=
{
    .sData=
    {
        .dwID=DISP_BACKEND_ILI9325,
    },

    .Reset=_DBE_ILI9325_Reset,
    .Initialize=_DBE_ILI9325_Initialize,
    .GetPixel=_DBE_ILI9325_GetPixel,
    .DrawPixel=_DBE_ILI9325_DrawPixel,
    .DrawLine=_DBE_ILI9325_DrawLine,
    .DrawCircle=_DBE_ILI9325_DrawCircle,
    .DrawFilledCircle=_DBE_ILI9325_DrawFilledCircle,
    .DrawRectangle=_DBE_ILI9325_DrawRectangle,
    .DrawFilledRectangle=_DBE_ILI9325_DrawFilledRectangle,
    .DrawBitmap=_DBE_ILI9325_DrawBitmap,
    .DrawText=_DBE_ILI9325_DrawText,
    .Fill=NULL,
    .IOCtl=_DBE_ILI9325_IOCtl
} ;
