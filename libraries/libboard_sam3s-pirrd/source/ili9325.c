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

/**
 * \file
 *
 * Implementation of ILI9325 driver.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"

#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Write data to LCD Register.
 *
 * \param reg   Register address.
 * \param data  Data to be written.
 */
void LCD_WriteReg( uint8_t reg, uint16_t data )
{
    LCD_IR() = 0;
    LCD_IR() = reg;
    LCD_D()  = (data >> 8) & 0xFF;
    LCD_D()  = data & 0xFF;
}

/**
 * \brief Read data from LCD Register.
 *
 * \param reg   Register address.
 *
 * \return      Readed data.
 */
uint16_t LCD_ReadReg( uint8_t reg )
{
    uint16_t value;

    LCD_IR() = 0;
    LCD_IR() = reg;

    value = LCD_D();
    value = (value << 8) | LCD_D();

    return value;
}

/**
 * \brief Read LCD status Register.
 *
 * \return      status data.
 */
extern uint16_t LCD_ReadStatus( void )
{
    return LCD_SR();
}

/**
 * \brief Prepare to write GRAM data.
 */
extern void LCD_WriteRAM_Prepare( void )
{
    LCD_IR() = 0 ;
    LCD_IR() = ILI9325_R22H ;
}

/**
 * \brief Write data to LCD GRAM.
 *
 * \param color  24-bits RGB color.
 */
extern void LCD_WriteRAM( uint32_t dwColor )
{
    LCD_D() = ((dwColor >> 16) & 0xFF);
    LCD_D() = ((dwColor >> 8) & 0xFF);
    LCD_D() = (dwColor  & 0xFF);
}

/**
 * \brief Write one byte to LCD GRAM.
 *
 * \param color  24-bits RGB color.
 */
extern void LCD_WriteRAMByte( uint8_t uc )
{
    LCD_D() = uc ;
}

/**
 * \brief Prepare to read GRAM data.
 */
void LCD_ReadRAM_Prepare( void )
{
    LCD_IR() = 0 ;
    LCD_IR() = ILI9325_R22H ;
}

/**
 * \brief Read data to LCD GRAM.
 *
 * \note Because pixel data LCD GRAM is 18-bits, so convertion to RGB 24-bits
 * will cause low color bit lose.
 *
 * \return color  24-bits RGB color.
 */
extern uint32_t LCD_ReadRAM( void )
{
    uint8_t value[2];
    uint32_t color;

    value[0] = LCD_D();       /* dummy read */
    value[1] = LCD_D();       /* dummy read */
    value[0] = LCD_D();       /* data upper byte */
    value[1] = LCD_D();       /* data lower byte */

    /* Convert RGB565 to RGB888 */
    /* For BGR format */
    color = ((value[0] & 0xF8)) |                                  /* R */
            ((value[0] & 0x07) << 13) | ((value[1] & 0xE0) << 5) | /* G */
            ((value[1] & 0x1F) << 19);                             /* B */

    return color;
}

/**
 * \brief Dump register data. For debug only.
 *
 * \param startAddr  Register start address.
 * \param endAddr    Register end address.
 */
void LCD_DumpReg( uint8_t startAddr, uint8_t endAddr )
{
    uint16_t tmp ;
    uint8_t addr ;

    for ( addr = startAddr ; addr <= endAddr ; addr++ )
    {
        tmp = LCD_ReadReg( addr ) ;
        printf( "LCD.r 0x%02x = 0x%04x\n\r", addr, tmp ) ;
    }
}

/**
 * \brief Initialize the LCD controller.
 */
extern uint32_t LCD_Initialize( void )
{
    uint16_t chipid ;

    /* Check ILI9325 chipid */
    chipid = LCD_ReadReg( ILI9325_R00H ) ;
    if ( chipid != ILI9325_DEVICE_CODE )
    {
        printf( "Read ILI9325 chip ID (0x%04x) error, skip initialization.\r\n", chipid ) ;
        return 1 ;
    }

    /* Turn off LCD */
    LCD_PowerDown() ;

    /* Start initial sequence */
    LCD_WriteReg(ILI9325_R10H, 0x0000); /* DSTB = LP = STB = 0 */
    LCD_WriteReg(ILI9325_R00H, 0x0001); /* start internal OSC */
    LCD_WriteReg(ILI9325_R01H, ILI9325_R01H_SS ) ; /* set SS and SM bit */
    LCD_WriteReg(ILI9325_R02H, 0x0700); /* set 1 line inversion */

    LCD_WriteReg(ILI9325_R04H, 0x0000); /* Resize register */
    LCD_WriteReg(ILI9325_R08H, 0x0207); /* set the back porch and front porch */
    LCD_WriteReg(ILI9325_R09H, 0x0000); /* set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(ILI9325_R0AH, 0x0000); /* FMARK function */
    LCD_WriteReg(ILI9325_R0CH, 0x0000); /* RGB interface setting */
    LCD_WriteReg(ILI9325_R0DH, 0x0000); /* Frame marker Position */
    LCD_WriteReg(ILI9325_R0FH, 0x0000); /* RGB interface polarity */

    /* Power on sequence */
    LCD_WriteReg(ILI9325_R10H, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(ILI9325_R11H, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(ILI9325_R12H, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(ILI9325_R13H, 0x0000); /* VDV[4:0] for VCOM amplitude */
    Wait( 200 ) ;                       /* Dis-charge capacitor power voltage */
    LCD_WriteReg(ILI9325_R10H, 0x1290); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(ILI9325_R11H, 0x0227); /* DC1[2:0], DC0[2:0], VC[2:0] */
    Wait( 50 ) ;
    LCD_WriteReg(ILI9325_R12H, 0x001B); /* Internal reference voltage= Vci; */
    Wait( 50 ) ;
    LCD_WriteReg(ILI9325_R13H, 0x1100); /* Set VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(ILI9325_R29H, 0x0019); /* Set VCM[5:0] for VCOMH */
    LCD_WriteReg(ILI9325_R2BH, 0x000D); /* Set Frame Rate */
    Wait( 50 ) ;

    /* Adjust the Gamma Curve */
    LCD_WriteReg(ILI9325_R30H, 0x0000);
    LCD_WriteReg(ILI9325_R31H, 0x0204);
    LCD_WriteReg(ILI9325_R32H, 0x0200);
    LCD_WriteReg(ILI9325_R35H, 0x0007);
    LCD_WriteReg(ILI9325_R36H, 0x1404);
    LCD_WriteReg(ILI9325_R37H, 0x0705);
    LCD_WriteReg(ILI9325_R38H, 0x0305);
    LCD_WriteReg(ILI9325_R39H, 0x0707);
    LCD_WriteReg(ILI9325_R3CH, 0x0701);
    LCD_WriteReg(ILI9325_R3DH, 0x000e);

    LCD_SetDisplayPortrait( 0 ) ;
    /* Vertical Scrolling */
    LCD_WriteReg( ILI9325_R61H, 0x0001 ) ;
    LCD_WriteReg( ILI9325_R6AH, 0x0000 ) ;

    /* Partial Display Control */
    LCD_WriteReg(ILI9325_R80H, 0x0000);
    LCD_WriteReg(ILI9325_R81H, 0x0000);
    LCD_WriteReg(ILI9325_R82H, 0x0000);
    LCD_WriteReg(ILI9325_R83H, 0x0000);
    LCD_WriteReg(ILI9325_R84H, 0x0000);
    LCD_WriteReg(ILI9325_R85H, 0x0000);

    /* Panel Control */
    LCD_WriteReg(ILI9325_R90H, 0x0010);
    LCD_WriteReg(ILI9325_R92H, 0x0600);
    LCD_WriteReg(ILI9325_R95H, 0x0110);

    LCD_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;
    LCD_SetCursor( 0, 0 ) ;


    return 0 ;
}

/**
 * \brief Turn on the LCD.
 */
extern void LCD_On( void )
{
    LCD_WriteReg( ILI9325_R07H, ILI9325_R07H_BASEE|ILI9325_R07H_GON|ILI9325_R07H_DTE|ILI9325_R07H_D1|ILI9325_R07H_D0 ) ;
}

/**
 * \brief Turn off the LCD.
 */
extern void LCD_Off( void )
{
    LCD_WriteReg( ILI9325_R07H, ILI9325_R07H_GON|ILI9325_R07H_DTE|ILI9325_R07H_D1|ILI9325_R07H_D0 ) ;
}

/**
 * \brief Power down the LCD.
 */
extern void LCD_PowerDown( void )
{
    LCD_WriteReg( ILI9325_R07H, 0x00 ) ;
}

/**
 * \brief Set cursor of LCD srceen.
 *
 * \param x  X-coordinate of upper-left corner on LCD.
 * \param y  Y-coordinate of upper-left corner on LCD.
 */
extern void LCD_SetCursor( uint16_t x, uint16_t y )
{
    LCD_WriteReg( ILI9325_R20H, x ) ; /* column */
    LCD_WriteReg( ILI9325_R21H, y ) ; /* row */
}

extern void LCD_SetWindow( uint32_t dwX, uint32_t dwY, uint32_t dwWidth, uint32_t dwHeight )
{
    /* Set Horizontal Address Start Position */
   LCD_WriteReg( ILI9325_R50H, (uint16_t)dwX ) ;

   /* Set Horizontal Address End Position */
   LCD_WriteReg( ILI9325_R51H, (uint16_t)dwX+dwWidth-1 ) ;

   /* Set Vertical Address Start Position */
   LCD_WriteReg( ILI9325_R52H, (uint16_t)dwY ) ;

   /* Set Vertical Address End Position */
   LCD_WriteReg( ILI9325_R53H, (uint16_t)dwY+dwHeight-1 ) ;
}

static void LCD_SetDisplayLandscape_kk( uint32_t dwRGB )
{
    uint16_t dwValue ;
//ILI9325_R03H_HWM|ILI9325_R03H_ORG|
    dwValue=ILI9325_R03H_AM|ILI9325_R03H_DFM|ILI9325_R03H_TRI|ILI9325_R03H_HWM|ILI9325_R03H_ORG ;

    if ( dwRGB == 0 )
    {
        dwValue|=ILI9325_R03H_BGR ;
    }
    LCD_WriteReg( ILI9325_R03H, dwValue ) ;
//    LCD_WriteReg( ILI9325_R60H, (0x1d<<8)|0x00 ) ;
    LCD_SetWindow( 0, 0, BOARD_LCD_HEIGHT, BOARD_LCD_WIDTH ) ;
}
extern void LCD_SetDisplayLandscape( uint32_t dwRGB )
{
    uint16_t dwValue ;
    dwValue=ILI9325_R03H_AM|ILI9325_R03H_DFM|ILI9325_R03H_TRI|ILI9325_R03H_ORG;

    if ( dwRGB == 0 )
    {
        dwValue|=ILI9325_R03H_BGR ;
    }
    LCD_WriteReg( ILI9325_R03H, dwValue ) ;
    LCD_WriteReg( ILI9325_R60H, (0x27<<8)|0x0 |ILI9325_R60H_GS) ;
    LCD_WriteReg( ILI9325_R61H, 0x5 ) ;
}
extern void LCD_SetDisplayPortrait( uint32_t dwRGB )
{
    uint16_t dwValue ;

    dwValue=ILI9325_R03H_TRI|ILI9325_R03H_DFM|ILI9325_R03H_ID1|ILI9325_R03H_ID0;

    if ( dwRGB == 0 )
    {
        dwValue|=ILI9325_R03H_BGR ;
    }
    LCD_WriteReg( ILI9325_R03H, dwValue ) ;
    LCD_WriteReg( ILI9325_R60H, ILI9325_R60H_GS|(0x27<<8)|0x00 ) ;
}

extern void LCD_TestPattern( uint32_t dwRGB )
{
    uint32_t dwLine ;
    uint32_t dw ;

    LCD_SetWindow( 10, 10, 100, 20 ) ;
    LCD_SetCursor( 10, 10 ) ;
    LCD_WriteRAM_Prepare() ;

    for ( dwLine=0 ; dwLine < 20 ; dwLine++ )
    {
        // Draw White bar
        for ( dw=0 ; dw < 20 ; dw++ )
        {
                LCD_D() = 0xff ;
                LCD_D() = 0xff ;
                LCD_D() = 0xff ;
        }
        // Draw Red bar
        for ( dw=0 ; dw < 20 ; dw++ )
        {
            if ( dwRGB == 0 )
            {
                LCD_D() = 0xff ;
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
            }
            else
            {
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
                LCD_D() = 0xff ;
            }
        }
        // Draw Green bar
        for ( dw=0 ; dw < 20 ; dw++ )
        {
                LCD_D() = 0x00 ;
                LCD_D() = 0xff ;
                LCD_D() = 0x00 ;
        }
        // Draw Blue bar
        for ( dw=0 ; dw < 20 ; dw++ )
        {
            if ( dwRGB == 0 )
            {
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
                LCD_D() = 0xff ;
            }
            else
            {
                LCD_D() = 0xff ;
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
            }
        }
        // Draw Black bar
        for ( dw=0 ; dw < 20 ; dw++ )
        {
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
                LCD_D() = 0x00 ;
        }
    }

    LCD_SetWindow( 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT ) ;
}

extern void LCD_VerticalScroll( uint16_t wY )
{
    LCD_WriteReg( ILI9325_R61H, 3 ) ;
    LCD_WriteReg( ILI9325_R6AH, wY ) ;
}

extern void LCD_SetPartialImage1( uint32_t dwDisplayPos, uint32_t dwStart, uint32_t dwEnd )
{
    assert( dwStart <= dwEnd ) ;

    LCD_WriteReg( ILI9325_R80H, dwDisplayPos&0x1ff ) ;
    LCD_WriteReg( ILI9325_R81H, dwStart&0x1ff ) ;
    LCD_WriteReg( ILI9325_R82H, dwEnd&0x1ff ) ;
}

extern void LCD_SetPartialImage2( uint32_t dwDisplayPos, uint32_t dwStart, uint32_t dwEnd )
{
    assert( dwStart <= dwEnd ) ;

    LCD_WriteReg( ILI9325_R83H, dwDisplayPos&0x1ff ) ;
    LCD_WriteReg( ILI9325_R84H, dwStart&0x1ff ) ;
    LCD_WriteReg( ILI9325_R85H, dwEnd&0x1ff ) ;
}

