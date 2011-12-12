/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
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

//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------
#include "board.h"

/** Slave address of OMNIVISION chips. */
#define OV_CAPTOR_ADDRESS   (0x42>>1)   //OV7740 -> 0x42


/** terminating list entry for register in configuration file */
#define OV_REG_TERM 0xff
/** terminating list entry for value in configuration file */
#define OV_VAL_TERM 0xff

//-----------------------------------------------------------------------------
//         Local Functions
//-----------------------------------------------------------------------------

/**
 *  Reset using ctrl signal
 *
 *  \param pCtrl1 PIO interface
 */
extern void ov_reset( const Pin *pPinReset )
{
  volatile uint32_t i ;

  if ( pPinReset->id == 0 )
  {
      return ;
  }

  pPinReset->pio->PIO_CODR = pPinReset->mask ;
  Wait( 2 ) ;

  pPinReset->pio->PIO_SODR = pPinReset->mask ;
  Wait( 2 ) ;
}

/**
 *  Read PID and VER
 *
 *  \param pTwid TWI interface
 *
 *  \return VER | (PID<<8) == 0x7740
 */
static uint32_t _ov_id( Twid *pTwid )
{
    uint8_t id=0 ;
    uint8_t ver=0 ;

    // OV_PID
    ov_read_reg( pTwid, OV7740_PIDH, &id ) ;
    TRACE_INFO( "PID  = 0x%X\n\r", id ) ;

    // OV_VER
    ov_read_reg( pTwid, OV7740_PIDL, &ver ) ;
    TRACE_INFO( "VER  = 0x%X\n\r", ver ) ;

    return ((uint32_t)(id << 8) | ver) ;
}

/**
 *  Read Manufacturer
 *
 *  \param pTwid TWI interface
 *  \return 0 if error; 1 if the good captor is present
 */
static uint32_t _ov_manufacturer( Twid *pTwid )
{
    uint8_t midh=0 ;
    uint8_t midl=0 ;

    // OV_MIDH
    ov_read_reg( pTwid, OV7740_MIDH, &midh ) ;
    TRACE_DEBUG( "MIDH = 0x%X\n\r", midh ) ;

    // OV_MIDL
    ov_read_reg( pTwid, OV7740_MIDL, &midl ) ;
    TRACE_DEBUG( "MIDL = 0x%X\n\r", midl ) ;

    if ( ( midh == 0x7F) && (midl == 0xA2) )
    {
        return 1 ;
    }

    return 0 ;
}

/**
 *  ov_TestWrite
 *  \param pTwid TWI interface
 *  \return 1 if  the write is correct; 0 otherwise
 */
static uint32_t _ov_TestWrite( Twid *pTwid )
{
    uint8_t value=0 ;
    uint8_t oldvalue=0 ;

    // OV_BLUE_GAIN
    ov_read_reg( pTwid, 0x01, &oldvalue ) ;
    ov_write_reg( pTwid, 0x01, 0xAD ) ;
    ov_read_reg( pTwid, 0x01, &value ) ;

    if ( value != 0xAD )
    {
        return 0 ;
    }

    // return old value
    ov_write_reg( pTwid, 0x01, oldvalue ) ;
    ov_read_reg( pTwid, 0x01, &value ) ;

    if ( value != oldvalue )
    {
        return 0 ;
    }

    return 1 ;
}

//-----------------------------------------------------------------------------
//         Global Functions
//-----------------------------------------------------------------------------

/**
 *  Read a value from a register in an OV9650 sensor device.
 *  \param pTwid TWI interface
 *  \param reg Register to be read
 *  \param pData Data read
 *  \return 0 if no error, otherwize TWID_ERROR_BUSY
 */
extern uint32_t ov_read_reg( Twid *pTwid, uint8_t ucReg, uint8_t *pucVal )
{
    uint32_t dwStatus ;

 //   dwStatus = TWID_Write( pTwid, OV_CAPTOR_ADDRESS, 0, 0, &ucReg, 1, NULL ) ;
//    dwStatus |= TWID_Read( pTwid, OV_CAPTOR_ADDRESS, 0, 0, pucVal, 1, NULL ) ;

    dwStatus = TWID_Read( pTwid, OV_CAPTOR_ADDRESS, ucReg, 1, pucVal, 1, 0 ) ;

    if ( dwStatus != 0 )
    {
        TRACE_ERROR( "ov_read_reg pb" ) ;
    }

    return dwStatus ;
}

/**
 *  Write a value to a register in an OV9650 sensor device.
 *  \param pTwid TWI interface
 *  \param reg Register to be write
 *  \param val Value to be writte
 *  \return 0 if no error, otherwize TWID_ERROR_BUSY
 */
extern uint32_t ov_write_reg( Twid *pTwid, uint8_t ucReg, uint8_t ucVal )
{
    uint32_t dwStatus ;

    dwStatus = TWID_Write( pTwid, OV_CAPTOR_ADDRESS, ucReg, 1, &ucVal, 1, 0 ) ;

    if ( dwStatus != 0 )
    {
        TRACE_ERROR( "ov_write_reg pb" ) ;
    }

    return dwStatus ;
}

/**
 *  Initialize a list of OV registers.
 *  The list of registers is terminated by the pair of values
 *  { OV_REG_TERM, OV_VAL_TERM }.
 *  Returns zero if successful, or non-zero otherwise.
 *  \param pTwid TWI interface
 *  \param pReglist Register list to be written
 *  \return 0 if no error, otherwize TWID_ERROR_BUSY
 */
int ov_write_regs( Twid *pTwid, const ov_reg* pReglist )
{
    int err ;
    int size=0 ;
    const ov_reg *pNext = pReglist ;
    uint32_t i=0 ;

    TRACE_DEBUG( "ov_write_regs:" ) ;

    while ( !((pNext->reg == OV_REG_TERM) && (pNext->val == OV_VAL_TERM)) )
    {
        if(pNext->reg == 0xFE)
        {
            Wait(5);
        }
        else
        {
            err = ov_write_reg( pTwid, pNext->reg, pNext->val ) ;
            TRACE_DEBUG_WP( "+(%d) ", size ) ;
            size++ ;
            
            if ( err == TWID_ERROR_BUSY )
            {
                TRACE_ERROR( "ov_write_regs: TWI ERROR\n\r" ) ;
                
                return err ;
            }
        }
        pNext++ ;
    }

    TRACE_DEBUG_WP( "\n\r" ) ;

    return 0 ;
}

/**
 *  Dump all register
 *  \param pTwid TWI interface
 *  \param ovType Sensor type
 */
void ov_DumpRegisters( Twid *pTwid, ov_reg* pRegs )
{
    int i ;
    uint8_t value ;
    uint8_t regNum ;

    regNum = 0xd9 ;

    TRACE_INFO_WP( "Dump all camera register\n\r" ) ;

    for ( i = 0 ; i <= regNum ; i++ )
    {
        value = 0 ;
        ov_read_reg( pTwid, i, &value ) ;

        if ( pRegs != NULL )
        {
            pRegs[i].reg=i ;
            pRegs[i].val=value ;
        }

        TRACE_INFO_WP( "[0x%02x]=0x%02x ", i, value ) ;
        if ( ((i+1)%5) == 0 )
        {
            TRACE_INFO_WP( "\n\r" ) ;
        }
    }
    TRACE_INFO_WP( "\n\r" ) ;
}

/**
 *  Sequence For correct operation of the sensor
 *
 *  \param pTwid TWI interface
 *  \param ovType Sensor type
 *
 *  \return 1 if initialization ok, otherwise 0
 */
extern uint32_t ov_init( Twid *pTwid )
{
    uint32_t dwId=0 ;

    dwId = _ov_id( pTwid ) ;

    if ( (dwId>>8)  == 0x77 )
    {
        TRACE_DEBUG( "ID and PID OK\n\r" ) ;
        if ( _ov_manufacturer( pTwid ) == 1 )
        {
            TRACE_DEBUG( "Manufacturer OK\n\r" ) ;
            if ( _ov_TestWrite( pTwid ) == 1 )
            {
                return 1 ;
            }
            else
            {
                TRACE_ERROR( "Problem captor: bad write\n\r" ) ;
            }
        }
        else
        {
            TRACE_ERROR( "Problem captor: bad Manufacturer\n\r" ) ;
        }
    }
    else
    {
        TRACE_ERROR( "Problem captor: bad PID\n\r" ) ;
    }

    TRACE_INFO( "Problem: captor not responding\n\r" ) ;

    return 0 ;
}

/**
 *  Power control using ctrl2 signal
 *
 *  \param pCtrl2 PIO interface
 *  \param power on, off
 */
extern void ov_pwd( const Pin *pCtrl2, uint32_t dwPower )
{
  volatile uint32_t i ;

  if ( pCtrl2->id == 0 )
  {
      return ;
  }

  switch ( dwPower )
  {
      case OV_CTRL2_POWER_OFF :
          pCtrl2->pio->PIO_CODR = pCtrl2->mask ;
      break ;

      case OV_CTRL2_POWER_ON :
          pCtrl2->pio->PIO_SODR = pCtrl2->mask ;
      break ;
  }
}

/**
 * Configure the OV9650 for a specified image size, pixel format
 */
extern uint32_t ov_configure( Twid *pTwid, const EOV7740_Format eFormat )
{
    const ov_reg* pRegs_conf=NULL ;

    TRACE_DEBUG( "ov_configure\n" ) ;

    // common register initialization
    switch ( eFormat )
    {
        case QVGA_YUV422 :
            TRACE_DEBUG( "QVGA_YUV422\n" ) ;
            pRegs_conf = OV7740_QVGA_YUV422 ;
        break ;

        case QQVGA_YUV422 :
            TRACE_DEBUG( "QQVGA_YUV422\n" ) ;
            pRegs_conf = OV7740_QQVGA_YUV422 ;
        break ;

        case QVGA_RGB888 :
            TRACE_DEBUG( "QVGA_YUV422\n" ) ;
            pRegs_conf = OV7740_QVGA_RGB888 ;
        break ;

        case QQVGA_RGB888 :
            TRACE_DEBUG( "QVGA_YUV422\n" ) ;
            pRegs_conf = OV7740_QQVGA_RGB888 ;
        break ;

        default :
            TRACE_DEBUG( "ov_configure problem\n" ) ;
        break ;
    }

    if ( pRegs_conf != NULL )
    {
        ov_write_regs( pTwid, pRegs_conf ) ;
        return 1 ;
    }

    return 0 ;
}
static ov_reg regs_manual[]=
{
    {0x13,0x0},
    {0x0f,0x0},
    {0x10,0x0},
    {0x80,0x0},
    {0x01,0x0},
    {0x02,0x0},
    {0x03,0x0},
    {0xff,0xff},
    
};

extern uint32_t ov_configure_manual( Twid *pTwid )
{
    ov_write_regs(pTwid,regs_manual);
}
extern uint32_t ov_configure_finish( Twid *pTwid )
{
    ov_write_reg(pTwid,0xff,0xff);
}


static uint32_t ov_retrieve_manual( Twid *pTwid )
{
    int i = 0;
    for(;i < sizeof(regs_manual)/sizeof(ov_reg);i++)
    {
        ov_read_reg(pTwid,regs_manual[i].reg, &regs_manual[i].val );
        switch( regs_manual[i].reg)
        {
            case 0x13:
            /* no ae*/
            regs_manual[i].val = 0x0; 
            break;
//            case 0x0F:
//            /* no ae*/
//            regs_manual[i].val = 0x0f; 
            break;
            case 0x80:
            /* no ae*/
            regs_manual[i].val = 0x0F; 
            break;
            case 0xff:
            /* no ae*/
            regs_manual[i].val = 0xff; 
            break;
            
            
        }
        
    }
    
    
}
/** size of uint32_t,store this to continuous back up registers */
extern uint8_t ov_sotre_manual(Twid *pTwid, volatile uint32_t *bk_addr,uint8_t size)
{
    uint8_t offset = 0;
    
    if(sizeof(regs_manual)/sizeof(ov_reg) <= size * sizeof(uint32_t))
    {
        /* retrieve values from image sensor */
        ov_retrieve_manual(pTwid);
        while(offset < size)
        {
        
            * (bk_addr + offset ) = 0x0;
            for(int i = 0; i < sizeof(uint32_t) ;i++)
            {
                * (bk_addr + offset ) |= ((regs_manual[i + offset*sizeof(uint32_t)].val & 0xff)<< (i*8));
            }
            offset++;
        }
        return size;
    }
    else
    {
        return 0;
    }
    
}

/** size of uint32_t,restore previoius saved value */
extern uint8_t ov_resotre_manual(volatile uint32_t *bk_addr,uint8_t size)
{
   uint8_t offset = 0;
    
    if(sizeof(regs_manual)/sizeof(ov_reg) <= size * sizeof(uint32_t))
    {
        while(offset < size)
        {
        
            for(int i = 0; i < sizeof(uint32_t) ;i++)
            {
                regs_manual[i + offset*sizeof(uint32_t)].val = (((* (bk_addr + offset ))>>(i*8))&0xFF);
            }
            offset++;
        }
        return size;
    }
    else
    {
        return 0;
    }
}