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

#ifndef _OV_7740_
#define _OV_7740_

/** define a structure for omnivision register initialization values */
typedef struct _ov_reg
{
    /** Register to be written */
    uint8_t reg ;
    /** Value to be written in the register */
    uint8_t val ;
} ov_reg ;

typedef enum _EOV7740_Format
{
    QVGA_YUV422,
    QQVGA_YUV422,
    QVGA_RGB888,
    QQVGA_RGB888
} EOV7740_Format ;

/// Omnivision power control
#define OV_CTRL2_POWER_OFF      0
#define OV_CTRL2_POWER_ON       1

extern const ov_reg OV7740_QVGA_RGB888[] ;
extern const ov_reg OV7740_QQVGA_RGB888[] ;
extern const ov_reg OV7740_QVGA_YUV422[] ;
extern const ov_reg OV7740_QQVGA_YUV422[] ;

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

extern void ov_reset( const Pin *pPinReset ) ;
extern uint32_t ov_init( Twid *pTwid ) ;
extern uint32_t ov_configure( Twid *pTwid, const EOV7740_Format eFormat ) ;
extern uint32_t ov_configure_manual( Twid *pTwid );
uint32_t ov_configure_finish( Twid *pTwid );

extern uint32_t ov_read_reg( Twid *pTwid, uint8_t ucReg, uint8_t *pData ) ;
extern uint32_t ov_write_reg( Twid *pTwid, uint8_t ucReg, uint8_t ucVal ) ;

extern int ov_write_regs( Twid *pTwid, const ov_reg* pReglist ) ;
extern uint32_t ov_retrieve_manual( Twid *pTwid );

/** size of uint32_t,store this to continuous back up registers */
extern uint8_t ov_sotre_manual(Twid *pTwid, volatile uint32_t *bk_addr,uint8_t size);
/** size of uint32_t,restore previoius saved value */
extern uint8_t ov_resotre_manual(volatile uint32_t *bk_addr,uint8_t size);

extern void ov_pwd( const Pin *pCtrl2, uint32_t dwPower ) ;

extern void ov_DumpRegisters( Twid *pTwid, ov_reg* pRegs ) ;

#endif // _OV_7740_
