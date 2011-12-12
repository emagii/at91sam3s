/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation

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

/** \addtogroup sdmmc_api
 *  @{
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/


#include "memories.h"

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern void MCI_Handler(Mcid*pMci);

/*----------------------------------------------------------------------------
 *         Global variables
 *----------------------------------------------------------------------------*/
/** Bit mask for status register errors. */
#define STATUS_ERRORS ((uint32_t)(HSMCI_SR_UNRE  \
                       | HSMCI_SR_OVRE \
                       | HSMCI_SR_ACKRCVE \
                       /*| HSMCI_SR_BLKOVRE*/ \
                       | HSMCI_SR_CSTOE \
                       | HSMCI_SR_DTOE \
                       | HSMCI_SR_DCRCE \
                       | HSMCI_SR_RTOE \
                       | HSMCI_SR_RENDE \
                       | HSMCI_SR_RCRCE \
                       | HSMCI_SR_RDIRE \
                       | HSMCI_SR_RINDE))

/**
 * Processes pending events on the given MCI driver.
 * \param pMci  Pointer to a MCI driver instance.
 */
void MCI_Handler(Mcid*pMci)
{
    Hsmci *pMciHw = pMci->pMciHw;
    MciCmd *pCommand = pMci->pCommand;
    volatile uint32_t SR;
    volatile uint32_t maskedSR;
    volatile uint32_t mask;
    uint8_t i;
    uint8_t resSize;

    assert(pMci);
    assert(pMciHw);
    assert(pCommand);

    /* Read the status register */
    SR = pMciHw->HSMCI_SR;
    mask = pMciHw->HSMCI_IMR;
    maskedSR = SR & mask;

//    TRACE_INFO_WP("i%dS %x\n\r", (pCommand->cmd & HSMCI_CMDR_CMDNB), SR);
//    TRACE_INFO_WP("i%dM %x\n\r", (pCommand->cmd & HSMCI_CMDR_CMDNB), maskedSR);

    /* Check if an error has occured */
    if ((maskedSR & STATUS_ERRORS) != 0)
    {
        pCommand->state = SDMMC_CMD_ERROR;
        /* Check error code */
        if ((maskedSR & STATUS_ERRORS) == HSMCI_SR_RTOE)
        {
            pCommand->status = SDMMC_ERROR_NORESPONSE;
        }
        else
        {
//            TRACE_INFO( "iERR %x: C%x(%d), R%d\n\r", maskedSR, pCommand->cmd, (pCommand->cmd & HSMCI_CMDR_CMDNB), pCommand->resType ) ;

            pCommand->status = SDMMC_ERROR;
        }
    }
    mask &= ~STATUS_ERRORS;

    /* Check if a command has been completed */
    if (maskedSR & HSMCI_SR_CMDRDY)
    {
        /* Command OK, disable interrupt */
        pMciHw->HSMCI_IDR = HSMCI_IDR_CMDRDY;

        /* Check BUSY */
        if (pCommand->busyCheck
           && (pCommand->tranType == MCI_NO_TRANSFER
              || (pCommand->tranType == MCI_STOP_TRANSFER)) )
        {
            /* Check XFRDONE to confirm no bus action */
            if (SR & HSMCI_SR_NOTBUSY)
            {
                mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
            }
            else
            {
                pMciHw->HSMCI_IER = HSMCI_SR_NOTBUSY;
            }
        }
        else
        {
            mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
        }
    }

    /* Check if transfer stopped */
    if (maskedSR & HSMCI_SR_XFRDONE) {
        mask &= ~(uint32_t)HSMCI_IMR_XFRDONE;
    }

    /* Check if not busy */
    if (maskedSR & HSMCI_SR_NOTBUSY)
    {
        mask &= ~(uint32_t)HSMCI_IMR_NOTBUSY;
    }

    /* Check if data read done */
    if (maskedSR & HSMCI_SR_RXBUFF)
    {
        mask &= ~(uint32_t)HSMCI_IMR_RXBUFF;
    }

    /* Check if TX ready */
    if (maskedSR & HSMCI_SR_TXRDY)
    {
        mask &= ~(uint32_t)HSMCI_IMR_TXRDY;
    }

    /* Check if FIFO empty (TX & All data sent) */
    if (maskedSR & HSMCI_SR_FIFOEMPTY)
    {
        /* Disable FIFO EMPTY */
        pMciHw->HSMCI_IDR = HSMCI_SR_FIFOEMPTY;
        /* Byte transfer no BLKE */
        if (pMciHw->HSMCI_MR & HSMCI_MR_FBYTE)
        {
            mask &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
        }
        /* End command until no data on bus */
        else
        {
            if ((SR & HSMCI_SR_BLKE) == 0)
            {
                pMciHw->HSMCI_IER = HSMCI_IER_BLKE;
            }
            else
            {
                mask &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
            }
        }
    }

    /* Check if BLKE (TX & no data on bus) */
    if (maskedSR & HSMCI_SR_BLKE)
    {
        if (SR & HSMCI_SR_TXRDY )
        {
            mask &= ~(uint32_t)HSMCI_SR_BLKE;
        }
        else
        {
            pMciHw->HSMCI_IDR = HSMCI_SR_BLKE;
            pMciHw->HSMCI_IER = HSMCI_SR_TXRDY;
        }
    }

    /* Check if data write finished */
    if (maskedSR & HSMCI_SR_TXBUFE)
    {
        /* Data OK, disable interrupt */
        pMciHw->HSMCI_IDR = HSMCI_IDR_TXBUFE;

        /* End command until no data on bus */
        if ((SR & HSMCI_SR_FIFOEMPTY) == 0)
        {
            pMciHw->HSMCI_IER = HSMCI_IER_FIFOEMPTY;
        }
        else
        {
            mask &= ~(uint32_t)HSMCI_SR_TXBUFE;
        }
    }

    /* All non-error mask done, complete the command */
    if (0 == mask || pCommand->state != SDMMC_CMD_PENDING)
    {
        /* Store the card response in the provided buffer */
        if (pCommand->pResp)
        {
            switch (pCommand->resType)
            {
                case 1:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                    resSize = 1;
                break;

                case 2:
                    resSize = 4;
                break;

                default:
                    resSize = 0;
                break;
            }

            for (i=0; i < resSize; i++)
            {
                pCommand->pResp[i] = pMciHw->HSMCI_RSPR[0];
            }
        }

        /* If no error occured, the transfer is successful */
        if (pCommand->state == SDMMC_CMD_PENDING)
        {
            pCommand->state = 0;
        }
        /* Error ? Reset to remove transfer statuses */
        else
        {
            MCI_Reset(pMci, 1);
        }

        /* Disable PDC */
        pMciHw->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;

        /* Disable interrupts */
        pMciHw->HSMCI_IDR = pMciHw->HSMCI_IMR;

        /* Disable peripheral */
        PMC_DisablePeripheral(pMci->mciId);

        /* Release the semaphore */
        pMci->semaphore++;

        /* Invoke the callback associated with the current command (if any) */
        if ( pCommand->callback )
        {
            (pCommand->callback)(pCommand->status, pCommand->pArg);
        }
    }
}
/**@}*/
