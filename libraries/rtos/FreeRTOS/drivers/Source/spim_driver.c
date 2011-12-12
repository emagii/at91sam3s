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

/** \addtogroup spi_pdc_module SPI PDC driver
 *  \ingroup spi_at25_module
 * The Spi driver is a low level spi driver which performs SPI device Initializes,
 * spi transfer and receive. It can be used by upper SPI driver such as AT25/26
 * driver and AT45 driver.
 *
 * \section Usage
 * <ul>
 * <li> Initializes a SPI instance and the corresponding SPI hardware,
 *    Configure SPI in Master Mode using SPID_Configure().</li>
 * <li> Configures the SPI characteristics (such as Clock Polarity, Phase,
 *    transfers delay and Baud Rate) for the device corresponding to the
 *    chip select using SPID_ConfigureCS().</li>
 * <li> Starts a SPI master transfer using SPID_SendCommand().
 *    The transfer is performed using the PDC channels. </li>
 *    <li> It enable the SPI clock.</li>
 *    <li> Set the corresponding peripheral chip select.</li>
 *    <li> Initialize the two SPI PDC buffers.</li>
 *       <li> Initialize SPI_TPR and SPI_TCR with SPI command data and size
 *         to send command data first.</li>
 *       <li> Initialize SPI_RPR and SPI_RCR with SPI command data and size
 *         as dummy value.</li>
 *       <li> Initialize SPI_TNPR and SPI_TNCR with rest of the data to be
 *        transfered.(if the data specified in cmd structure)</li>
 *       <li> Initialize SPI_RNPR and SPI_RNCR with rest of the data to be
 *         received.(if the data specified in cmd structure)</li>
 *    <li> Initialize the callback function if specified.</li>
 *    <li> Enable transmitter and receiver.</li>
 *    <li> Example for sending a command to the dataflash through the SPI.</li>
 * \code
 *      /// Build command to be sent.
 *      ...
 *      // Send Command and data through the SPI
 *      if (SPID_SendCommand(pAt25->pSpid, pCommand)) {
 *          return AT25_ERROR_SPI;
 *      }
 * \endcode
 * <li> The SPI_Handler() must be called by the SPI Interrupt Service Routine
 *    with the corresponding Spi instance. It is invokes to check for pending
 *    interrupts. </li>
 *    <li> Example for initializing SPI interrupt handler in upper application.</li>
 * \code
 *        AIC_ConfigureIT(AT91C_ID_SPI, 0, SPI_Handler);
 * \endcode
 * </ul>
 * Related files :\n
 * \ref spi_master.c\n
 * \ref spi_master.h.\n
*/
/*@{*/
/*@}*/


/**
 * \file
 *
 * Implementation of SPI PDC driver.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*----------------------------------------------------------------------------
 *        Internal structures
 *----------------------------------------------------------------------------*/
/**
 * \brief Spi Status object updated by the AT25_SpiCallback once an SPI operation
 *     is achieved.
 */
typedef struct _SpimRwData
{
	xSemaphoreHandle spiSemaphore; /**< Semaphore to protect the SPI periph */
	xSemaphoreHandle txvSemaphore; /**< Semaphore used for synchronous commands */
    volatile ESpimStatus txvStatus; /**< SPI transfer status */
} SpimRwData;

/**
 * \brief Spi Driver associated with an Hw Spi peripheral
 */
typedef struct _Spim {
    Spi   *pSpi;        /**< Pointer to SPI Hw peripheral */
    IRQn_Type spiId;   /**< Spi peripheral ID */
    Pin  spiPins[3];   /**< Spi spck, miso, mosi pin description */
    Pin  csPins[4];    /**< Spi NPCS pin descriptiom */
    SpimRwData *pData; /**< Pointer to Spim R/W data */
} Spim;

/* Local variables specific to the sam3n architecture */
SpimRwData sam3nSpimRwData;

const Spim sam3nSpim = {
  SPI, SPI_IRQn,
  {PINS_SPI},
  {PIN_SPI_NPCS0_SDCARD, PIN_SPI_NPCS1_ZIGBEE, PIN_SPI_NPCS2_LCD, PIN_SPI_NPCS3_AT25},
  &sam3nSpimRwData
};

/*----------------------------------------------------------------------------
 *        SPI Handler functions
 *----------------------------------------------------------------------------*/
/**
 * \brief The SPI_Handler must be called by the SPI Interrupt Service Routine with the
 * corresponding Spi instance.
 *
 */
extern void SPIM_Handler( void )
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
    Spi *pSpi   = pSpim->pSpi;
    SpimRwData  *pSpimRwData = pSpim->pData;
    uint32_t    dwSpiReg;

    /* Disable interrupts */
    dwSpiReg = SPI_GetItMask(pSpi);
    SPI_DisableIt( pSpi, dwSpiReg) ;

    pSpimRwData->txvStatus = SPIM_OK;

    /* Check status */
    dwSpiReg = SPI_GetStatus(pSpi);
    if (dwSpiReg & SPI_SR_OVRES) {
       pSpimRwData->txvStatus = SPIM_ERROR_OVERRUN;
    }
    if (dwSpiReg & (SPI_SR_MODF | SPI_SR_UNDES)) {
    	pSpimRwData->txvStatus = SPIM_ERROR;
    }
    /* Release the semaphore to notify the end of transfer */
    xSemaphoreGiveFromISR( pSpimRwData->txvSemaphore, &xHigherPriorityTaskWoken );
    
    /* We may want to switch to the SPIM task, if this message has made
    it the highest priority task that is ready to execute. */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );


}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Initializes an AT25 driver instance with the given SPI driver and chip
 * select value. This function is invoked by the application.
 * \param pSpi  Pointer to an Spi hw peripheral.
 */
extern ESpimStatus SPIM_Initialize( Spi* pSpi )
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
	SpimRwData *pSpimRwData = pSpim->pData;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);
    assert(pSpimRwData);

    /* Enable clock on SPI */
    PMC_EnablePeripheral( pSpim->spiId ) ;

    /* Configure the SPIin master mode */
    SPI_Configure( pSpi, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS);

    /* Disable clock on SPI */
    PMC_DisablePeripheral( pSpim->spiId ) ;

    /* Initialize semaphore */
    vSemaphoreCreateBinary( pSpimRwData->spiSemaphore );
    if( pSpimRwData->spiSemaphore == NULL )
    	return SPIM_ERROR;
    vSemaphoreCreateBinary( pSpimRwData->txvSemaphore );
    if( pSpimRwData->txvSemaphore == NULL )
    	return SPIM_ERROR;

    /* Configure PIO for SPCK, MOSI and MISO */
    PIO_Configure(pSpim->spiPins, PIO_LISTSIZE(pSpim->spiPins));

    /* Configure the interrupt with the according priority */
    NVIC_SetPriority( pSpim->spiId, configMAX_SYSCALL_INTERRUPT_PRIORITY + 16 ) ;
    NVIC_EnableIRQ( pSpim->spiId ) ;

    return SPIM_OK;

}

/**
 * \brief Select the slave device. This function shall lock the semaphore
 * to prevent from concurrent acces on another slave device connected to
 * the same SPI HW.
 *
 * \param pSpi  Pointer to an Spi hw peripheral.
 * \param dwCS      Chip Select
 *
 * \return SPIM_OK = 0 if successful; otherwise, returns SPIM error code.
  */
extern ESpimStatus SPIM_LockCS(
	    Spi *pSpi,
	    uint8_t ucCS)
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
	SpimRwData *pSpimRwData = pSpim->pData;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);
    assert(ucCS < 4);
    assert(pSpimRwData);

    /* This is the place to test a semaphore as upper layer tasks can do
	 * concurrent access to the SPI hw */
    if( xSemaphoreTake( pSpimRwData->spiSemaphore, portMAX_DELAY ) != pdTRUE )
        return SPIM_BUSY;

    /* Enable the SPI clock */
    PMC_EnablePeripheral( pSpim->spiId ) ;

    /* Enable the SPI peripheral */
    SPI_Enable(pSpi);

    /* Select the corresponding chip select */
    SPI_EnableCs(pSpi, ucCS);

    return SPIM_OK;
}

/**
 * \brief Select the slave device. This function shall lock the semaphore
 * to prevent from concurrent acces on another slave device connected to
 * the same SPI HW.
 *
 * \param pSpi  Pointer to an Spi hw peripheral.
 *
*/
extern void SPIM_ReleaseCS(
	    Spi *pSpi)
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
	SpimRwData *pSpimRwData = pSpim->pData;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);

    /* The following instruction set all NPCS: all transfer are supposed */
    /* to be achieved */
    SPI_Disable(pSpi);;

    /* Disable the SPI clock */
    PMC_DisablePeripheral( pSpim->spiId ) ;

    /* This is the place to release a semaphore so upper layer tasks can do
	* concurrent access to the SPI hw */
    xSemaphoreGive( pSpimRwData->spiSemaphore );
}

/**
 * \brief Configures the SPI timings for the device corresponding to the cs.
 *
 * \param pSpi  Pointer to an Spi hw peripheral.
 * \param ucCS  number corresponding to the SPI chip select.
 * \param dwCSR  SPI_CSR value to setup.
 */
extern void SPIM_ConfigureCS( Spi* pSpi, uint8_t ucCS, uint32_t dwCSR )
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);
    assert(ucCS < 4); /* Check correct initialization */

    /* Chip select shall not rise after each transfer */
    /* It will rise only after RealeaseCS() is invoked */
    dwCSR |= SPI_CSR_CSAAT;

    /* Configure PIO for NPCS */
    PIO_Configure(&(pSpim->csPins[ucCS]), 1);

    /* Configure SPI timings for the corresponding device */
    SPI_ConfigureNPCS( pSpi, ucCS, dwCSR ) ;

}

/**
 * \brief Sends a character through the SPI. The command is made up
 * of two parts: the first is used to transmit the command byte and optionally,
 * address and dummy bytes. The second part is the data to send or receive.
 * This function does not block: it returns as soon as the transfer has been
 * started. An optional callback can be invoked to notify the end of transfer.
 *
 * \param pSpi  Pointer to an Spi hw peripheral.
 * \param pwData  Pointer to the SpimCmd command to execute.
 *
 * \return SPIM_SUCCESS = 0 if successful; otherwise, returns SPIM error code.
 */
extern ESpimStatus SPIM_TransferData(
    Spi *pSpi,
    uint16_t wOutData,
    uint16_t *pwInData)
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
    SpimRwData  *pSpimRwData = pSpim->pData;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);

    /* Use a semaphore as an event  */
    if( xSemaphoreTake( pSpimRwData->txvSemaphore, portMAX_DELAY ) != pdTRUE )
        return SPIM_ERROR;

    /* Output a data */
    SPI_Write( pSpi, 0, wOutData );

    /* Enable buffer complete interrupt*/
    SPI_EnableIt( pSpi, SPI_IER_RDRF );

    /* Poll the end of transfer waiting that ISR releases the semaphore  */
     if( xSemaphoreTake( pSpimRwData->txvSemaphore, portMAX_DELAY ) != pdPASS )
         return SPIM_ERROR;
     xSemaphoreGive( pSpimRwData->txvSemaphore );

    /* Return the data received */
    if (pwInData) {
    	*pwInData = SPI_Read( pSpi );
    }
    /* TODO: handle transfer status */
    return (pSpimRwData->txvStatus);
}

/**
 * \brief Starts a SPI master transfer.
 *
 * \param pSpi  Pointer to an Spi hw peripheral.
 * \param pOutBuffer Pointer to the buffer to send.
 * \param pInBuffer Pointer memory buffer to store data received
 * \param bufferSize Buffer size
 * \return SPIM_OK if the transfer has been started successfully; otherwise returns
 * SPIM_BUSY is the driver is in use, or SPIM_ERROR if the command is not
 * valid.
 */
extern ESpimStatus SPIM_TransferBuffer(
		Spi *pSpi,
		const uint16_t *pOutBuffer,
		uint16_t *pInBuffer,
                uint16_t wBufferSize)
{
    /* pSpim shall be initialized with the instance which corresponds */
    /* to the Spi Hw interface */
    const Spim *pSpim = &sam3nSpim;
    SpimRwData  *pSpimRwData = pSpim->pData;

    /* Sanity checks */
    assert(pSpim->pSpi == pSpi);

    /* Use a semaphore as an event  */
    if( xSemaphoreTake( pSpimRwData->txvSemaphore, portMAX_DELAY ) != pdTRUE )
        return SPIM_ERROR;

    /* Disable transmitter and receiver*/
    SPI_PdcDisableRx( pSpi ) ;
    SPI_PdcDisableTx( pSpi ) ;

    /* Initialize the two SPI PDC buffer and enable transmitter or receiver*/
    SPI_WriteBuffer( pSpi, pOutBuffer,  wBufferSize ) ;
    if (pInBuffer) {
        SPI_ReadBuffer ( pSpi, pInBuffer, wBufferSize ) ;
        SPI_PdcEnableRx( pSpi ) ;
        SPI_EnableIt( pSpi, SPI_IER_RXBUFF ) ;
    }
    else {
        SPI_EnableIt( pSpi, SPI_IER_TXBUFE ) ;
    }
    SPI_PdcEnableTx( pSpi ) ;

    /* Poll the end of transfer waiting that ISR releases the semaphore  */
     if( xSemaphoreTake( pSpimRwData->txvSemaphore, portMAX_DELAY ) != pdTRUE )
         return SPIM_ERROR;
     
     /* Some data may still be pending in the TDR and in the shift register */
     if (pInBuffer == NULL) {
        SPI_EnableIt( pSpi, SPI_IER_TXEMPTY ) ;
         if( xSemaphoreTake( pSpimRwData->txvSemaphore, portMAX_DELAY ) != pdTRUE )
             return SPIM_ERROR;
     }
     xSemaphoreGive( pSpimRwData->txvSemaphore );

    /* If no receive buffer have been allocated then all values have been accululated */
    /* in the same register so getting OVERRUN error is normal */
    if ((pSpimRwData->txvStatus == SPIM_ERROR_OVERRUN) && (pInBuffer == NULL)) {
        pSpimRwData->txvStatus = SPIM_OK;
    }
    /* TODO: handle transfer status */
    return (pSpimRwData->txvStatus);
}

