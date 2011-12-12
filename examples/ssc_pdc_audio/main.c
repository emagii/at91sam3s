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
 * \page ssc_pdc_audio SSC with PDC Audio Example
 *
 * \section Purpose
 * This example uses the Synchronous Serial Controller (SSC) of an SAM3S microcontroller
 * to output an audio steam through the on-board WM8731 CODEC.
  *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits with external codec WM8731 components.
 *
 * \section Description
 * This program plays a WAV file pre-loaded into the flash The audio stream is sent through
 * the SSC interface connected to the on-board WM8731, enabling the sound to be audible
 * using a pair of headphones.
 *
 * Since the WM8731 DAC requires that it be feeded a master clock multiple of the sample rate,
 * it is difficult to handle any WAV file. As such, this example application is limited to
 * playing files with the following format:
 * - Format: WAV
 * - Sample rate: 48 kHz
 *
 * The code can be roughly broken down as follows:
 * <ul>
 *   <li> Enable the clock </li>
 *   <li> Load WAV file information</li>
 *   <li> Configure and enable the Codec</li>
 *   <li> Configure and enable the SSC interrupt</li>
 *   <li> Play WAV file</li>
 * </ul>
 *
 * \section usage
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- ssc_pdc_audio example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Menu :
 *     ------
 *     W: Play the WAV file pre-loaded in flash
 *     I: Display the information of the WAV file
 *    \endcode
 * The user can then choose any of the available options to perform the described action.
 *
 * \section References
 * - ssc_pdc_audio/main.c
 * - ssc.c
 * - twi.c
 * - twid.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the ssc audio example.
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local macros
 *----------------------------------------------------------------------------*/

#define min( a, b ) (((a) < (b)) ? (a) : (b))

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define I2S_MASTER_TX_SETTING(nb_bit_by_slot, nb_slot_by_frame) (\
                               SSC_TCMR_CKS_MCK |\
                               SSC_TCMR_CKO_CONTINUOUS |\
                               SSC_TCMR_START_RF_FALLING |\
                               SSC_TCMR_STTDLY(1) | \
                               SSC_TCMR_PERIOD(((nb_bit_by_slot * nb_slot_by_frame) / 2) - 1) )

#define I2S_TX_FRAME_SETTING(nb_bit_by_slot, nb_slot_by_frame) (\
                             (nb_bit_by_slot - 1) |\
                             SSC_TFMR_MSBF |\
                             SSC_TFMR_DATNB(nb_slot_by_frame - 1) |\
                             SSC_TFMR_FSLEN(nb_bit_by_slot - 1) |\
                             SSC_TFMR_FSOS_Msk )

/** Master clock frequency in Hz */
#define SSC_MCK               49152000

/** Address at which the WAV file is located*/
#define WAV_FILE_ADDRESS      0x400000

/** Maximum size in bytes of the WAV file.*/
#define MAX_WAV_SIZE           0x40000

/** TWI clock */
#define TWI_CLOCK               100000

/** Wav feature. */
#define SAMPLE_RATE              48000
#define SLOT_BY_FRAME                2
#define BITS_BY_SLOT                16

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** List of pins to configure. */
static const Pin pins[] = { PINS_TWI0,
                            PINS_SSC_CODEC,
                            PIN_PCK0};

/** Pointer to the playback WAV file header.*/
static const WavHeader *userWav = (WavHeader *) WAV_FILE_ADDRESS;


/** Indicates if the WAV file is currently being played.*/
static uint8_t isWavPlaying;

/** Number of samples which have already been transmitted.*/
static uint32_t transmittedSamples;

/** Number of samples that have not yet been transmitted.*/
static uint32_t remainingSamples;

/** Twi instance*/
static Twid twid;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Display the information of the WAV file (sample rate, stereo/mono and frame
 * size) on the DBGU.
 */

static void DisplayWavInfo(void)
{
    printf("%c[2J", 27);
    WAV_DisplayInfo(userWav);
    UART_GetChar();
}

/**
 * \brief Displays the user menu on the DBGU.
 */

static void DisplayMenu(void)
{
    printf("%c[2J-- ssc_pdc_audio --\n\r", 27);
    printf("Menu :\n\r");
    printf("------\n\r");

    /* Play a WAV file pre-loaded in Flash using SAM-BA */
    if (!isWavPlaying) {

        printf("  W: Play the WAV file pre-loaded in internal flash\n\r");
    }

    /* Display the information of the WAV file (sample rate, stereo/mono and frame size) */
    printf("  I: Display the information of the WAV file\n\r");

    /* Stop the current playback (if any) */
    if (isWavPlaying) {

        printf("  S: Stop playback\n\r");
    }
}


/**
 * \brief Interrupt handler for the SSC. Loads the PDC with the audio data to stream.
 */

 void SSC_IrqHandler(void)
{
    uint32_t status;
    uint32_t size;
    status = SSC->SSC_SR;
    /* Last buffer sent */
    if ((status & SSC_SR_TXBUFE) != 0) {

        isWavPlaying = 0;
        SSC_DisableInterrupts(SSC_IDR_TXBUFE | SSC_IDR_ENDTX);
        SSC->SSC_PTCR = SSC_PTCR_TXTDIS;
        DisplayMenu();
    }
    /* One buffer sent & more buffers to send */
    else if (remainingSamples > 0) {

        size = min(remainingSamples / (userWav->bitsPerSample / 8), 65535);
        SSC_WriteBuffer((void *) (WAV_FILE_ADDRESS + sizeof(WavHeader) + transmittedSamples), size);
        remainingSamples -= size * (userWav->bitsPerSample / 8);
        transmittedSamples += size * (userWav->bitsPerSample / 8);
    }
    /* One buffer sent, no more buffers */
    else {
        SSC_DisableInterrupts(SSC_IDR_ENDTX);
    }
}



/**
 * \brief Play a WAV file pre-loaded in SDRAM using SAM-BA.
 */
static void PlayWavFile(void)
{
    uint32_t size;

    assert( userWav->chunkSize < MAX_WAV_SIZE ) ; /* "-F- WAV file too big (increase MAX_WAV_SIZE)\n\r" */

    /* Start transmitting WAV file to SSC */
    remainingSamples = userWav->subchunk2Size;
    transmittedSamples = 0;

    /* Fill first PDC buffer */
    size = min(remainingSamples / (userWav->bitsPerSample / 8), 65535);
    SSC_WriteBuffer((void *) (WAV_FILE_ADDRESS + sizeof(WavHeader) + transmittedSamples), size);
    remainingSamples -= size * (userWav->bitsPerSample / 8);
    transmittedSamples += size * (userWav->bitsPerSample / 8);

    /* Fill second buffer if necessary */
    if (remainingSamples > 0) {

        size = min(remainingSamples / (userWav->bitsPerSample / 8), 65535);
        SSC_WriteBuffer((void *) (WAV_FILE_ADDRESS + sizeof(WavHeader) + transmittedSamples), size);
        remainingSamples -= size * (userWav->bitsPerSample / 8);
        transmittedSamples += size * (userWav->bitsPerSample / 8);
    }

    SSC_EnableInterrupts(SSC_IER_TXBUFE | SSC_IER_ENDTX);
}

/**
 * \brief  Stop the current playback (if any).
 */

static void StopPlayback(void)
{
    SSC_DisableInterrupts(SSC_IDR_TXBUFE | SSC_IDR_ENDTX);
    SSC->SSC_PTCR = SSC_PTCR_TXTDIS;
    SSC->SSC_TNCR = 0;
    SSC->SSC_TCR = 0;
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint8_t key;
    uint8_t isValid;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("\n\r\n\r\n\r");
    printf("-- Ssc-Pdc-audio Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* Configure all pins */
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    /* Enable TWI peripheral clock */
    PMC->PMC_PCER0 = 1 << ID_TWI0;
    /* Configure and enable the TWI (required for accessing the DAC) */
    TWI_ConfigureMaster(TWI0, TWI_CLOCK, SSC_MCK);
    TWID_Initialize(&twid, TWI0);

    /* Enable the DAC master clock */
    REG_PMC_PCK = PMC_MCKR_CSS_PLLA_CLK | PMC_MCKR_PRES_CLK_8;
    /* Programmable Clock 0 Output Enable*/
    REG_PMC_SCER = PMC_SCER_PCK0;
    /* Wait for the PCKRDYx bit to be set in the PMC_SR register*/
    while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);

    /* Load WAV file information */
    isValid = WAV_IsValid(userWav);
    assert( isValid ) ; /* "-F- Invalid WAV file provided\n\r" */
    isWavPlaying = 0;

    /* Sample rate must be 48kHz*/
    printf("-I- Sample rate = %d Hz\n\r", userWav->sampleRate);
    assert( userWav->sampleRate == 48000 ) ; /* "-F- The WAV file must have a sample rate of 48kHz\n\r" */

    /* Initialize the audio DAC */
    WM8731_DAC_Init(&twid, WM8731_SLAVE_ADDRESS);

    /* Configure SSC*/
    SSC_Configure(SAMPLE_RATE * BITS_BY_SLOT * 2, SSC_MCK);
    SSC_ConfigureReceiver(0, 0);
    SSC_ConfigureTransmitter( I2S_MASTER_TX_SETTING(BITS_BY_SLOT, SLOT_BY_FRAME),
                              I2S_TX_FRAME_SETTING(BITS_BY_SLOT, SLOT_BY_FRAME) );
    SSC_DisableTransmitter();

    /* Configure SSC interrupts */
    NVIC_DisableIRQ(SSC_IRQn);
    NVIC_ClearPendingIRQ(SSC_IRQn);
    NVIC_SetPriority(SSC_IRQn, 0);
    NVIC_EnableIRQ(SSC_IRQn);

    /* Enter menu loop */
    while (1) {

        /* Display menu */
        DisplayMenu();

        /* Process user input */
        key = UART_GetChar();

        /* Play WAV file */
        if ((key == 'W') && !isWavPlaying) {

            PlayWavFile();
            isWavPlaying = 1;
        }
        /* Display WAV information */
        else if (key == 'I') {

            DisplayWavInfo();
        }
        /* Stop playback*/
        else if ((key == 'S') && isWavPlaying) {

            StopPlayback();
            isWavPlaying = 0;
        }
    }
}

