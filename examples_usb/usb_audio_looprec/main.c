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
 *  \page usb_audio_looprec USB Audio Speaker & Loopback-Recorder Example
 *
 *  \section Purpose
 *
 *  The USB Audio Speaker & Loopback-Recorder Example will help you to get
 *  familiar with the USB Device Port(UDP) and DACC on SAM microcontrollers.
 *  Also it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB Audio Device
 *  class.
 *
 *  You can find following information depends on your needs:
 *  - Sample usage of USB Audio Device Class driver and DACC driver.
 *  - USB Audio Class driver development based on the USB Framework.
 *  - USB enumerate sequence, the standard and class-specific descriptors and
 *    requests handling.
 *  - The initialize sequence and usage of UDP interface.
 *  - The initialize sequence and usage of DACC interface with PDC.
 *
 *  \section Related
 *
 *  - dacc: DACC interface driver
 *  - usb: USB Framework, Audio Device Class driver and UDP interface driver
 *      - \ref usbd_framework
 *         - \ref usbd_api
 *      - \ref usbd_audio_rec_drv
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3S evaluation kits that have both
 *  UDP and DACC.
 *
 *  \section Description
 *
 *  When an EK running this program connected to a host (PC for example), with
 *  USB cable, the EK appears as a desktop speaker for the host. Then the host
 *  can play sound through host software. The %audio stream from the host is
 *  then sent to the EK, and eventually sent to %audio DAC connected to the
 *  amplifier. At the same time, the %audio stream received is also sent
 *  back to host from EK for recording.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6421.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a> application note or to the
 *     <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *     IAR EWARM User Guide</a>, depending on your chosen solution.
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear:
 *  \code
 *  -- USB Device Audio LoopREC Example xxx --
 *  -- xxxxxx-xx
 *  -- Compiled: xxx xx xxxx xx:xx:xx --
 *  \endcode
 *  -# When connecting USB cable to windows, the LED blinks, and the host
 *     reports a new USB %device attachment (if it's the first time you connect
 *     an %audio speaker demo board to your host). You can find new
 *     "USB Composite Device" and "USB Audio Device" appear in the hardware
 *     %device list.
 *  -# You can play sound in host side through the USB Audio Device, and it
 *     can be heard from the earphone connected to the EK.
 *  -# When playing sound, you can also record through the USB Audio Device on
 *     the host.
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_audio_looprec example.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "include/USBD_LEDs.h"
#include "include/USBD_Config.h"

#include "AUDDSpeakerPhoneDriver.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/**  Number of available audio buffers. */
#define BUFFER_NUMBER       8
/**  Number of available microphone buffers. */
#define BUFFER_NUMBER_MIC   7
/**  Size of one buffer in bytes. */
#define BUFFER_SIZE         (AUDDevice_BYTESPERFRAME \
                             + AUDDevice_BYTESPERSUBFRAME)

/**  Delay in ms for starting the DAC transmission
     after a frame has been received. */
#define DAC_DELAY           2

/**  Delay in ms for starting the USB IN stream
     after interface selected. */
#define ADC_DELAY           50

/** Audio output channel RIGHT */
#define CHANNEL_R       DACC_CHANNEL_0
/** Audio output channel LEFT */
#define CHANNEL_L       DACC_CHANNEL_1

#define MID_VALUE_16BITS  (0x7FFF)
#define MASK_12BITS       (0xFFF)

/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

/** Descriptor list for USB Audio SpeakerPhone Driver */
extern const USBDDriverDescriptors auddSpeakerPhoneDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

/**  Data buffers for receiving audio frames from the USB host. */
static uint8_t buffers[BUFFER_NUMBER][BUFFER_SIZE];
/**  Number of samples stored in each data buffer. */
static uint32_t bufferSizes[BUFFER_NUMBER];
/**  Next buffer in which USB data can be stored. */
static uint32_t inBufferIndex = 0;
/**  Next buffer which should be sent to the DAC. */
static uint32_t outBufferIndex = 0;
/**  Number of buffers that can be sent to the DAC. */
static volatile uint32_t numBuffersToSend = 0;

/**  Buffer list information for a write frame list */
static USBDTransferBuffer frmMbl[BUFFER_NUMBER];

/**  Current state of the playback stream interface. */
static volatile uint8_t isPlyActive = 0;
/**  Current state of the DAC transmission. */
static volatile uint8_t isDacActive = 0;
/**  Current state of the record stream interface. */
static volatile uint8_t isRecActive = 0;
/**  Number of buffers to wait for before the DAC starts to transmit data. */
static volatile uint8_t dacDelay;
/**  Number of buffers to wait for before the ADC starts to transmit data. */
static volatile uint8_t adcDelay;

/**  Default TC frequency setting backup: RA */
static uint32_t defaultRA;

/*----------------------------------------------------------------------------
 *         VBus monitoring (optional)
 *----------------------------------------------------------------------------*/

/**  VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;

/**
 *  Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus(const Pin *pPin)
{
    /* Check current level on VBus */
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("VBUS conn\n\r");
        USBD_Connect();
    }
    else {

        TRACE_INFO("VBUS discon\n\r");
        USBD_Disconnect();
    }
}

/**
 *  Configures the VBus pin to trigger an interrupt when the level on that pin
 *  changes.
 */
static void VBus_Configure( void )
{
    /* Configure PIO */
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    /* Check current level on VBus */
    if (PIO_Get(&pinVbus)) {

        /* if VBUS present, force the connect */
        USBD_Connect();
    }
    else {
        TRACE_INFO("discon\n\r");
        USBD_Disconnect();
    }
}

/**
 * Enable/Disable audio channels
 */
static void AudioPlayEnable(uint8_t enable)
{
    if (enable == 1) {
        TC_Start(TC0, 0);
        DACC_EnableChannel(DACC, CHANNEL_R);
        DACC_EnableChannel(DACC, CHANNEL_L);
    }
    else if (enable == 0) {
        TC_Stop(TC0, 0);
        DACC_DisableChannel(DACC, CHANNEL_R);
        DACC_DisableChannel(DACC, CHANNEL_R);
    }
}

/**
 * Configure the TC0 and DACC for audio output.
 * \param sampleRate Audio sample rate.
 * \param nbChannels Number of audio channels.
 * \param mck        MCK frequence.
 */
static void ConfigureAudioPlay(uint32_t sampleRate,
                               uint8_t nbChannels,
                               uint32_t mck)
{
    uint32_t div = 2;
    uint32_t tcclks = TC_CMR_TCCLKS_TIMER_CLOCK1;
    uint32_t freq = sampleRate * nbChannels;
    double   ra, rc;

    /* Enable TC0 Peripheral */
    PMC_EnablePeripheral(ID_TC0);
    {
        uint32_t divs[5] = {2, 8, 32, 128,BOARD_MCK / 32768};
        uint8_t i = 0;
        divs[4] = mck/32768;
        /* Minimize DIV & Maxmize RC for better waveform */
        while(freq < ((mck / divs[i]) / 65536)) {
            ++ i;
            if (i == 5) {
                TRACE_FATAL("Cann't find TC0 divisor!\n\r");
            }
        }
        div = divs[i];
        tcclks = i;
    }
    /* Configure TC for tioa output: 48M -> 48K*2 */
    TC_Configure(TC0,0, tcclks /*MCK/2*/
                        | TC_CMR_ACPC_SET
                        | TC_CMR_WAVE
                        | TC_CMR_ACPA_CLEAR
                        | TC_CMR_CPCTRG);
    /* 50% duty ,freq frequency*/
    ra = (((double)mck/div)/(freq*2) + 0.5 );
    rc = (((double)mck/div)/(freq)   + 0.99);
    TC0->TC_CHANNEL[0].TC_RA = (uint32_t)ra;
    TC0->TC_CHANNEL[0].TC_RC = (uint32_t)ra*2;
    defaultRA = (uint32_t)ra;

    printf("-I- MCK %dKHz, Div %d(%x), RA: %d*.01(%x), RC: %d*.01(%x)\n\r",
            (int)mck/1000, (int)div, (unsigned int)div,
            (int)(ra*100), (int)ra, (int)(rc*100), (int)rc);

    /* Initialize DACC with HW as trigger */
    DACC_Initialize( DACC, ID_DACC,
                     1,
                     1,
                     0, /* DACC_MR_WORD_HALF */
                     0,
                     BOARD_MCK,
                     8,
                     CHANNEL_R,
                     1,
                     16);
    /* channel number is in the first 4 significant bits*/
    DACC->DACC_MR |= DACC_MR_TAG;
    /* disable PDC for DAC */
    DACC->DACC_PTCR = DACC_PTCR_TXTDIS;
    /* Mute */
    AudioPlayEnable(0);
    /* Enable DACC ISR */
    NVIC_EnableIRQ(DACC_IRQn);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 * Buffer translation for PCM format
 * \param pBuffer    Pointer for source and destination buffer
 * \param size       Buffer size in number of samples
 * \param nbChannels Channel numbers for source data
*/
static void Pcm2DacTranslate(int16_t *pBuffer,
                             uint32_t size,
                             uint8_t nbChannels)
{
    uint32_t i = 0;
    while(i <size){
        if(nbChannels > 1) {
            /* channle0-0,channel1-1*/
            pBuffer[i] = ( ( (pBuffer[i] + MID_VALUE_16BITS) >> 4 )
                           & MASK_12BITS )
                       | ( (uint16_t)(i%2) << 12);
        }
        else if (nbChannels == 1){
          /* one channel data stored continously */
          pBuffer[i] = ( (pBuffer[i] + MID_VALUE_16BITS) >> 4 )
                         & MASK_12BITS;
        }
        i++;
    }
}

/**
 * Play PCM data through DAC
 * \param pBuffer    Pointer to data buffer (16-Bit aligned)
 * \param nbSamples  Number of audio data samples
 * \param nbChannels Number of audio channels
 */
static void DacPlayBuffer(uint8_t* pBuffer,
                          uint32_t nbSamples,
                          uint8_t  nbChannels)
{
    Pcm2DacTranslate((int16_t*)pBuffer, nbSamples, nbChannels);
    DACC_WriteBuffer(DACC, (uint16_t*)pBuffer, nbSamples);
}

/**
 * Handles interrupts coming from the DACC.
 */
void DAC_IrqHandler(void)
{
    Dacc *pDac = DACC;
    uint32_t sr = pDac->DACC_ISR;

    if (sr & DACC_ISR_TXBUFE) {
        /* End of transmission */
        AudioPlayEnable(0);
        DACC_DisableIt(pDac, DACC_IDR_TXBUFE | DACC_IDR_ENDTX);
        pDac->DACC_PTCR = DACC_PTCR_TXTDIS;
        isDacActive = 0;
    }
    else if (numBuffersToSend) {
        /* Check the number of available buffers */
        if (numBuffersToSend > DAC_DELAY) {
            //printf("%d+", numBuffersToSend);
            /* Speed up DAC freq (TC) */
            {   uint32_t newRA = defaultRA - (numBuffersToSend-DAC_DELAY);
                if (TC0->TC_CHANNEL[0].TC_RA != newRA) {
                    TC0->TC_CHANNEL[0].TC_RA =  newRA;
                    TC0->TC_CHANNEL[0].TC_RC =  newRA*2;
                }
            }
        }
        else if (numBuffersToSend < DAC_DELAY) {
            //printf("%d-", numBuffersToSend);
            /* Slow down DAC (TC) */
            {   uint32_t newRA = defaultRA + (DAC_DELAY-numBuffersToSend);
                if (TC0->TC_CHANNEL[0].TC_RA != newRA) {
                    TC0->TC_CHANNEL[0].TC_RA = newRA;
                    TC0->TC_CHANNEL[0].TC_RC = newRA*2;
                }
            }
        }
        else {
            /* Keep DAC (TC) */
            if (TC0->TC_CHANNEL[0].TC_RA != defaultRA) {
                TC0->TC_CHANNEL[0].TC_RA = defaultRA;
                TC0->TC_CHANNEL[0].TC_RC = defaultRA*2;
            }
        }
        /* Load next buffer */
        DacPlayBuffer(buffers[outBufferIndex],
                      bufferSizes[outBufferIndex],
                      AUDDevice_NUMCHANNELS);
        outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER;
        numBuffersToSend --;
    }
    else {
        /* Last frame remaining */
        DACC_DisableIt(pDac, DACC_IDR_ENDTX);
    }
}

/**
 *  Invoked when a frame has been sent.
 */
static void FrameSent(uint32_t unused,
                      uint8_t status,
                      uint32_t transferred,
                      uint32_t remaining)
{
    if (status == USBD_STATUS_PARTIAL_DONE) {
        /* A frame finished */
    }
    else if (status == USBD_STATUS_SUCCESS) {
        /* List finished */
        printf("isoE ");
    }
    else if (status == USBD_STATUS_CANCELED) {
        /* Interface OFF */
        printf("isoC ");
    }
}

/**
 *  Invoked when a frame has been received.
 */
static void FrameReceived(uint32_t unused,
                          uint8_t status,
                          uint32_t transferred,
                          uint32_t remaining)
{
    Dacc *pDac = DACC;

    if (status == USBD_STATUS_SUCCESS) {

        /* Loopback! add this buffer to write list */
        if (!isRecActive)  {}
        else if (adcDelay) adcDelay --;
        else {
            AUDDSpeakerPhoneDriver_Write(buffers[inBufferIndex],
                                         AUDDevice_BYTESPERFRAME);
        }

        /* Update input status data */
        bufferSizes[inBufferIndex] = transferred
                                        / AUDDevice_BYTESPERSAMPLE;
        inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER;
        numBuffersToSend++;

        /* Start DAc transmission if necessary */
        if (!isDacActive) {

            dacDelay = DAC_DELAY;
            isDacActive = 1;
        }
        /* Wait until a few buffers have been received */
        else if (dacDelay > 0) {

            dacDelay--;
        }
        /* Start sending buffers */
        else if ((pDac->DACC_PTSR & DACC_PTSR_TXTEN) == 0) {
            AudioPlayEnable(1);

            DacPlayBuffer(buffers[outBufferIndex],
                          bufferSizes[outBufferIndex],
                          AUDDevice_NUMCHANNELS);
            outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER;
            numBuffersToSend --;

            DacPlayBuffer(buffers[outBufferIndex],
                          bufferSizes[outBufferIndex],
                          AUDDevice_NUMCHANNELS);
            outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER;
            numBuffersToSend --;

            DACC_EnableIt(pDac, DACC_IER_TXBUFE | DACC_IER_ENDTX);
            pDac->DACC_PTCR = DACC_PTCR_TXTEN;
        }
    }
    else if (status == USBD_STATUS_ABORTED) {
        /* Error , ABORT, add NULL buffer */
        bufferSizes[inBufferIndex] = 0;
        inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER;
        numBuffersToSend++;
    }
    else {
        /* Packet is discarded */
    }

    /* Receive next packet */
    AUDDSpeakerPhoneDriver_Read(buffers[inBufferIndex],
                           AUDDevice_BYTESPERFRAME,
                           (TransferCallback) FrameReceived,
                           0); // No optional argument
}

/**
 * \brief Configure 48MHz Clock for USB
 */
static void _ConfigureUsbClock(void)
{
    /* Enable PLLB for USB */
    PMC->CKGR_PLLBR = CKGR_PLLBR_DIVB(1)
                    | CKGR_PLLBR_MULB(7)
                    | CKGR_PLLBR_PLLBCOUNT_Msk;
    while((PMC->PMC_SR & PMC_SR_LOCKB) == 0);
    /* USB Clock uses PLLB */
    PMC->PMC_USB = PMC_USB_USBDIV(1)    /* /2   */
                 | PMC_USB_USBS;        /* PLLB */
}

/*----------------------------------------------------------------------------
 *         Callbacks re-implementation
 *----------------------------------------------------------------------------*/

/**
 * Invoked after the USB driver has been initialized. By default, configures
 * the UDP/UDPHS interrupt.
 */
void USBDCallbacks_Initialized( void )
{
    NVIC_EnableIRQ( UDP_IRQn ) ;
}

/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
    AUDDSpeakerPhoneDriver_ConfigurationChangeHandler(cfgnum);
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(unsigned char interface,
                                                 unsigned char setting)
{
    AUDDSpeakerPhoneDriver_InterfaceSettingChangedHandler(interface, setting);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    AUDDSpeakerPhoneDriver_RequestHandler(request);
}

/**
 *  Invoked when an audio channel get muted or unmuted. Mutes/unmutes the
 *  channel at the DAC level.
 *  \param mic      Microphone/Speaker stream changed.
 *  \param channel  Channel number that changed.
 *  \param muted    Indicates the new mute status of the channel.
 */
void AUDDSpeakerPhoneDriver_MuteChanged(uint8_t mic,
                                        uint8_t channel,
                                        uint8_t muted)
{
    printf("m%d,%d,%d ", mic, channel, muted);
    /* Speaker Master channel */
    if (!mic && channel == AUDDSpeakerPhoneDriver_MASTERCHANNEL) {

        if (muted) {

            //AudioPlayEnable(0);
            TRACE_WARNING("MuteMaster ");
        }
        else {

            TRACE_INFO("UnmuteMaster ");
            //AudioPlayEnable(1);
        }
    }
}

/**
 *  Invoked when an audio streaming interface setting changed.
 *  Audio stream is automatically reseted.
 *  Actually control streaming rate.
 *  \param mic         Microphone/Speaker stream changed.
 *  \param newSetting  New stream (interface) setting.
 */
void AUDDSpeakerPhoneDriver_StreamSettingChanged(uint8_t mic,
                                                 uint8_t newSetting)
{
    /* Speaker stream */
    if (!mic) {

        if (newSetting) LED_Set(USBD_LEDOTHER);
        else            LED_Clear(USBD_LEDOTHER);
        isPlyActive = (newSetting > 0);
    }
    else {

        isRecActive = (newSetting > 0);
        adcDelay = ADC_DELAY;
    }
}

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief usb_audio_looprec Application entry point.
 *
 *  Starts the driver and waits for an audio input stream to forward to the DAC.
 */
int main(void)
{
    volatile uint8_t usbConn = 0;
    volatile uint8_t audioOn = 0, recOn = 0;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB Device Audio LoopREC Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Audio STREAM LED */
    LED_Configure(USBD_LEDOTHER);

    /* Configure USB & DACC interrupt priority */
    NVIC_SetPriority(UDP_IRQn,  6);
    NVIC_SetPriority(DACC_IRQn, 6);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Configure Audio */
    ConfigureAudioPlay(AUDDevice_SAMPLERATE,
                       AUDDevice_NUMCHANNELS,
                       BOARD_MCK);

    /* USB audio driver initialization */
    AUDDSpeakerPhoneDriver_Initialize(&auddSpeakerPhoneDriverDescriptors);

    /* USB audio frame write configure */
    AUDDSpeakerPhoneDriver_SetupWrite(frmMbl,
                                      NULL,
                                      BUFFER_NUMBER,
                                      2,
                                      (TransferCallback) FrameSent,
                                      NULL);

    /* connect if needed */
    VBus_Configure();

    /* Infinite loop */
    while (1) {

        if (USBD_GetState() < USBD_STATE_CONFIGURED) {
            usbConn = 0;
            continue;
        }

        if (isPlyActive) {
            /* Try to Start Reading the incoming audio stream */
            AUDDSpeakerPhoneDriver_Read(buffers[inBufferIndex],
                                   AUDDevice_BYTESPERFRAME,
                                   (TransferCallback) FrameReceived,
                                   0); // No optional argument
        }

        if (audioOn) {
            if(isDacActive == 0) {
                printf("plyE ");
                audioOn = 0;
            }
        }
        else if (isDacActive) {
            printf("plyS ");
            audioOn = 1;
        }

        if (recOn) {
            if (isRecActive == 0) {
                printf("recE ");
                recOn = 0;
            }
        }
        else if (isRecActive) {
            printf("recS ");
            recOn = 1;
        }
    }
}

