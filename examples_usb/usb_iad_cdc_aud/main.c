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
 *  \page usb_iad_cdc_aud USB CDC(Serial)+Audio(Speaker) Example
 *
 *  \section Purpose
 *
 *  The Example will help you to get familiar with the USB Framework that is
 *  used for rapid development of USB-compliant class drivers such as USB
 *  Composite device that integrates a CDC Virtual Serial Port Function and
 *  an Audio Speaker Function.
 *
 *  You can find following information depends on your needs:
 *  - Sample usage of USB CDC & Audio Composite driver.
 *  - Sample usage of USB CDC Serial Function driver.
 *  - Sample usage of USB Audio Function driver and DACC driver.
 *  - USB CDC Serial Function driver development based on the USB Framwork.
 *  - USB Audio Function driver development based on the USB Framework.
 *  - USB enumerate sequence, the standard and class-specific descriptors and
 *    requests handling.
 *  - The initialize sequence and usage of UDP interface.
 *  - The initialize sequence and usage of ADC interface with PDC.
 *
 *  \section Related
 *
 *  - pio: Pin configurations and peripheral configure.
 *  - dacc: DACC interface driver
 *  - adc: ADC interface driver
 *  - usb: USB Framework, CDC, Audio function driver and UDP interface driver
 *      - \ref usbd_framework
 *         - \ref usbd_api
 *      - \ref usbd_composite "composite"
 *         - \ref usbd_composite_drv
 *      - \ref usbd_cdc "cdc-serial"
 *         - \ref usbd_cdc_serial_drv
 *      - \ref usbd_aud "audio"
 *         - \ref usbd_audio_rec_drv
 *  - projects: more detailed information for CDC(Serial) and Audio(Speaker)
 *      - \ref usb_core
 *      - \ref usb_cdc_serial
 *      - \ref usb_audio_speaker, \ref usb_audio_headphone
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3S evaluation kits that have UDP
 *  and DACC.
 *
 *  \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio. The
 * example composite devices are tested under windows XP (SP3). For CDC
 * serial port, additional windows driver file (CompositeCDCSerial.inf) can
 * be found at
 * libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf.
 *
 * The following is alternate update to fix the composite device support
 * on windows XP:
 *
 * \subsection install_win_sp3 Install Windows Service Pack 3 (SP3)
 *
 * All the fixes for USB generic driver are included in window XP service pack
 * 3. It can be found at
 * http://technet.microsoft.com/zh-cn/windows/bb794714(en-us).aspx .
 *
 * \subsection install_win_hotfix Install Windows Hot Fixes
 *
 * Two hot fixes are necessary for window to recognize the composite device
 * correctly:
 *
 * -# http://support.microsoft.com/kb/814560
 * -# http://support.microsoft.com/kb/918365
 *
 *  \section Description
 *
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, host will notice the attachment of a USB %device (USB Composite
 * Device) with a USB Virtual COM port(SAM3S USB to Serial Converter)
 * and a USB Audio Device.
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
 *  -- USB CDC + Audio Device Example xxx --
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
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_iad_cdc_aud example.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <CDCAUDDDriver.h>
#include <CDCDSerial.h>
#include <AUDDFunction.h>

#include <include/USBD_Config.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/*- CDC */
/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE \
    (CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)+2)

/*- Audio */
/**  Number of available audio buffers. */
#define BUFFER_NUMBER       6
/**  Size of one buffer in bytes. */
#define BUFFER_SIZE         (AUDDevice_BYTESPERFRAME)

/**  Buffer adjust level */
#define BUFFER_DELAY        (2)

/**  Delay in ms for starting the DAC transmission
     after a frame has been received. */
#define DAC_DELAY           BUFFER_DELAY

/** Audio output channel RIGHT */
#define SPEAKER_CHANNEL_R       DACC_CHANNEL_0
/** Audio output channel LEFT */
#define SPEAKER_CHANNEL_L       DACC_CHANNEL_1

#define MID_VALUE_16BITS  (0x7FFF)
#define MASK_12BITS       (0xFFF)
#define DACC_REFRESH      (8)
#define DACC_STARTUP      (16)

/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

/** Descriptor list for USB Audio SpeakerPhone Driver */
extern const USBDDriverDescriptors cdcauddDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/


/*- CDC */
/** Buffer for storing incoming USB data. */
static uint8_t usbSerialBuffer0[DATABUFFERSIZE];
/** Serial port openned */
static volatile uint8_t isSerialPortON = 0;

/*- Audio */
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

/**  Current state of the USB OUT transmission. */
static volatile uint8_t  isPlyActive = 0;
/**  Current state of the DAC transmission. */
static volatile uint8_t  isDacActive = 0;
/**  Number of buffers to wait for before the DAC starts to transmit data. */
static volatile uint8_t  dacDelay;

/**  Basic RA configure value for sampling frequency control */
static uint32_t bakRA;

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

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 * Enable/Disable audio speaker channels
 */
static void AudioSpeakerEnable(uint8_t enable)
{
    if (enable == 1) {
        TC_Start(TC0, 0);
        DACC_EnableChannel(DACC, SPEAKER_CHANNEL_R);
        DACC_EnableChannel(DACC, SPEAKER_CHANNEL_L);
    }
    else if (enable == 0) {
        DACC_DisableChannel(DACC, SPEAKER_CHANNEL_R);
        DACC_DisableChannel(DACC, SPEAKER_CHANNEL_L);
        TC_Stop(TC0, 0);
    }
}

/**
 * Configure the TC0 and DACC, ADC for audio playback/record.
 * \param sampleRate Audio sample rate.
 * \param nbChannels Number of audio channels.
 * \param mck        MCK frequence.
 */
static void _ConfigureAudio(uint32_t sampleRate,
                           uint8_t nbChannels,
                           uint32_t mck)
{
    uint32_t div = 2;
    uint32_t tcclks; // TC_CMR0_TCCLKS_TIMER_CLOCK1
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
    bakRA = (uint32_t)ra;

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
                     SPEAKER_CHANNEL_R,
                     1,
                     16);
    /* channel number is in the first 4 significant bits*/
    DACC->DACC_MR |= DACC_MR_TAG;
    /* disable PDC for DAC */
    DACC->DACC_PTCR = DACC_PTCR_TXTDIS;
    /* Mute */
    AudioSpeakerEnable(0);
    /* Enable DACC ISR */
    NVIC_EnableIRQ(DACC_IRQn);

}

/**
 * Buffer translation for PCM format to DAC values
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
        AudioSpeakerEnable(0);
        DACC_DisableIt(pDac, DACC_IDR_TXBUFE | DACC_IDR_ENDTX);
        pDac->DACC_PTCR = DACC_PTCR_TXTDIS;
        isDacActive = 0;
        /* Reset playback index */
        inBufferIndex = outBufferIndex = 0;
    }
    else if (1 /*sr & DACC_ISR_ENDTX*/) {

        if (numBuffersToSend) {
            /* Check the number of available buffers */
            if (numBuffersToSend > BUFFER_DELAY) {
                //printf("%d+", numBuffersToSend);
	            /* Speed up DAC freq (TC) */
	            {   uint32_t newRA = bakRA - (numBuffersToSend-DAC_DELAY);
	                if (TC0->TC_CHANNEL[0].TC_RA != newRA) {
	                    TC0->TC_CHANNEL[0].TC_RA =  newRA;
	                    TC0->TC_CHANNEL[0].TC_RC =  newRA*2;
	                }
	            }
            }
            else if (numBuffersToSend < BUFFER_DELAY) {
                //printf("%d-", numBuffersToSend);
	            /* Slow down DAC (TC) */
	            {   uint32_t newRA = bakRA + (DAC_DELAY-numBuffersToSend);
	                if (TC0->TC_CHANNEL[0].TC_RA != newRA) {
	                    TC0->TC_CHANNEL[0].TC_RA = newRA;
	                    TC0->TC_CHANNEL[0].TC_RC = newRA*2;
	                }
	            }
	        }
	        else {
	            /* Keep DAC (TC) */
	            if (TC0->TC_CHANNEL[0].TC_RA != bakRA) {
	                TC0->TC_CHANNEL[0].TC_RA = bakRA;
	                TC0->TC_CHANNEL[0].TC_RC = bakRA*2;
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
    else {
        /* Last frame remaining */
        DACC_DisableIt(pDac, DACC_IDR_ENDTX);
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
            AudioSpeakerEnable(1);

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
    AUDDFunction_Read(buffers[inBufferIndex],
                           AUDDevice_BYTESPERFRAME,
                           (TransferCallback) FrameReceived,
                           0); // No optional argument
}

/**
 * Callback invoked when data has been received on the USB.
 */
static void CdcDataReceived(uint32_t unused,
                            uint8_t status,
                            uint32_t received,
                            uint32_t remaining)
{
    /* Check that data has been received successfully */
    if (status == USBD_STATUS_SUCCESS) {

        /* Echo back */
        CDCDSerial_Write(usbSerialBuffer0, received, 0, 0);
        /* Receive next */
        CDCDSerial_Read(usbSerialBuffer0,
                        DATABUFFERSIZE,
                        (TransferCallback) CdcDataReceived,
                        0);

    }
    else {

        TRACE_WARNING( "CdcDataReceived: Transfer error\n\r");
    }
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

/*-------------------------------------------
 *      USB Device Driver callbacks
 *-------------------------------------------*/

/**
 * Invoked after the USB driver has been initialized. By default, configures
 * the UDP/UDPHS interrupt.
 */
void USBDCallbacks_Initialized(void)
{
    NVIC_EnableIRQ(UDP_IRQn);
}

/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
    CDCAUDDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(uint8_t interface,
                                                 uint8_t setting)
{
    CDCAUDDDriver_InterfaceSettingChangedHandler(interface, setting);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
    CDCAUDDDriver_RequestHandler(request);
}

/*-------------------------------------------
 *      USB Function driver callbacks
 *-------------------------------------------*/

/**
 * Invoked when the CDC ControlLineState is changed
 * \param DTR   New DTR value.
 * \param RTS   New RTS value.
 */
void CDCDSerial_ControlLineStateChanged(uint8_t DTR,
                                        uint8_t RTS)
{
    isSerialPortON = DTR;
}

/**
 *  Invoked when an audio channel get muted or unmuted. Mutes/unmutes the
 *  channel at the DAC level.
 *  \param mic      Microphone/Speaker stream changed.
 *  \param channel  Channel number that changed.
 *  \param muted    Indicates the new mute status of the channel.
 */
void AUDDFunction_MuteChanged(uint8_t mic,
                                        uint8_t channel,
                                        uint8_t muted)
{
    //printf("m%d,%d,%d ", mic, channel, muted);
    if (mic) return;

    /* Speaker Master channel */
    if (channel == AUDD_CH_Master) {

        if (muted) {

            AudioSpeakerEnable(0);
            TRACE_WARNING("MuteMaster ");
        }
        else {

            TRACE_INFO("UnmuteMaster ");
            AudioSpeakerEnable(1);
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
void AUDDFunction_StreamSettingChanged(uint8_t mic,
                                       uint8_t newSetting)
{
    /* Speaker stream */
    if (!mic) {
        if (newSetting) LED_Set(LED_BLUE);
        else            LED_Clear(LED_BLUE);
        isPlyActive = (newSetting > 0);
    }
}

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief usb_iad_cdc_aud Application entry point.
 *
 *  Starts the driver and
 *  - waits for an audio input stream to forward to the DAC.
 *  - waits for a cdc serial input to forward to cdc output (echo).
 */
int main(void)
{
    volatile uint8_t usbConn = 0;
    volatile uint8_t audioOn = 0;
    volatile uint8_t serialOn = 0;

    /* Disable watchdog */
    WDT_Disable( WDT );

    printf("-- USB CDC + Audio Device Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* If they are present, configure Vbus & Wake-up pins */
    PIO_InitializeInterrupts(0);

    /* Audio STREAM LED */
    LED_Configure(LED_BLUE);

    /* Configure USB, ADC & DACC interrupt priority:
       All interrupt to same priority so that variables accessing
       are not interrupted */
    NVIC_SetPriority(UDP_IRQn, 6);
    NVIC_SetPriority(DACC_IRQn, 6);

    /* Enable UPLL for USB */
    _ConfigureUsbClock();

    /* Configure Audio */
    _ConfigureAudio(AUDDevice_SAMPLERATE,
                   AUDDevice_NUMCHANNELS,
                   BOARD_MCK);

    /* USB audio driver initialization */
    CDCAUDDDriver_Initialize(&cdcauddDriverDescriptors);

    /* connect if needed */
    VBus_Configure();

    /* Infinite loop */
    while (1) {

        if (USBD_GetState() < USBD_STATE_CONFIGURED) {
            usbConn = 0;
            continue;
        }

        if (audioOn) {
            if(isDacActive == 0) {
                printf("audE ");
                audioOn = 0;
            }
        }
        else if (isDacActive) {
            printf("audS ");
            audioOn = 1;
        }

        if (usbConn == 0) {
            usbConn = 1;
        }

        if (isPlyActive) {
            /* Try to Start Reading the incoming audio stream */
            AUDDFunction_Read(buffers[inBufferIndex],
                              AUDDevice_BYTESPERFRAME,
                              (TransferCallback) FrameReceived,
                              0); // No optional argument
        }

        if (!serialOn && isSerialPortON) {
            printf("-I- SerialPort ON\n\r");
            /* Start receiving data on the USB */
            CDCDSerial_Read(usbSerialBuffer0,
                            DATABUFFERSIZE,
                            (TransferCallback) CdcDataReceived,
                            0);
            serialOn = 1;
        }
        else if (serialOn && !isSerialPortON) {
            printf("-I- SeriaoPort OFF\n\r");
            serialOn = 0;
        }

    }
}

