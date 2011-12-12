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

/** \file
 * \addtogroup usbd_audio_speaker
 *@{
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <include/USBD_Config.h>

#include <HIDAUDDDriver.h>
#include <HIDDKeyboard.h>
#include <AUDDFunction.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/** Address of the HID interrupt IN endpoint. */
#define HIDD_Descriptors_INTERRUPTIN                1
/** Address of the HID interrupt OUT endpoint. */
#define HIDD_Descriptors_INTERRUPTOUT               2

/** Audio data out endpoint number */
#define AUDD_Descriptors_DATAOUT                    0x05

/** \addtogroup usbd_audio_id USB Device Audio Speaker Codes
 *      @{
 * This section lists the device IDs and release number of the USB Audio
 * Speaker device.
 * - \ref AUDDSpeakerDriverDescriptors_VENDORID
 * - \ref AUDDSpeakerDriverDescriptors_PRODUCTID
 * - \ref AUDDSpeakerDriverDescriptors_RELEASE
 */
/** Device vendor ID. */
#define AUDDSpeakerDriverDescriptors_VENDORID            0x03EB
/** Device product ID. */
#define AUDDSpeakerDriverDescriptors_PRODUCTID           0x6134
/** Device release number in BCD format. */
#define AUDDSpeakerDriverDescriptors_RELEASE             0x0100
/**     @}*/

/*----------------------------------------------------------------------------
 *         Internal types
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 *         Exported variables
 *----------------------------------------------------------------------------*/

/** Device descriptor for a USB audio speaker driver. */
const USBDeviceDescriptor deviceDescriptor = {

    sizeof(USBDeviceDescriptor),
    USBGenericDescriptor_DEVICE,
    USBDeviceDescriptor_USB2_00,
    AUDDeviceDescriptor_CLASS,
    AUDDeviceDescriptor_SUBCLASS,
    AUDDeviceDescriptor_PROTOCOL,
    CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
    AUDDSpeakerDriverDescriptors_VENDORID,
    AUDDSpeakerDriverDescriptors_PRODUCTID,
    AUDDSpeakerDriverDescriptors_RELEASE,
    1, /* Manufacturer string descriptor index */
    2, /* Product string descriptor index */
    3, /* Index of serial number string descriptor */
    1  /* One possible configuration */
};

/** Configuration descriptors for a USB HID(keyboard) + Audio(speaker) driver. */
const HidAuddDriverConfigurationDescriptors fsConfigurationDescriptors = {

    /* Configuration descriptor */
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_CONFIGURATION,
        sizeof(HidAuddDriverConfigurationDescriptors),
        HIDAUDDDriverDescriptors_NUMINTERFACE, /* 3 interfaces */
        1, /* This is configuration #1 */
        0, /* No string descriptor */
        USBD_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
    /* -- HID */
    /* Interface descriptor */
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        HIDAUDDDriverDescriptors_HID_INTERFACE,
        0, // This is alternate setting #0
        2, // Two endpoints used
        HIDInterfaceDescriptor_CLASS,
        HIDInterfaceDescriptor_SUBCLASS_NONE,
        HIDInterfaceDescriptor_PROTOCOL_NONE,
        0  // No associated string descriptor
    },
    /* HID descriptor */
    {
        sizeof(HIDDescriptor1),
        HIDGenericDescriptor_HID,
        HIDDescriptor_HID1_11,
        0, // Device is not localized, no country code
        1, // One HID-specific descriptor (apart from this one)
        HIDGenericDescriptor_REPORT,
        HIDDKeyboard_REPORTDESCRIPTORSIZE
    },
    /* Interrupt IN endpoint descriptor */
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(
            USBEndpointDescriptor_IN,
            HIDD_Descriptors_INTERRUPTIN),
        USBEndpointDescriptor_INTERRUPT,
        sizeof(HIDDKeyboardInputReport),
        HIDDKeyboardDescriptors_INTERRUPTIN_POLLING_FS
    },
    /* Interrupt OUT endpoint descriptor */
    {
        sizeof(USBEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(
            USBEndpointDescriptor_OUT,
            HIDD_Descriptors_INTERRUPTOUT),
        USBEndpointDescriptor_INTERRUPT,
        sizeof(HIDDKeyboardOutputReport),
        HIDDKeyboardDescriptors_INTERRUPTOUT_POLLING_FS
    },

    /* -- AUDIO */
    /* IAD for AUDIO function */
    {
        sizeof(USBInterfaceAssociationDescriptor),
        USBGenericDescriptor_INTERFACEASSOCIATION,
        HIDAUDDDriverDescriptors_AUD_INTERFACE,
        2,
        AUDControlInterfaceDescriptor_CLASS,
        AUDControlInterfaceDescriptor_SUBCLASS,
        AUDControlInterfaceDescriptor_PROTOCOL,
        0  // No string descriptor for this interface
    },
    /* Audio control interface standard descriptor */
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        HIDAUDDDriverDescriptors_AUD_INTERFACE,
        0, /* This is alternate setting #0 */
        0, /* This interface uses no endpoint */
        AUDControlInterfaceDescriptor_CLASS,
        AUDControlInterfaceDescriptor_SUBCLASS,
        AUDControlInterfaceDescriptor_PROTOCOL,
        0  /* No string descriptor */
    },
    /* Audio control interface descriptors */
    {
        /* Header descriptor */
        {
            {
                sizeof(AUDHeaderDescriptor1),
                AUDGenericDescriptor_INTERFACE,
                AUDGenericDescriptor_HEADER,
                AUDHeaderDescriptor_AUD1_00,
                sizeof(AUDDSpeakerDriverAudioControlDescriptors),
                1 /* One streaming interface */
            },
            HIDAUDDDriverDescriptors_AUD_INTERFACE + 1
        },
        /* Input terminal descriptor */
        {
            sizeof(AUDInputTerminalDescriptor),
            AUDGenericDescriptor_INTERFACE,
            AUDGenericDescriptor_INPUTTERMINAL,
            AUDDFunction_INPUTTERMINAL,
            AUDInputTerminalDescriptor_USBSTREAMING,
            AUDDFunction_OUTPUTTERMINAL,
            AUDDevice_NUMCHANNELS, /* L,R */
            AUDInputTerminalDescriptor_LEFTFRONT
            | AUDInputTerminalDescriptor_RIGHTFRONT,
            0, /* No string descriptor for channels */
            0  /* No string descriptor for input terminal */
        },
        /* Output terminal descriptor */
        {
            sizeof(AUDOutputTerminalDescriptor),
            AUDGenericDescriptor_INTERFACE,
            AUDGenericDescriptor_OUTPUTTERMINAL,
            AUDDFunction_OUTPUTTERMINAL,
            AUDOutputTerminalDescriptor_SPEAKER,
            AUDDFunction_INPUTTERMINAL,
            AUDDFunction_FEATUREUNIT,
            0 /* No string descriptor */
        },
        /* Feature unit descriptor */
        {
            {
                sizeof(AUDFeatureUnitDescriptor3),
                AUDGenericDescriptor_INTERFACE,
                AUDGenericDescriptor_FEATUREUNIT,
                AUDDFunction_FEATUREUNIT,
                AUDDFunction_INPUTTERMINAL,
                1, /* 1 byte per channel for controls */
            },
            {
                AUDFeatureUnitDescriptor_MUTE, /* Master channel controls */
                0, /* Right channel controls */
                0  /* Left channel controls */
            },
            0 /* No string descriptor */
        }
    },
    /* Audio streaming interface with 0 endpoints */
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        HIDAUDDDriverDescriptors_AUD_INTERFACE + 1,
        0, /* This is alternate setting #0 */
        0, /* This interface uses no endpoints */
        AUDStreamingInterfaceDescriptor_CLASS,
        AUDStreamingInterfaceDescriptor_SUBCLASS,
        AUDStreamingInterfaceDescriptor_PROTOCOL,
        0  /* No string descriptor */
    },
    /* Audio streaming interface with data endpoint */
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        HIDAUDDDriverDescriptors_AUD_INTERFACE + 1,
        1, /* This is alternate setting #1 */
        1, /* This interface uses 1 endpoint */
        AUDStreamingInterfaceDescriptor_CLASS,
        AUDStreamingInterfaceDescriptor_SUBCLASS,
        AUDStreamingInterfaceDescriptor_PROTOCOL,
        0  /* No string descriptor */
    },
    /* Audio streaming class-specific descriptor */
    {
        sizeof(AUDStreamingInterfaceDescriptor),
        AUDGenericDescriptor_INTERFACE,
        AUDStreamingInterfaceDescriptor_GENERAL,
        AUDDFunction_INPUTTERMINAL,
        0, /* No internal delay because of data path */
        AUDFormatTypeOneDescriptor_PCM
    },
    /* Format type I descriptor */
    {
        {
            sizeof(AUDFormatTypeOneDescriptor1),
            AUDGenericDescriptor_INTERFACE,
            AUDStreamingInterfaceDescriptor_FORMATTYPE,
            AUDFormatTypeOneDescriptor_FORMATTYPEONE,
            AUDDevice_NUMCHANNELS,
            AUDDevice_BYTESPERSAMPLE,
            AUDDevice_BYTESPERSAMPLE*8,
            1 /* One discrete frequency supported */
        },
        {
            AUDDevice_SAMPLERATE & 0xFF,
            (AUDDevice_SAMPLERATE >> 8) & 0xFF,
            (AUDDevice_SAMPLERATE >> 16) & 0xFF
        }
    },
    /* Audio streaming endpoint standard descriptor */
    {
        sizeof(AUDEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(
            USBEndpointDescriptor_OUT, AUDD_Descriptors_DATAOUT),
        USBEndpointDescriptor_ISOCHRONOUS,
        AUDDevice_BYTESPERFRAME,
        AUDDFunction_FS_INTERVAL, /* Polling interval = 1 ms */
        0, /* This is not a synchronization endpoint */
        0  /* No associated synchronization endpoint */
    },
    /* Audio streaming endpoint class-specific descriptor */
    {
        sizeof(AUDDataEndpointDescriptor),
        AUDGenericDescriptor_ENDPOINT,
        AUDDataEndpointDescriptor_SUBTYPE,
        0, /* No attributes */
        0, /* Endpoint is not synchronized */
        0  /* Endpoint is not synchronized */
    }
};

/** String descriptor with the supported languages. */
const unsigned char languageIdDescriptor[] = {

    USBStringDescriptor_LENGTH(1),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_ENGLISH_US
};

/** Manufacturer name. */
const unsigned char manufacturerDescriptor[] = {

    USBStringDescriptor_LENGTH(5),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('A'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('m'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('l')
};

/** Product name. */
const unsigned char productDescriptor[] = {

    USBStringDescriptor_LENGTH(14),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('K'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('y'),
    USBStringDescriptor_UNICODE('p'),
    USBStringDescriptor_UNICODE('a'),
    USBStringDescriptor_UNICODE('d'),
    USBStringDescriptor_UNICODE('&'),
    USBStringDescriptor_UNICODE('s'),
    USBStringDescriptor_UNICODE('p'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('a'),
    USBStringDescriptor_UNICODE('k'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('r')
};

/** Product serial number. */
const unsigned char serialNumberDescriptor[] = {

    USBStringDescriptor_LENGTH(4),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('0'),
    USBStringDescriptor_UNICODE('1'),
    USBStringDescriptor_UNICODE('2'),
    USBStringDescriptor_UNICODE('3')
};

/** Array of pointers to the four string descriptors. */
const unsigned char *stringDescriptors[] = {

    languageIdDescriptor,
    manufacturerDescriptor,
    productDescriptor,
    serialNumberDescriptor,
};

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/** List of descriptors required by an USB audio speaker device driver. */
const USBDDriverDescriptors hidauddDriverDescriptors = {

    &deviceDescriptor,
    (const USBConfigurationDescriptor *) &fsConfigurationDescriptors,
    0, 0, 0, 0, 0, 0,
    stringDescriptors,
    4 /* Number of string descriptors */
};

/**@}*/

