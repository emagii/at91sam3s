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
 * \page sam3s_pirrd_board_desc SAM3S-PIRRD - Board Description
 *
 * \section Purpose
 *
 * A file is dedicated to descibe the SAM3S-PIRRD board.
 *
 * \section Contents
 *
 *  - For SAM3S-PIRRD information, see \subpage sam3s_pirrd_board_info.
 *  - For operating frequency information, see \subpage sam3s_pirrd_opfreq.
 *  - For using portable PIO definitions, see \subpage sam3s_pirrd_piodef.
 *  - For on-board memories, see \subpage sam3s_pirrd_mem.
 *  - Several USB definitions are included here, see \subpage sam3s_pirrd_usb.
 *  - For External components, see \subpage sam3s_pirrd_extcomp.
 *  - For Individual chip definition, see \subpage sam3s_pirrd_chipdef.
 *
 * To get more software details and the full list of parameters related to the
 * SAM3S-PIRRD board configuration, please have a look at the source file:
 * \ref board.h\n
 *
 * \section Usage
 *
 *  - The code for booting the board is provided by board_cstartup_xxx.c and
 *    board_lowlevel.c.
 *  - For using board PIOs, board characteristics (clock, etc.) and external
 *    components, see board.h.
 *  - For manipulating memories, see board_memories.h.
 *
 * This file can be used as a template and modified to fit a custom board, with
 * specific PIOs usage or memory connections.
 */

/**
 *  \file board.h
 *
 *  Definition of SAM3S-PIRRD characteristics, SAM3S-dependant PIOs and
 *  external components interfacing.
 */

#ifndef _BOARD_
#define _BOARD_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "chip.h"

#include "include/ads7843.h"
#include "include/bitbanding.h"
#include "include/bmp.h"
#include "include/board_lowlevel.h"
#include "include/board_memories.h"
#include "include/clock.h"
#include "include/hamming.h"
#include "include/ili9325.h"
#include "include/lcdd.h"
#include "include/lcd_color.h"
#include "include/lcd_draw.h"
#include "include/lcd_font.h"
#include "include/lcd_font10x14.h"
#include "include/lcd_gimp_image.h"
#include "include/led.h"
#include "include/math.h"
#include "include/ov7740.h"
#include "include/omnivision_ov7740.h"
#include "include/rand.h"
#include "include/timetick.h"
#include "include/tsd.h"
#include "include/tsd_ads7843.h"
#include "include/tsd_com.h"
#include "include/uart_console.h"
#include "include/wav.h"
#include "include/wm8731.h"

/**
 * Libc porting layers
 */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68/2010q1-188 */
#    include "include/syscalls.h" /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_pirrd_board_info "SAM3S-PIRRD - Board informations"
 * This page lists several definition related to the board description.
 *
 * \section Definitions
 * - \ref BOARD_NAME
 */

/** Name of the board */
#define BOARD_NAME "SAM3S-PIRRD"
/** Board definition */
#define sam3s_pirrd
/** Family definition (already defined) */
#define sam3s
/** Core definition */
#define cortexm3

#define BOARD_REV_A

/*----------------------------------------------------------------------------*/
/**
 *  \page sam3s_pirrd_opfreq "SAM3S-PIRRD - Operating frequencies"
 *  This page lists several definition related to the board operating frequency
 *  (when using the initialization done by board_lowlevel.c).
 *
 *  \section Definitions
 *  - \ref BOARD_MAINOSC
 *  - \ref BOARD_MCK
 */

/** Frequency of the board main oscillator */
#define BOARD_MAINOSC           12000000

/** Master clock frequency (when using board_lowlevel.c) */
#define BOARD_MCK               64000000

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_pirrd_piodef "SAM3S-PIRRD - PIO definitions"
 * This pages lists all the pio definitions contained in board.h. The constants
 * are named using the following convention: PIN_* for a constant which defines
 * a single Pin instance (but may include several PIOs sharing the same
 * controller), and PINS_* for a list of Pin instances.
 *
 * ADC
 * - \ref PIN_ADC0_AD0
 * - \ref PIN_ADC0_AD4
 * - \ref PIN_ADC0_AD5
 * - \ref PINS_ADC
 *
 * UART
 * - \ref PINS_UART
 *
 * EBI
 * - \ref PIN_EBI_DATA_BUS
 * - \ref PIN_EBI_NRD
 * - \ref PIN_EBI_NWE
 * - \ref PIN_EBI_NCS0
 * - \ref PIN_EBI_PSRAM_ADDR_BUS
 * - \ref PIN_EBI_PSRAM_NBS
 * - \ref PIN_EBI_A1
 * - \ref PIN_EBI_NCS1
 * - \ref PIN_EBI_LCD_RS
 *
 * LEDs
 * - \ref PIN_LED_0
 * - \ref PINS_LEDS
 *
 * MCI
 * - \ref PINS_MCI
 *
 * Push buttons
 * - \ref PIN_PUSHBUTTON_1
 * - \ref PINS_PUSHBUTTONS
 * - \ref PUSHBUTTON_BP1
 *
 * SPI
 * - \ref PIN_SPI_MISO
 * - \ref PIN_SPI_MOSI
 * - \ref PIN_SPI_SPCK
 * - \ref PINS_SPI
 * - \ref PIN_SPI_NPCS0_PA11
 *
 * PCK0
 * - \ref PIN_PCK0
 *
 * PIO PARALLEL CAPTURE
 * - \ref PIN_PIODCEN1
 * - \ref PIN_PIODCEN2
 *
 * TWI
 * - \ref TWI_V3XX
 * - \ref PIN_TWI_TWD0
 * - \ref PIN_TWI_TWCK0
 * - \ref PINS_TWI0
 *
 * USB
 * - \ref PIN_USB_VBUS
 *
 */

/** ADC_AD0 pin definition. */
#define PIN_ADC0_AD0 {PIO_PA17, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD4 pin definition. */
#define PIN_ADC0_AD4 {1 << 0, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}
/** ADC_AD5 pin definition. */
#define PIN_ADC0_AD5 {1 << 1, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}

/** Pins ADC */
#define PINS_ADC PIN_ADC0_AD0, PIN_ADC0_AD4, PIN_ADC0_AD5

/** Startup time max, return from Idle mode (in µs) */
#define ADC_STARTUP_TIME_MAX       15
/** Track and hold Acquisition Time min (in ns) */
#define ADC_TRACK_HOLD_TIME_MIN  1200
/** ADC clock frequence */
#define BOARD_ADC_FREQ     (6000000)

/** UART pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART  {0x00000600, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/** EBI Data Bus pins */
#define PIN_EBI_DATA_BUS            {0xFF,    PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI NRD pin */
#define PIN_EBI_NRD                 {1 << 11, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI NWE pin */
#define PIN_EBI_NWE                 {1 << 8,  PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI NCS0 pin */
#define PIN_EBI_NCS0                {1 << 14, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI pins for PSRAM address bus */
#define PIN_EBI_PSRAM_ADDR_PIOC      {0xFFFC0000, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP} //A0-A13
#define PIN_EBI_PSRAM_ADDR_PIOA      {0x001C0003, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP} // A14-A18
/** EBI pins for PSRAM NBS pins */
#define PIN_EBI_PSRAM_NBS           {1 << 7, PIOB, ID_PIOB, PIO_PERIPH_B, PIO_PULLUP}, \
                                    {1 << 15, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI A1 pin */
#define PIN_EBI_A1                  {1 << 19, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** EBI NCS2 pin */
#define PIN_EBI_NCS2                {PIO_PA22, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP} /* LCD CS pin */
/** EBI pin for LCD RS */
#define PIN_EBI_LCD_RS              {1 << 19, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP} /* LCD RS pin */

#define LED_BLUE      0

/** LED #0 pin definition (BLUE). */
#define PIN_LED_0   {PIO_PC12, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}

/** List of all LEDs definitions. */
#define PINS_LEDS   PIN_LED_0

/** Push button #0 definition. Attributes = pull-up + debounce + interrupt on rising edge. */
#define PIN_PUSHBUTTON_1    {PIO_PA2, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE}
/** List of all push button definitions. */
#define PINS_PUSHBUTTONS    PIN_PUSHBUTTON_1

/** Push button #1 index. */
#define PUSHBUTTON_BP1   0

/** SPI MISO pin definition. */
#define PIN_SPI_MISO    {PIO_PA12A_MISO, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI MOSI pin definition. */
#define PIN_SPI_MOSI    {PIO_PA13A_MOSI, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI SPCK pin definition. */
#define PIN_SPI_SPCK    {PIO_PA14A_SPCK, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** SPI chip select pin definition. */
#define PIN_SPI_NPCS0_PA11  {PIO_PA11A_NPCS0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** List of SPI pin definitions (MISO, MOSI & SPCK). */
#define PINS_SPI        PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SPCK

/** PCK0 */
#define PIN_PCK0        {PIO_PB13B_PCK0, PIOB, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT}

/** PIO PARALLEL CAPTURE */
/** Parallel Capture Mode Data Enable1 */
#define PIN_PIODCEN1    PIO_PA15
/** Parallel Capture Mode Data Enable2 */
#define PIN_PIODCEN2    PIO_PA16

/** TWI ver 3.xx */
#define TWI_V3XX
/** TWI0 data pin */
#define PIN_TWI_TWD0    {PIO_PA3A_TWD0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 clock pin */
#define PIN_TWI_TWCK0   {PIO_PA4A_TWCK0, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
/** TWI0 pins */
#define PINS_TWI0       PIN_TWI_TWD0, PIN_TWI_TWCK0

/** USB VBus monitoring pin definition. */
#define PIN_USB_VBUS    {PIO_PC17, PIOC, ID_PIOC, PIO_INPUT, PIO_PULLUP}

/** PIR sensor pin definition */
/** pin for ADVREF */
#define PIN_PIR_ADVREF  { PIO_PC10, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT} /* PIR_ADVREF */
/** List of PIR sensor definitions. */
#define PINS_PIR        PIN_PIR_ADVREF, PIN_ADC0_AD0

/** Image sensor pin definition */
/** pin for OV_VDD */
#define PIN_OV_VDD      { PIO_PC16, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT} /* OV_VDD */
#define PIN_OV_PWDN     { PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_OV_RST      { PIO_PC15, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_OV_FSIN     { PIO_PA21, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}

/** List of Image sensor definitions. */
//#define PINS_OV         PIN_OV_VDD, PIN_OV_PWDN, PIN_PCK0
#define PINS_OV         PIN_OV_VDD, PIN_PCK0, PIN_OV_RST

/** HSYNC definition. Attributes = pull-up + interrupt on rising edge. */
#define PIN_OV_HSYNC    { PIO_PA16, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP | PIO_IT_RISE_EDGE }
/** VSYNC definition. Attributes = pull-up + interrupt on falling edge. */
#define PIN_OV_VSYNC    { PIO_PA15, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP | PIO_IT_RISE_EDGE }

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_pirrd_usb "SAM3S-PIRRD - USB device"
 *
 * \section Definitions
 * - \ref BOARD_USB_BMATTRIBUTES
 * - \ref CHIP_USB_UDP
 * - \ref CHIP_USB_PULLUP_INTERNAL
 * - \ref CHIP_USB_NUMENDPOINTS
 * - \ref CHIP_USB_ENDPOINTS_MAXPACKETSIZE
 * - \ref CHIP_USB_ENDPOINTS_BANKS
 */

/** USB attributes configuration descriptor (bus or self powered, remote wakeup) */
#define BOARD_USB_BMATTRIBUTES              USBConfigurationDescriptor_SELFPOWERED_RWAKEUP

/** Indicates chip has an UDP Full Speed. */
#define CHIP_USB_UDP

/** Indicates chip has an internal pull-up. */
#define CHIP_USB_PULLUP_INTERNAL

/** Number of USB endpoints */
#define CHIP_USB_NUMENDPOINTS 8

/** Endpoints max paxcket size */
#define CHIP_USB_ENDPOINTS_MAXPACKETSIZE(i) \
   ((i == 0) ? 64 : \
   ((i == 1) ? 64 : \
   ((i == 2) ? 64 : \
   ((i == 3) ? 64 : \
   ((i == 4) ? 512 : \
   ((i == 5) ? 512 : \
   ((i == 6) ? 64 : \
   ((i == 7) ? 64 : 0 ))))))))

/** Endpoints Number of Bank */
#define CHIP_USB_ENDPOINTS_BANKS(i) \
   ((i == 0) ? 1 : \
   ((i == 1) ? 2 : \
   ((i == 2) ? 2 : \
   ((i == 3) ? 1 : \
   ((i == 4) ? 2 : \
   ((i == 5) ? 2 : \
   ((i == 6) ? 2 : \
   ((i == 7) ? 2 : 0 ))))))))

/*----------------------------------------------------------------------------*/
/**
 * \page sam3s_pirrd_extcomp "SAM3S-PIRRD - External components"
 * This page lists the definitions related to external on-board components
 * located in the board.h file for the SAM3S-PIRRD.
 *
 * SD Card
 * - \ref BOARD_SD_PINS
 * - \ref BOARD_SD_PIN_CD
 *
 * LCD
 * - \ref BOARD_LCD_ILI9325
 * - \ref BOARD_LCD_PINS
 * - \ref BOARD_BACKLIGHT_PIN
 * - \ref BOARD_LCD_BASE
 * - \ref BOARD_LCD_RS
 * - \ref BOARD_LCD_WIDTH
 * - \ref BOARD_LCD_HEIGHT
 *
 * TouchScreen
 * - \ref BOARD_TSC_ADS7843
 * - \ref PIN_TCS_IRQ
 * - \ref PIN_TCS_BUSY
 * - \ref BOARD_TSC_SPI_BASE
 * - \ref BOARD_TSC_SPI_ID
 * - \ref BOARD_TSC_SPI_PINS
 * - \ref BOARD_TSC_NPCS
 * - \ref BOARD_TSC_NPCS_PIN
 */

/** MCI pins that shall be configured to access the SD card. */
#define BOARD_SD_PINS               PINS_MCI
/** MCI Card Detect pin. */
#define BOARD_SD_PIN_CD             PIN_MCI_CD

/** Indicates board has an ILI9325 external component to manage LCD. */
#define BOARD_LCD_ILI9325

/** LCD pins definition. */
#define BOARD_LCD_PINS              PIN_EBI_DATA_BUS, PIN_EBI_NRD, PIN_EBI_NWE, PIN_EBI_NCS2, PIN_EBI_LCD_RS
/** Backlight pin definition. */
#define BOARD_BACKLIGHT_PIN         { PIO_PC13, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT }
/** Define ILI9325 base address. */
#define BOARD_LCD_BASE              0x62000000
/** Define ILI9325 register select signal. */
#define BOARD_LCD_RS                (1 << 1)
/** Display width in pixels. */
#define BOARD_LCD_WIDTH             240
/** Display height in pixels. */
#define BOARD_LCD_HEIGHT            320

/** Touchscreen controller IRQ pin definition. */
#ifdef BOARD_REV_A
#define PIN_TSC_IRQ     { PIO_PB3, PIOB, ID_PIOB, PIO_INPUT, PIO_PULLUP }
/** Touchscreen controller Busy pin definition. */
#define PIN_TSC_BUSY    { PIO_PB2, PIOB, ID_PIOB, PIO_INPUT, PIO_PULLUP }
#endif


/** Base address of SPI peripheral connected to the touchscreen controller. */
#define BOARD_TSC_SPI_BASE         SPI
/** Identifier of SPI peripheral connected to the touchscreen controller. */
#define BOARD_TSC_SPI_ID           ID_SPI
/** Pins of the SPI peripheral connected to the touchscreen controller. */
#define BOARD_TSC_SPI_PINS         PINS_SPI
/** Chip select connected to the touchscreen controller. */
#define BOARD_TSC_NPCS             0
/** Chip select pin connected to the touchscreen controller. */
#define BOARD_TSC_NPCS_PIN         PIN_SPI_NPCS0_PA11

/** PSRAM pins */
#define PIN_PSRAM_VCC              { PIO_PC9, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT }
#define BOARD_PSRAM_PINS           PIN_EBI_DATA_BUS, PIN_EBI_NCS0, PIN_EBI_NRD, PIN_EBI_NWE, PIN_EBI_PSRAM_NBS, PIN_EBI_A1, PIN_PSRAM_VCC, PIN_EBI_PSRAM_ADDR_PIOA, PIN_EBI_PSRAM_ADDR_PIOC
/** Define PSRAM base address. */
#define BOARD_PSRAM_BASE           ((void*)0x60000000)
/** Define PSRAM length. */
#define BOARD_PSRAM_LENGTH         ((uint32_t)0x00100000)

#endif /* #ifndef _BOARD_ */

