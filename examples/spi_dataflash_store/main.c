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

/**
 * \Page spi_dataflash SPI Dataflash Example
 *
 * \section Purpose
 *
 * The spi_dataflash example will help new users get familiar with SPI interface
 * on sam3s. This example gives you an AT45 Dataflash programming code so
 * that can help develop your own SPI devices applications with maximum
 * efficiency.
 *
 * You can find following information depends on your needs:
 * - A Spi low level driver performs SPI device Initializes, data transfer and
 * receive. It can be used by upper SPI driver such as AT45 %dataflash.
 * - A Dataflash driver is based on top of the corresponding Spi driver.
 * It allow user to do operations with %dataflash in a unified way.
 *
 * \section Requirements
 *unsigned char	fw []
 * This package can be used with sam3s-ek and external Data Flash connected.
 * Please connect the SPI peripheral to external board like following matching
 * pairs:
 *        - <b>SAM3S--DataFlash</b>
 *        - VCC--VCC
 *        - GND--GND
 *        - NPCS0--NSS
 *        - MISO--MISO
 *        - MOSI--MOSI
 *        - SPCK--SPCK
 *
 * \section Description
 *
 * The demonstration program tests the dataflash connected to the evaluation kit by
 * erasing and writing each one of its pages.
 *
 * \section Requirements
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">SAM-BA User Guide</a>,
 *    the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
 *    application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the terminal.
 *    \code
 *    -- SPI Dataflash Example xxx --
 *    -- xxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    -I- Initializing the SPI and AT45 drivers
 *    -I- At45 enabled
 *    -I- Waiting for a dataflash to be connected ...
 *    \endcode
 * -# As soon as a dataflash is connected, the tests will start. Eventually,
 *    the test result (pass or fail) will be output to the hyperterminal.
 * \section References
 * - spi_dataflash/main.c
 * - spi_pdc.c
 * - spi_at45.c
 * - at45d.c
 */

/*
 *
 * \file
 *
 * This file contains all the specific code for the spi_dataflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------
 */

#include "board.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"

extern	char		fw_version[];
extern	char 		fw[];
extern	unsigned int	fw_size;

/*----------------------------------------------------------------------------
 *        Internal definitions
 *----------------------------------------------------------------------------
 */

/** SPI clock frequency, in Hz.*/
#define SPCK        1000000
#define	INFOPAGE	1
#define	INFOBLOCK	0

/*----------------------------------------------------------------------------
 *        Internal variables
 *----------------------------------------------------------------------------
 */

/** SPI driver instance.*/
static Spid spid;

/** AT45 driver instance.*/
static At45 at45;

/** Pins used by the application.*/
static const Pin pins[] = { PINS_SPI, PIN_SPI_NPCS0_PA11 };

/** Page buffer.*/
static uint8_t pBuffer[2048 + 64];
unsigned int pagesize;	/* The size of an AT45 page */
unsigned int maxpage;	/* The number of pages needed to fit fw */
unsigned int maxaddr;	/* The number of bytes needed to fit all written pages */

void clear_buffer(void)
{
	memset(pBuffer, 0, pagesize);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
void header(void)
{
	/* Output example information */
	printf("-- OWL Firmware Write %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
}

void hw_init(void)
{
	const At45Desc *pDesc;
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Configure pins */
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	/* SPI and At45 driver initialization. */
	SPID_Configure(&spid, SPI, ID_SPI);
	SPID_ConfigureCS(&spid, 0, AT45_CSR(BOARD_MCK, SPCK));
	AT45_Configure(&at45, &spid, 0);

	/* Identify the At45 device */
	pDesc = 0;
	while (!pDesc) {
		pDesc = AT45_FindDevice(&at45, AT45D_GetStatus(&at45));
	}
	printf("-I- %s detected\n\r", at45.pDesc->name);
}

void param_init(void)
{
	pagesize = AT45_PageSize(&at45);
	maxpage = (fw_size + pagesize - 1) / pagesize;
	maxaddr = (INFOPAGE + maxpage) * pagesize;
}

unsigned int erase(void)
{
	unsigned int addr = 0;
	unsigned int	page,i;

	printf("Erasing!\r\n");
	for (page = 0; page < maxpage; page++) {
		if ((addr % (pagesize * 32)) == 0)
			printf("\r\n");

		printf(".");
		AT45D_Erase(&at45, addr);

		/* Verify that dwPage has been erased correctly */
		memset(pBuffer, 0, pagesize);
		AT45D_Read(&at45, pBuffer, pagesize, addr);

		for (i = 0; i < pagesize; i++) {
			if (pBuffer[i] != 0xff) {
				printf
				    ("\r\n-E- Could not erase page %u\r\n",
				     page);
				return 0;
			}
		}

		addr += pagesize;
	}
	printf("\r\n");
	return 1;
}

void	fw_write(void)
{
	unsigned int addr, page, i;
	char *pfw = fw;
	char *info;

	info	= (char *) pBuffer;

	clear_buffer();
	info += sprintf(info, "<TITLE>%s</TITLE>\r\n", fw_version);
	info += sprintf(info, "<PAGES>%u</PAGES>\r\n", maxpage);
	info += sprintf(info, "<SIZE>%u</SIZE>\r\n", fw_size);
	AT45D_Write(&at45, pBuffer, pagesize, INFOBLOCK);

	addr = 0;
	for (page = 1; page < maxpage; page++) {
		for (i = 0; i < pagesize; i++) {
			pBuffer[i] = *pfw++;
		}
		AT45D_Write(&at45, pBuffer, pagesize, addr);
		addr += pagesize;
	}
}

unsigned int fw_verify(void)
{
	unsigned int addr = 0;
	unsigned int page, i;

	AT45D_Read(&at45, pBuffer, pagesize, addr);
	pBuffer[pagesize - 1] = '\0';
	char *info;

	info	= (char *) pBuffer;
	printf("%s",info);

	addr = pagesize;
	for (page = 1; page < maxpage; page++) {
		AT45D_Read(&at45, pBuffer, pagesize, addr);
		for (i = 0; i < pagesize; i++) {
			if (pBuffer[i] != fw[addr + i]) {
				if (addr > maxaddr) {
					return 1;
				} else {
					printf
					    ("-E- Could not verify page at %u\n\r", addr + i);
					return	0;
				}
			}
		}
		addr += pagesize;
	}
	return	1;
}

/**
 * \brief Writes the contents of 'fw' to the dataflash
 */
extern int main(void)
{
	uint8_t status = 0;

	header();
	hw_init();
	if (erase()) {
		fw_write();
		if (fw_verify()) {
			status = 1;
		}
	}
	if (status) {
		printf("OWL Firmware update succeeded\r\n");
	} else {
		printf("OWL Firmware update failed\r\n");
	}
	while (1);
}
