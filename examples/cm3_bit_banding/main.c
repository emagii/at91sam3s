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
 * \page cm3_bit_banding Bit Banding for SAM CM3 Microcontrollers
 *
 * \section Purpose
 *
 * The cm3-bitbanding example will help new users get familiar with
 * bitbanding feature of SAM3 microcontrollers.
 *
 * Bit-banding a useful feature of Cortex M3 processor. The processor memory map
 * includes two bit-band regions, that occupy the lowest 1MB of the SRAM and
 * Peripheral memory regions. These bit-band regions map each word in an alias
 * region of memory to a bit in a bit-band region of memory. Accessing to the
 * alias region map to the corresponding 1MB memory region map word applies to
 * the bit that mapped.
 *
 * With bit-banding, bits modification can be simplified to just memory write,
 * so that the bit modification is easier and faster
 * (see result of BitbandingSramTest()):
 * - Old bit modification (see WRITE_BIT())
 *    -# Read word from memory
 *    -# Mask out the bit that is to be modified
 *    -# Set the expected bit to right value
 *    -# Write word back to memory
 * - Bit modification using bit-banding (see WRITE_BITBANDING())
 *    -# Calculate bit alias word address
 *    -# Write to address memory with expected value
 *
 * The example will execute following tests:
 * - Test bit accessing speed with a sram buffer
 *   (BitbandingSramTest() & testData[])
 * - Test REGISTER bit-banding accessing with LED0
 *   (BitbandingPeripheralTest())
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 * The cm3-bitbanding example will help new users get familiar with
 * bitbanding feature of SAM3 microcontrollers.
 *
 * \section Usage
 *
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
 * -# In the terminal window, the following text should appear
 *     (values depend on the board and chip used):
 *    \code
 *     -- CM3 Bit Banding Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 *
 * \section References
 * - bitbanding.h
 * - timetick.h
 * - pio.h
 * - trace.h
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Test Bit writing round */
#define NB_TEST_ROUND                   1000

/** Test data size in ARM word */
#define TEST_DATA_SIZE                  16

/**
 * \brief Normal bit write macro
 *
 * Write bit through normal bit accessing mode.
 * \param addr32 32-bit aligned byte address where the bit exists.
 * \param bit    Bit position in the DW.
 * \param val    The value that the bit is set to.
 */
#define WRITE_BIT(addr32, bit, val) do {\
        register uint32_t tmp = *(uint32_t*)(addr32); \
        tmp &= ~(1     << (bit)); \
        tmp |=  ((val) << (bit)); \
        *(uint32_t*)(addr32) = tmp; \
    } while (0);

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** data buffer for bit accessing test (32*TEST_DATA_SIZE bits) */
static uint32_t testData[TEST_DATA_SIZE];

/*----------------------------------------------------------------------------
 *        Local Functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Systick handler to manage timestamping and delay
 */
void SysTick_Handler( void )
{
    TimeTick_Increment() ;
}

/**
 * \brief Perform bitbanding tests in SRAM
 *
 * Compare writing to SRAM bits with bitbanding against normal bit access.
 * -# Record start tick via GetTickCount().
 * -# Modify bits in testData[] bit by bit for NB_TEST_ROUND times.
 * -# Record end tick via GetTickCount().
 * -# Calculate and show the speed on DEBUG output.
 *
 * \callgraph
 */
static void _BitbandingSramTest( void )
{
    uint32_t i, j, nbTests ;
    uint32_t startTick, endTick ;

    printf( "\n\r** Modify SRAM with bitbanding:\n\r" ) ;
    if ( !IS_BITBAND_SRAM_ADDR( testData ) )
    {
        TRACE_FATAL("Data Address %x not in region!!", (int)testData);
    }

    /* Bit accessing speed compare */
    printf( "-- BITs Writing performance test:\n\r" ) ;

    /* - Write to bits with normal bit accessing */
    printf( " Without bitbanding:" ) ;
    Wait(1); /* Start on tick point! */
    startTick = GetTickCount();
    for (nbTests = 0; nbTests < NB_TEST_ROUND; nbTests ++)
    {
        for (i = 0; i < TEST_DATA_SIZE; i ++)
        {
            for (j = 0; j < 32; j ++)
            {
                WRITE_BIT(&testData[i] , j, j & 0x1);
            }
        }
    }
    endTick = GetTickCount() ;
    printf( " %d kb/s\n\r", (int)(nbTests*TEST_DATA_SIZE*32/(endTick - startTick)));

    /* - Write to bits with bit banding */
    printf(" With bitbanding:");
    Wait(1); /* Start on tick point! */
    startTick = GetTickCount();
    for ( nbTests=0 ; nbTests < NB_TEST_ROUND ; nbTests++ )
    {
        for ( i=0 ; i < TEST_DATA_SIZE ; i++ )
        {
            for ( j=0 ; j < 32 ; j++ )
            {
                WRITE_BITBANDING(&testData[i] , j, j & 0x1);
            }
        }
    }
    endTick = GetTickCount();
    printf(" %d kb/s\n\r", (int)(nbTests*TEST_DATA_SIZE*32/(endTick - startTick)));
}

/**
 * \brief Perform bitbanding tests in PERIPHERAL REGs
 *
 * Blink a LED via bitbanding operations.
 * -# Configure LED0 (defined in board.h as PIN_LED_0), via PIO_Configure()
 * -# Blink LED through bitbanding operation
 *    - Access bit through BITBAND_ALIAS_ADDRESS() macro
 *    - PIO Registers used: PIO_ODSR, PIO_CODR, PIO_SODR
 *
 * \sa Pin, Pio
 *
 * \callgraph
 */
static void _BitbandingPeripheralTest( void )
{
    const Pin pinLED0 = PIN_LED_0 ;  /* LED PIO defined in board.h */
    uint8_t ucLed0Bit ;

    printf( "\n\r** Modify PIO with bitbanding:\n\r" ) ;

    /* Calculate LED0 bit */
    ucLed0Bit = (uint8_t)( log10( pinLED0.mask )/log10( 2 ) ) ;
    printf( "LED0.REG base %x .. bit %x\n\r", (int)pinLED0.pio, ucLed0Bit ) ;
    printf( "    .BITbanding base %x\n\r", (int)BITBAND_ALIAS_ADDRESS( pinLED0.pio, 0 ) ) ;

    /* Configure LED0 pin */
    PIO_Configure( &pinLED0, 1 ) ;

    /* Toggle LED every 500ms */
    while( 1 )
    {
        printf(": ");
        if ( *BITBAND_ALIAS_ADDRESS( &pinLED0.pio->PIO_ODSR, ucLed0Bit ) )
        {
            printf( " " ) ;
            *BITBAND_ALIAS_ADDRESS( &pinLED0.pio->PIO_CODR, ucLed0Bit ) = 1 ;
        }
        else
        {
            printf( "*" ) ;
            *BITBAND_ALIAS_ADDRESS( &pinLED0.pio->PIO_SODR, ucLed0Bit ) = 1 ;
        }
        printf( "\r" ) ;
        Sleep( 500 ) ;
    }
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for Bit Banding example
 *
 * The example do following:
 * -# Compare the speed for bit writing between normal bit
 *    access method and bit banding method.
 * -# Blink LED via bit banding access to PIO register.
 *
 * \callgraph
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "-- CM3 Bit Banding Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Start System Tick */
    TimeTick_Configure( BOARD_MCK ) ;

    /* 1. SRAM Bitbanding speed test */
    _BitbandingSramTest() ;

    /* 2. Bitbanding PIO test (LED0), access register with bit banding */
    _BitbandingPeripheralTest() ;

    return 0 ;
}

