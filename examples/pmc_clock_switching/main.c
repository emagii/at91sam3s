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
 * \page pmc_clock_switching PMC Clock Switching Example.
 *
 * \section Purpose
 * This example shows how to switch from a clock to another (PLLA, PLLB, SLCK, MAINCK)
 * or change divider.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * Upon startup, the program configure pios for UART,PCK and botton. The baud rate of
 * UART is configured as 2400 bps. The application does some printf with the current
 * configuration (except 32KHz slow clock ) and waits for button pressed to switch to
 * next configuration. PCK1 Outputs can be selected from the clocks provided by the
 * clock (PLLA, PLLB, SLCK, MAINCK) and driven on the pin PCK1 on PA17 (Peripheral B).
 * After the clock switchs, the PCK1 output signal can be measured by scope compare with
 * the clock configuration.
 *
 * <ul>
 * <li> The Clock Generator integrates a 32,768 Hz low-power oscillator.
 * In order to use this oscillator, the XIN32/PA7 and XOUT32/P8 pins must be connected
 * to a 32,768 Hz crystal. The user can select the crystal oscillator to be the source of
 * the slow clock, as it provides a more accurate frequency. The command is made by
 * function _PmcEnableExternal32K().</li>
 * <li> MAINCK is the output of the Main Clock Oscillator selection: either the Crystal
 * Oscillator or 4/8/12 MHz Fast RC Oscillator. The user can select the output frequency
 * of the Fast RC Oscillator: either 4 MHz, 8 MHz or 12 MHz are available. It can be done
 * through function _PmcMainClockSwitchFastRc(). The user can also select the 3 to 20 MHz Crystal
 * oscillator to be the source of MAINCK, as it provides a more accurate frequency.
 * The function _PmcMainClockSwitchMainOsc() show how to enable the oscillator. </li>
 * <li> The PMC features one Divider/PLL Blocks that permit a wide range of frequencies to be
 * selected on either the master clock, the processor clock or the programmable clock outputs.
 * Function _PmcConfigurePllClock() descripts the deatil how to configure PLLA/B.</li>
 * </ul>
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
 *   - 2400 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- Pmc clock Switching Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Press one of the keys listed in the menu to perform the corresponding action.
 *
 * \section References
 * - pmc_clock_switching/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the pmc clock switching example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define PLL_A            0           /* PLL A */
#define PLL_B            1           /* PLL B */

/* Define clock timeout */
#define CLOCK_TIMEOUT    0xFFFFFFFF

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pin PCK1 (PA17 Peripheral B) */
const Pin pinPCK = PIN_PCK1 ;

/** Pushbutton pin instance. */
const Pin pinPB = PIN_PUSHBUTTON_1 ;

/** Mutual semaphore. */
static volatile uint8_t _ucWaitButton = 0 ;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Handler for button rising edge interrupt.
 */
static void _Button_Handler( const Pin *pin )
{
    if ( pin == &pinPB )
    {
        _ucWaitButton = 0 ;
    }
}

/**
 *  \brief Configure the Pushbutton
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void _ConfigureButton( void )
{
    /* Configure pio as inputs. */
    PIO_Configure( &pinPB, 1 ) ;
    /* Adjust pio debounce filter patameters, uses 10 Hz filter. */
    PIO_SetDebounceFilter( &pinPB, 10 ) ;
    /* Initialize pios interrupt handlers, see PIO definition in board.h. */
    PIO_ConfigureIt( &pinPB, _Button_Handler ) ; /* Interrupt on rising edge  */
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ( (IRQn_Type)pinPB.id ) ;
    /* Enable PIO line interrupts. */
    PIO_EnableIt( &pinPB ) ;
}

/**
 * \brief Configure UART with given master clock, and Configure PCK with given
 *  divider source of master clock and prescaler.
 *
 * \param css  The master clock divider source.
 * \param pres Master Clock prescaler.
 * \param masterClk frequency of the master clock (in Hz).
 */
static void _ConfigureUartAndPck( uint32_t dwClockSource, uint32_t dwPrescaler, uint32_t dwMasterClock )
{
    /* Configures an USART Baudrate 2400 bps (except slow clock)*/
    if ( dwMasterClock > 32768 )
    {
        UART_Configure( 2400, dwMasterClock ) ;
    }

    /* Programmable clock 1 output disabled */
    PMC->PMC_SCDR = PMC_SCER_PCK1 ;
    /* Configure PMC Programmable Clock */
    PMC->PMC_PCK[1] = dwClockSource | dwPrescaler ;
    /* Enable PCK1 output */
    PMC->PMC_SCER = PMC_SCER_PCK1 ;
    /* Wait for the PCKRDY1 bit to be set in the PMC_SR register */
    while ( (PMC->PMC_SR & PMC_SR_PCKRDY1) == 0 ) ;
}

/**
 * \brief Disable Main XTAL Oscillator.
 */
static void _PmcDisableMainXTAL( void )
{
    /* Disable Main XTAL Oscillator */
    PMC->CKGR_MOR = CKGR_MOR_KEY( 0x37 ) | CKGR_MOR_MOSCRCEN ;
}


/**
 * \brief Enable fast RC oscillator.
 *
 * \param dwRC  fast RC oscillator(4/8/12Mhz).
 */
static void _PmcEnableFastRC( uint32_t dwRC )
{
    uint32_t dwTimeout ;

    /* Enable Fast RC oscillator but DO NOT switch to RC now . Keep MOSCSEL to 1 */
    PMC->CKGR_MOR = CKGR_MOR_KEY( 0x37 ) | CKGR_MOR_MOSCSEL | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCRCEN | dwRC ;

    /* Wait the Fast RC to stabilize */
    for ( dwTimeout=0 ; !(PMC->PMC_SR & PMC_SR_MOSCRCS) && (dwTimeout++ < CLOCK_TIMEOUT) ; ) ;
}

/**
 * \brief Switch main clock to fast RC oscillator.
 *
 * \param dwRC  fast RC oscillator(4/8/12Mhz).
 */
static void _PmcMainClockSwitchFastRC( void )
{
    uint32_t dwTimeout ;

    /* Switch to Fast RC */
    PMC->CKGR_MOR = CKGR_MOR_KEY( 0x37 ) | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCRCEN ;

    /* Wait for Main Oscillator Selection Status bit MOSCSELS */
    for ( dwTimeout=0 ; !(PMC->PMC_SR & PMC_SR_MOSCSELS) && (dwTimeout++ < CLOCK_TIMEOUT) ; ) ;

    /* Disable Main XTAL Oscillator */
    _PmcDisableMainXTAL() ;
}

/**
 * \brief Switch main clock to main 3-20Mhz oscillator.
 *
 */
static void _PmcMainClockSwitchMainOsc( void )
{
    uint32_t dwTimeout ;

    /* Enable Main XTAL Oscillator */
    PMC->CKGR_MOR = CKGR_MOR_KEY( 0x37 ) | CKGR_MOR_MOSCXTST( 0x8 ) | CKGR_MOR_MOSCSEL | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN ;

   /* Wait for Main Oscillator Selection Status bit MOSCSELS */
    for ( dwTimeout=0 ; !(PMC->PMC_SR & PMC_SR_MOSCSELS) && (dwTimeout++ < CLOCK_TIMEOUT) ; ) ;
}

/**
 * \brief Select the crystal oscillator to be the source of the slow clock.
 */
static void _PmcEnableExternal32K( void )
{
    uint32_t timeout = 0;

    /* Select external slow clock */
    if ( (SUPC->SUPC_SR & SUPC_SR_OSCSEL) != SUPC_SR_OSCSEL_CRYST )
    {
        /* writing the Supply Controller Control register (SUPC_CR) with the XTALSEL bit at 1*/
        SUPC->SUPC_CR = SUPC_CR_KEY( 0xA5u ) | SUPC_CR_XTALSEL_CRYSTAL_SEL ;

        /*The OSCSEL bit of the Supply Controller Status Register (SUPC_SR) allows knowing when the switch sequence is done. */
        while ( !(SUPC->SUPC_SR & SUPC_SR_OSCSEL_CRYST) && (timeout++ < CLOCK_TIMEOUT) ) ;
    }
}

/**
 * \brief Configure PLL(PLLA,PLLB) clock by giving MUL and DIV.
 *
 * \param mul  PLL multiplier factor..
 * \param div  PLL divider factor.
 */
static void _PmcConfigurePllClock( uint8_t pllab, uint32_t dwMul, uint32_t dwDiv )
{
    uint32_t dwTimeout=0 ;

    if ( pllab == PLL_A )
    {
        /* Configure PLLA and the divider */
        PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_PLLACOUNT( 0x1f ) | CKGR_PLLAR_MULA( dwMul ) | dwDiv ;

        /* If PLL activated, wait locking */
        if ( dwMul != 0 )
        {
            /* wait for the LOCKA bit to be set */
            while ( !(PMC->PMC_SR & PMC_SR_LOCKA) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;
        }
    }

    if ( pllab == PLL_B )
    {
        /* Configure PLLB and the divider */
        PMC->CKGR_PLLBR = CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_PLLACOUNT( 0x1f ) | CKGR_PLLAR_MULA( dwMul ) | dwDiv ;

        /* If PLL activated, wait locking */
        if ( dwMul != 0 )
        {
            /* wait for the LOCKB bit to be set */
            while ( !(PMC->PMC_SR & PMC_SR_LOCKB) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;
        }
    }
}

/**
 * \brief Selection of Master Clock.
 *
 * \remarks See chapter 26.13 in SAM3S datasheet.
 *
 * \param css  The Master Clock divider source.
 * \param pres Master Clock prescaler.
 */
static void _PmcMasterClockSelection( uint32_t dwClockSource, uint32_t dwPrescaler )
{
    uint32_t dwTimeout = 0 ;

    switch ( dwClockSource )
    {
        case PMC_MCKR_CSS_SLOW_CLK :
        case PMC_MCKR_CSS_MAIN_CLK :
            /* The Master Clock selection is made by writing the CSS field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | dwClockSource ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;

            /* The Master Clock selection is made by writing the PRES field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | dwPrescaler ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;
        break ;

        case PMC_MCKR_CSS_PLLA_CLK :
        case PMC_MCKR_CSS_PLLB_CLK :
            /* The Master Clock selection is made by writing the PRES field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | dwPrescaler ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;

            /* The Master Clock selection is made by writing the CSS field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | dwClockSource ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ < CLOCK_TIMEOUT) ) ;
        break ;
    }
}

/**
 * \brief Delay several loops.
 *
 * \param dly   delay loops.
 */
static void _delay( uint32_t dwDelay )
{
    while ( dwDelay-- ) ;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for pmc_clock switch example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Configure UART baud rate to 2400bps */
    UART_Configure( 2400, BOARD_MCK ) ;

    /* Output example information */
    printf( "-- Pmc Clock Switching example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    printf( "\n\rThe master clock can be measured on PCK1 pin(PA17) by a scope.\n\r" ) ;

    /* Configure PCK as peripheral */
    PIO_Configure( &pinPCK, 1 ) ;
    /* PIO configuration for button. */
    PIO_InitializeInterrupts( 0 ) ;
    /* Configure button debouncing */
    _ConfigureButton() ;

    printf( "\n\rPlease change baudrate to 2400bps of your terminal application.\n\r" ) ;
    printf( "-I- Press button USRPB1 to continue.\n\r" ) ;
    _delay( 5000 ) ;
    for ( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf( "\n\r-I- Switch 8Mhz fast RC oscillator to be the source of the main clock \n\r" ) ;
    printf( "-I- The master clock is main clock divided by 2\n\r" ) ;
    printf( "-I- Press button USRPB1 to switch next clock configuration...\n\r" ) ;
    _delay( 5000 ) ;

    /* We switch first to slow clock */
    _PmcMasterClockSelection( PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK ) ;
    /* Then we cut the PLL A */
    _PmcConfigurePllClock( PLL_A, 0 , 0 ) ;
    /* Then we activate the Fast RC */
    _PmcEnableFastRC( CKGR_MOR_MOSCRCF_8MHZ ) ;
    _PmcMainClockSwitchFastRC() ;
    /* And finalize by switching to Fast RC */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES_CLK_2, (8000000/2) ) ;
    for( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf( "\n\r-I- Switch the XTAL 32K crystal oscillator to be the source of the slow clock\n\r" ) ;
    printf( "-I- The master clock is slow clock\n\r" ) ;
    printf( "\n\r-I- Press button USRPB1 to switch next clock configuration after it has been measured. \n\r" ) ;
    _delay( 5000 ) ;
    /* Enable the External 32K oscillator */
    _PmcEnableExternal32K() ;
    /* If a new value for CSS field corresponds to Main Clock or Slow Clock. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_SLOW_CLK, PMC_PCK_PRES_CLK, 32768 ) ;
    for ( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    _PmcEnableFastRC( CKGR_MOR_MOSCRCF_12MHZ ) ;
    _PmcMainClockSwitchFastRC() ;
    /* If a new value for CSS field corresponds to Main Clock or Slow Clock, Program the CSS field first. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES_CLK, (12000000 / 1) ) ;
    printf( "\n\r-I- Switch 12Mhz fast RC oscillator to be the source of the main clock \n\r" ) ;
    printf( "-I- The master clock is main clock \n\r" ) ;
    printf( "-I- Press button USRPB1 to switch next clock configuration...\n\r" ) ;
    for ( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf( "\n\r-I- Switch to 128Mhz PLLA clock as the source of the master clock \n\r" ) ;
    printf( "\n\r-I- The master clock is PLLA clock divided by 2 \n\r" ) ;
    _delay( 5000 ) ;
    _PmcConfigurePllClock( PLL_A, (32 - 1), 3 ) ;
    /* If a new value for CSS field corresponds to PLL Clock, Program the PRES field first*/
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    /* Then program the CSS field. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_PLLA_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_PLLA_CLK, PMC_PCK_PRES_CLK_2, (((12000000 * 32) / 3) / 2 ) ) ;
    printf( "\n\r-I- Press button USRPB1 to switch next clock configuration... \n\r" ) ;
    for ( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf( "\n\r-I- Switch the XTAL 32K crystal oscillator to be the source of the slow clock\n\r" ) ;
    printf( "-I- The master clock is slow clock\n\r" ) ;
    printf( "-I- Press button USRPB1 to switch next clock configuration after it has been measured. \n\r" ) ;
    _delay( 5000 ) ;
    _PmcEnableExternal32K() ;

    /* If a new value for CSS field corresponds to Main Clock or Slow Clock, Program the CSS field first. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    _PmcMasterClockSelection( PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_2 ) ;

    /* Then program the PRES field. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_SLOW_CLK, PMC_PCK_PRES_CLK, 32768 ) ;
    for ( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    _PmcMainClockSwitchMainOsc() ;
    /* If a new value for CSS field corresponds to Main Clock or Slow Clock, Program the CSS field first. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK ) ;
    /* Then program the PRES field. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_16 ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES_CLK_16, (12000000 / 16) ) ;
    printf("\n\r-I- Switch the external 12MHz crystal oscillator to be the source of the main clock\n\r");
    printf("-I- The master clock is main  clock divided by 16\n\r");
    printf("-I- Press button USRPB1 to switch next clock configuration...\n\r");
    for( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf("\n\r-I- Switch to 96Mhz PLLB clock as the source of the master clock\n\r");
    printf("-I- The master clock is PLLB clock divided by 2 \n\r");
    _PmcConfigurePllClock( PLL_B, (8 - 1), 1 ) ;
    /* If a new value for CSS field corresponds to PLL Clock, Program the PRES field first*/
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2);
    /* Then program the CSS field. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_PLLB_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_PLLB_CLK, PMC_PCK_PRES_CLK_2, (12000000 * 8 / 2) ) ;
    printf( "\n\r-I- Press button USRPB1 to switch next clock configuration...\n\r" ) ;
    for( _ucWaitButton=1 ; _ucWaitButton ; ) ;


    printf( "\n\r-I- Switch 8Mhz fast RC oscillator to be the source of the main clock\n\r" ) ;
    printf( "-I- The master clock is main clock \n\r" ) ;
    _PmcEnableFastRC( CKGR_MOR_MOSCRCF_8MHZ ) ;
    _PmcMainClockSwitchFastRC() ;
    /* If a new value for CSS field corresponds to Main Clock or Slow Clock, Program the CSS field first. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2 ) ;
    /* Then program the PRES field. */
    _PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK ) ;
    _ConfigureUartAndPck( PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES_CLK, (8000000 / 1) ) ;
    for( _ucWaitButton=1 ; _ucWaitButton ; ) ;

    printf( "\n\rDone \n\r" ) ;
    while( 1 ) ;
}
