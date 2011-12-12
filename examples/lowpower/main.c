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
 * \page lowpower Lowpower Example
 *
 * \section Purpose
 * This example allows to measure the consumption of the core in different modes
 * (active mode, sleep mode, wait mode, backup mode) of SAM3S.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * At startup, the program configures all the PIOs as input to avoid
 * parasite consumption (except the UART PIOs). Then a menu is displayed.
 * It is divided into 2 parts. First part allows user to change the configuration
 * (MCK/PCK frequency configuration),
 * The second part allows user to enter in a mode To measure consumption.
 * An amperemeter has to be plugged on the board instead of the VIN jumper.
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
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- Lowpower Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Press one of the keys listed in the menu to perform the corresponding action.
 *
 * \section References
 * - lowpower/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the lowpower example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** (SCR) Sleep deep bit */
#define SCR_SLEEPDEEP   (0x1 <<  2)

/** Button pin definition for wakeup */
#define BUTTON_WAKEUP   (0x1 << 12)

/** Definition of Main On-Chip RC Oscillator Frequency Selection */
#define FAST_RC_OSC_4MHZ    CKGR_MOR_MOSCRCF(0x0)
#define FAST_RC_OSC_8MHZ    CKGR_MOR_MOSCRCF(0x1)
#define FAST_RC_OSC_12MHZ   CKGR_MOR_MOSCRCF(0x2)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pins used by Interrupt Signal for Touch Screen Controller */
const Pin pinPenIRQ  = PIN_TSC_IRQ;

/** Wakeup PIN Index definition */
const uint32_t gWakeUpPinId = PIN_TSC_IRQ_WUP_ID;

/** Pushbutton #1 pin instance. */
const Pin pinPB1 = PIN_PUSHBUTTON_1;

/** Clock backup value */
static uint32_t gPllarValBackup = 0;
static uint32_t gPllbrValBackup = 0;
static uint32_t gMckrValBackup  = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initialize the chip.
 */
static void _InitChip( void )
{
    /* Disable all the peripheral clocks */
    PMC_DisableAllPeripherals();

    /* Disable USB Clock */
    REG_PMC_SCDR = PMC_SCER_UDP;

    /* Configure all PIO as input */
    /* Set PIO as Input */
    PIOA->PIO_ODR = 0xFFFFFFFF;
    PIOB->PIO_ODR = 0xFFFFFFFF;
    PIOC->PIO_ODR = 0xFFFFFFFF;
    /* Enable PIO */
    PIOA->PIO_PER = 0xFFFFFFFF;
    PIOB->PIO_PER = 0xFFFFFFFF;
    PIOC->PIO_PER = 0xFFFFFFFF;

    /* Reconfigure PIO for UART */
    UART_Configure(115200, CLOCK_GetCurrMCK() * 1000000);
}

/**
 * \brief Delay for a while.
 */
static void _Delay( volatile uint32_t dwDelay )
{
    while ( dwDelay-- ) ;
}

/**
 * \brief Enter Sleep Mode.
 * Enter condition: (WFE or WFI) + (SLEEPDEEP bit = 0) + (LPM bit = 0)
 *
 * \param type 0 - wait for interrupt, 1 - wait for event.
 */
static void _EnterSleepMode( uint8_t ucType )
{
    PMC->PMC_FSMR &= (uint32_t)~PMC_FSMR_LPM ;
    SCB->SCR &= (uint32_t)~SCR_SLEEPDEEP ;

    if ( ucType == 0 )
    {
        __WFI() ;
    }
    else
    {
        __WFE() ;
    }
}

/**
 * \brief Enter Wait Mode.
 * Enter condition: WFE + (SLEEPDEEP bit = 0) + (LPM bit = 1)
 */
static void _EnterWaitMode( void )
{
    uint32_t i ;

    PMC->PMC_FSMR |= PMC_FSMR_LPM ;
    SCB->SCR &= (uint32_t)~SCR_SLEEPDEEP ;
    __WFE() ;

    /* Waiting for MOSCRCEN bit is cleared is strongly recommended
     * to ensure that the core will not execute undesired instructions
     */
    for ( i = 0 ; i < 500 ; i++ )
    {
        __NOP() ;
    }
    while ( !(PMC->CKGR_MOR & CKGR_MOR_MOSCRCEN) ) ;
}

/**
 * \brief Enter Backup Mode.
 * Enter condition: WFE + (SLEEPDEEP bit = 1)
 */
static void _EnterBackupMode( void )
{
    SCB->SCR |= SCR_SLEEPDEEP ;
    __WFE() ;
}

/**
 * \brief Interrupt handler for buttons.
 */
static void _ISR_Button1Interrupt( void )
{
    /* Check if the button1 has been pressed */
    if ( !PIO_Get( &pinPB1 ) )
    {
        /* Do nothing here */
    }
}

/**
 * \brief Sets the wake-up inputs for fast startup mode registers.
 *
 * \param inputs  Wake up inputs to enable.
 */
static void SetFastStartupInput( uint32_t dwInputs )
{
    PMC->PMC_FSMR &= (uint32_t)~0xFFFF ;
    PMC->PMC_FSMR |= dwInputs ;
}

/**
 * \brief Save working clock.
 * Here working clock must be running from PLL
 * and external crystal oscillator is used.
 */
static void _SaveWorkingClock( uint32_t *pPllarVal, uint32_t *pPllbrVal, uint32_t *pMckrVal )
{
    /* Save previous values for PLL A, PLL B and Master Clock configuration */
    *pPllarVal = PMC->CKGR_PLLAR ;
    *pPllbrVal = PMC->CKGR_PLLBR ;
    *pMckrVal = PMC->PMC_MCKR ;
}

/**
 * \brief Restore working clock.
 * Here working clock must be running from PLL
 * and external crystal oscillator is used.
 */
static void RestoreWorkingClock( uint32_t PllarVal, uint32_t PllbrVal, uint32_t MckrVal )
{
    /* switch to slow clock first */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_SLOW_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk) | PMC_MCKR_PRES_CLK ;

    /* Restart Main Oscillator */
    PMC->CKGR_MOR = (0x37 << 16) | (0x3F<<8) | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN ;
    while ( !(PMC->PMC_SR & PMC_SR_MOSCXTS) ) ;
    /* Switch to moscsel */
    PMC->CKGR_MOR = (0x37 << 16) | (0x3F<<8) | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL ;
    while ( !(PMC->PMC_SR & PMC_SR_MOSCSELS) ) ;

    /* Switch to main oscillator */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk) | PMC_MCKR_PRES_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    /* Restart PLL A */
    if ( (PllarVal & CKGR_PLLAR_MULA_Msk) != 0 )
    {
        PMC->CKGR_PLLAR = PllarVal ;
        while ( !(PMC->PMC_SR & PMC_SR_LOCKA) ) ;
    }

    /* Restart PLL B */
    if ( (PllbrVal & CKGR_PLLBR_MULB_Msk) != 0 )
    {
        PMC->CKGR_PLLBR = PllbrVal ;
        while ( !(PMC->PMC_SR & PMC_SR_LOCKB) ) ;
    }

    /* Switch to fast clock */
    PMC->PMC_MCKR = (MckrVal & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    PMC->PMC_MCKR = MckrVal ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;
}

/**
 * \brief Switch MCK to 32KHz RC Oscillator.
 *
 * \param pres Programmable Clock Prescaler
 */
static void _SwitchMckTo32KRcOsc( uint32_t dwPres )
{
    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_SLOW_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk) | dwPres ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    /* Stop PLL A */
    /* STMODE must be set at 2 when the PLLA is Off */
    PMC->CKGR_PLLAR = 0x2 << 14 ;

    /* Stop PLLB */
    /* STMODE must be set at 2 when the PLLB is OFF */
    PMC->CKGR_PLLBR = 0x2 << 14 ;

    /* Stop Main Oscillator */
    PMC->CKGR_MOR = (0x37 << 16) | (0x3F << 8) ;
}

/**
 * \brief Switch MCK to FastRC (Main On-Chip RC Oscillator).
 *
 * \param moscrcf Main On-Chip RC Oscillator Frequency Selection.
 * \param pres    Processor Clock Prescaler.
 */
static void _SwitchMck2FastRC( uint32_t dwMoscrcf, uint32_t dwPres )
{
    /* Enable Fast RC oscillator but DO NOT switch to RC now . Keep MOSCSEL to 1 */
    PMC->CKGR_MOR = CKGR_MOR_MOSCSEL | (0x37 << 16) | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCRCEN ;
    /* Wait the Fast RC to stabilize */
    while ( !(PMC->PMC_SR & PMC_SR_MOSCRCS) ) ;

    /* Switch from Main Xtal osc to Fast RC */
    PMC->CKGR_MOR = (0x37 << 16) | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN ;
    /* Wait for Main Oscillator Selection Status bit MOSCSELS */
    while ( !(PMC->PMC_SR & PMC_SR_MOSCSELS) ) ;

    /* Disable Main XTAL Oscillator */
    PMC->CKGR_MOR = (0x37 << 16) | CKGR_MOR_MOSCRCEN ;

    /* Change frequency of Fast RC oscillator */
    PMC->CKGR_MOR = (0x37 << 16) | PMC->CKGR_MOR | dwMoscrcf ;
    /* Wait the Fast RC to stabilize */
    while ( !(PMC->PMC_SR & PMC_SR_MOSCRCS) ) ;

    /* Switch to main clock */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;
    PMC->PMC_MCKR = (PMC->PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk) | dwPres ;
    while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) ) ;

    /* Stop PLL A */
    /* STMODE must be set at 2 when the PLLA is Off */
    PMC->CKGR_PLLAR = 0x2 << 14 ;

    /* Stop PLLB */
    /* STMODE must be set at 2 when the PLLB is OFF */
    PMC->CKGR_PLLBR = 0x2 << 14 ;
}

/**
 * \brief Test Active Mode
 */
static void _TestActiveMode( void )
{
    uint32_t dw ;

    printf( "Enter in Active Mode\n\r" ) ;
    printf( "Press a key to go out...\n\r" ) ;
    for( dw=0 ; !UART_IsRxReady() ; dw++ )
    {
    }
    UART_GetChar() ; /* empty RX buffer */

    printf( "Exit Active mode\n\r" ) ;
}

/**
 * \brief Display menu of sleep mode.
 */
static void _DisplayMenuSleep( void )
{
    printf("\n\r");
    printf("Select clock configuration for sleep mode:\n\r");
    printf("  0 : PCK=500Hz by using 32K RC Oscillator\n\r");
    printf("  1 : PCK=32KHz by using 32K RC Oscillator\n\r");
    printf("  2 : PCK=125KHz by using Fast RC Oscillator\n\r");
    printf("  3 : PCK=250KHz by using Fast RC Oscillator\n\r");
    printf("  4 : PCK=500KHz by using Fast RC Oscillator\n\r");
    printf("  5 : PCK=1MHz by using Fast RC Oscillator\n\r");
    printf("  6 : PCK=2MHz by using Fast RC Oscillator\n\r");
    printf("  7 : PCK=4MHz by using Fast RC Oscillator\n\r");
    printf("  8 : PCK=8MHz by using Fast RC Oscillator\n\r");
    printf("  9 : PCK=12MHz by using Fast RC Oscillator\n\r");
    printf("  c : Current clock PCK=%dMHz running from PLL\n\r", CLOCK_GetCurrPCK());
    printf("\n\r");
}

/**
 * \brief Test Sleep Mode
 */
static void _TestSleepMode( void )
{
    uint8_t ucKey = '0';
    uint32_t temp;

    /* Enable Button1 interrupt for wakeup */
    NVIC_EnableIRQ( (IRQn_Type)pinPB1.id ) ; /* Enable PIO controller IRQs. */
    PIO_EnableIt( &pinPB1 ) ;                /* Enable PIO line interrupts. */

    /* Disable Brownout Detector */
    SUPC->SUPC_MR |= (uint32_t)(0xA5 << 24) | (0x01 << 13);

    /* Save current working clock */
    _SaveWorkingClock( &gPllarValBackup, &gPllbrValBackup, &gMckrValBackup ) ;

    printf( "Enter in Sleep Mode\n\r" ) ;
    printf( " - Press USRPB1 button to wakeup\n\r" ) ;

    /* Select frequency of sleep mode */
    _DisplayMenuSleep() ;
    ucKey = UART_GetChar() ;
    printf( "Selected option is: %c\n\r", ucKey ) ;

    /* Switch clock frequency */
    switch ( ucKey )
    {
        case '0': /* 500Hz */
            _SwitchMckTo32KRcOsc( PMC_MCKR_PRES_CLK_64 ) ;
        break ;

        case '1': /* 32KHz */
            _SwitchMckTo32KRcOsc( PMC_MCKR_PRES_CLK ) ;
        break ;

        case '2': /* 125KHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK_32 ) ;
        break ;

        case '3': /* 250KHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK_16 ) ;
        break ;

        case '4': /* 500KHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK_8 ) ;
        break ;

        case '5': /* 1MHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK_4 ) ;
        break ;

        case '6': /* 2MHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK_2 ) ;
        break ;

        case '7': /* 4MHz */
            _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK ) ;
        break ;

        case '8': /* 8MHz */
            _SwitchMck2FastRC( FAST_RC_OSC_8MHZ, PMC_MCKR_PRES_CLK ) ;
        break ;

        case '9': /* 12MHz */
            _SwitchMck2FastRC( FAST_RC_OSC_12MHZ, PMC_MCKR_PRES_CLK ) ;
        break ;

        case 'c':
        default:
            /* do nothing */
        break ;
    }

    /* Enter Sleep Mode */
    _EnterSleepMode( 0 ) ;

    /* Restore working clock */
    RestoreWorkingClock( gPllarValBackup, gPllbrValBackup, gMckrValBackup ) ;

    /* Enable Brownout Detector */
    temp = SUPC->SUPC_MR & 0x00FFFFFF;
    SUPC->SUPC_MR = (uint32_t)(0xA5 << 24) | (temp & (uint32_t)(~(0x01 << 13)));

    /* Reconfigure UART */
    UART_Configure( 115200, CLOCK_GetCurrMCK()*1000000 ) ;
    printf( "Exit Sleep Mode\n\r" ) ;
}

/**
 * \brief Test Wait Mode
 */
static void _TestWaitMode( void )
{
    uint32_t dwTemp ;

    printf( "Enter in Wait Mode\n\r" ) ;
    printf( " - Switch to 4MHz Fast RC oscillator, PLL stopped\n\r" ) ;
    printf( " - Touch the LCD screen to wakeup\n\r" ) ;

    /* Delay for a while */
    _Delay( 500 ) ;

    /* Save current working clock */
    _SaveWorkingClock( &gPllarValBackup, &gPllbrValBackup, &gMckrValBackup ) ;

    /* Disable Brownout Detector */
    SUPC->SUPC_MR |= (uint32_t)(0xA5 << 24) | (0x01 << 13) ;

    /* Configure 4Mhz fast RC oscillator */
    _SwitchMck2FastRC( FAST_RC_OSC_4MHZ, PMC_MCKR_PRES_CLK ) ;

    /* Set wakeup input for fast startup */
    SetFastStartupInput( gWakeUpPinId ) ;

    /* Enter Wait Mode */
    _EnterWaitMode() ;

    /* Restore working clock */
    RestoreWorkingClock( gPllarValBackup, gPllbrValBackup, gMckrValBackup ) ;

    /* Enable Brownout Detector */
    dwTemp = SUPC->SUPC_MR & 0x00FFFFFF;
    SUPC->SUPC_MR = (uint32_t)(0xA5 << 24) | (dwTemp & (uint32_t)(~(0x01 << 13))) ;

    /* Reconfigure UART */
    UART_Configure( 115200, CLOCK_GetCurrMCK()*1000000 ) ;

    printf( "Exit Wait Mode\n\r" ) ;
}

/**
 * \brief Test Backup Mode.
 */
static void _TestBackupMode( void )
{
    printf( "Enter in Backup Mode\n\r" ) ;
    printf( " - Touch the LCD screen to wakeup\n\r" ) ;

    /* Delay for a while */
    _Delay( 500 ) ;

    /* GPBR0 is for recording times of entering backup mode */
    REG_GPBR_GPBR0 += 1 ;

    /* Enable the PIO for wake-up */
    SUPC->SUPC_WUIR = (gWakeUpPinId << 16) | gWakeUpPinId ;

    /* Enter Backup Mode */
    _EnterBackupMode() ;
}

/**
 * \brief Display test Core menu.
 */
static void _DisplayMenuCore( void )
{
    printf( "\n\r" ) ;
    printf( "===============================================================\n\r" ) ;
    printf( "Menu: press a key to change the configuration or select a mode.\n\r" ) ;
    printf( "===============================================================\n\r" ) ;
    printf( "Config:\n\r" ) ;
    printf( "  f : Clock      = PCK=%d MCK=%d\n\r", CLOCK_GetCurrPCK(), CLOCK_GetCurrMCK() ) ;
    printf( "Mode:\n\r" ) ;
    printf( "  A : Active Mode\n\r" ) ;
    printf( "  S : Sleep Mode\n\r" ) ;
    printf( "  W : Wait Mode\n\r" ) ;
    printf( "  B : Backup Mode(Had entered %d times).\n\r", (int)REG_GPBR_GPBR0 ) ;
    printf( "Quit:\n\r" ) ;
    printf( "  Q : Quit test.\n\r" ) ;
    printf( "---------------------------------------------------------------\n\r" ) ;
    printf( "\n\r" ) ;
}

/**
 * \brief Test Core consumption
 */
static void _TestCore( void )
{
    uint8_t ucKey = 0 ;

    while ( 1 )
    {
        _DisplayMenuCore() ;

        ucKey = UART_GetChar() ;
        switch ( ucKey )
        {
            /* Configuration */
            case 'f' :
            case 'F' :
                CLOCK_UserChangeConfig() ;
            break ;

            /* Mode test */
            case 'a' :
            case 'A' :
                _TestActiveMode() ;
            break ;

            case 's' :
            case 'S' :
                _TestSleepMode() ;
            break ;

            case 'w' :
            case 'W' :
                _TestWaitMode() ;
            break ;

            case 'b' :
            case 'B' :
                _TestBackupMode() ;
            break ;

            /* Quit test */
            case 'q' :
            case 'Q' :
                goto test_core_end ;

            default :
                printf( "This menu does not exist !\n\r" ) ;
            break ;
        } /* switch */
    }

test_core_end:
    printf(" Exit core consumption test mode.\n\r");
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for lowpower example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    /* Set FWS for Embedded Flash Access */
    EFC->EEFC_FMR = (1 << 8);

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* initialize the chip for the power consumption test */
    _InitChip() ;

    /* Set default clock */
    CLOCK_SetConfig( 0 ) ;
    printf( "\n\rThe core (PCK) is running @ %dMhz and peripherals (MCK) @ %dMHz\n\r", CLOCK_GetCurrPCK(), CLOCK_GetCurrMCK() ) ;

    /* Output example information */
    printf( "\n\r\n\r\n\r" ) ;
    printf( "-- Lowpower Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Configure pios */
    /* pinPenIRQ for wait & backup mode wakeup input */
    /* pinPB1 for sleep mode wakeup input*/
    PIO_InitializeInterrupts( 0 ) ;

    PIO_Configure( &pinPenIRQ, PIO_LISTSIZE( pinPenIRQ ) ) ;

    PIO_Configure( &pinPB1, 1 ) ;
    PIO_SetDebounceFilter( &pinPB1, 10 ) ;
    PIO_ConfigureIt( &pinPB1, (void (*)(const Pin *))_ISR_Button1Interrupt ) ;

    /* Test core consumption */
    _TestCore() ;

    return 0 ;
}
