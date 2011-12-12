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
 * \page rtc RTC Example
 *
 * \section Purpose
 *
 * This basic example shows how to use the Real-Time Clock (RTC) peripheral
 * available on the newest Atmel AT91 microcontrollers. The RTC enables easy
 * time and date management and allows the user to monitor events like a
 * configurable alarm, second change, calendar change, and so on.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * Upon startup, the program displays the currently set time and date
 * and a menu to perform the following:
 *     \code
 *     Menu:
 *        t - Set time
 *        d - Set date
 *        i - Set time alarm
 *        m - Set date alarm
 *        c - Clear the alarm notification (only if it has been triggered)
 *     \endcode
 *
 * Setting the time, date and time alarm is done by using Menu option "t", "d",
 * the display is updated accordingly.
 *
 * The time alarm is triggered only when the second, minute and hour match the preset
 * values; the date alarm is triggered only when the month and date match the preset
 * values. If both time alarm and date alarm are set, only when the second, minute,
 * hour, month and date match the preset values, the alarm will be triggered.
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
 *     -- RTC Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *
 *     Menu:
 *     t - Set time
 *     d - Set date
 *     i - Set time alarm
 *     m - Set date alarm
 *     q - Quit
 *    \endcode
 * -# Press one of the keys listed in the menu to perform the corresponding action.
 *
 * \section References
 * - rtc/main.c
 * - rtc.c
 * - rtc.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the rtc example.
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

/** Main menu is being displayed. */
#define STATE_MENU              0
/** Time is being edited. */
#define STATE_SET_TIME          1
/** Date is being edited. */
#define STATE_SET_DATE          2
/** Time alarm is being edited. */
#define STATE_SET_TIME_ALARM    3
/** Date alarm is being edited. */
#define STATE_SET_DATE_ALARM    4

/** Maximum size of edited string */
#define MAX_EDIT_SIZE       10

/** Macro for check digit character */
#define IsDigitChar(c) ((c) >= '0' && (c) <='9')
/** Macro for converting char to digit */
#define CharToDigit(c) ((c) - '0')

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Current state of application. */
static unsigned int bState = STATE_MENU;

/** Edited hour. */
static unsigned char newHour;
/** Edited minute. */
static unsigned char newMinute;
/** Edited second. */
static unsigned char newSecond;

/** Edited year. */
static unsigned short newYear;
/** Edited month. */
static unsigned char newMonth;
/** Edited day. */
static unsigned char newDay;
/** Edited day-of-the-week. */
static unsigned char newWeek;

/** Indicates if alarm has been trigerred and not yet cleared. */
static unsigned char alarmTriggered = 0;

/** store time string */
static char rtc_time[8+1] = {'0', '0', ':', '0', '0', ':', '0', '0','\0'};
/** store date string */
static char date[10+1] = {'0', '0', '/', '0', '0', '/', '0', '0', '0', '0', '\0'};
/** week string */
static char pDayNames[7][4] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
/** console erase sequence */
static char pEraseSeq[] = "\b \b";
/** output format string buffer */
static char calendar[80];
/** for idendify refreshing menu */
static unsigned int bMenuShown = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Display a string on the terminal.
 */
static inline void UART_puts( const char *pStr )
{
    while ( *pStr )
    {
        UART_PutChar(*pStr++ ) ;
    }
}

/**
 * \brief Print a formatted string into a buffer.
 */
static signed int _PrnToBuf( char *pBuf, const char *pFormat, ... )
{
    va_list    ap ;
    signed int rc ;

    va_start( ap, pFormat ) ;
    rc = vsprintf( pBuf, pFormat, ap ) ;
    va_end( ap ) ;

    return rc ;
}

/**
 * \brief Get new time, successful value is put in newHour, newMinute, newSecond.
 */
static int _GetNewTime( void )
{
    char ucKey ;
    int i=0 ;

    /* clear setting variable */
    newHour = newMinute = newSecond = 0xFF;

    /* use time[] as a format template */
    while ( 1 )
    {
        ucKey = UART_GetChar();

        /* end input */
        if ( ucKey == 0x0d || ucKey == 0x0a )
        {
            UART_puts("\n\r");
            break;
        }

        /* DEL or BACKSPACE */
        if ( ucKey == 0x7f || ucKey == 0x08 )
        {
            if ( i > 0 )
            {
                /* end of time[], index then one more back */
                if ( !rtc_time[i] )
                {
                    --i ;
                }

                UART_puts( pEraseSeq ) ;
                --i ;

                /* delimitor ':' for time is uneditable */
                if ( !IsDigitChar( rtc_time[i] ) && i > 0 )
                {
                    UART_puts( pEraseSeq ) ;
                    --i ;
                }
            }
        }

        /* end of time[], no more input except above DEL/BS or enter to end */
        if ( !rtc_time[i] )
        {
            continue ;
        }

        if ( !IsDigitChar( ucKey ) )
        {
            continue ;
        }

        UART_PutChar( ucKey ) ;
        rtc_time[i++] = ucKey ;

        /* ignore non digit position if not end */
        if ( !IsDigitChar( rtc_time[i] ) && i < 8 )
        {
            UART_PutChar( rtc_time[i] ) ;
            ++i ;
        }
    }

    if ( i == 0 )
    {
        return 0 ;
    }

    if ( i != 0 && rtc_time[i] != '\0' )
    {
        return 1 ; /* failure input */
    }

    newHour = CharToDigit( rtc_time[0] ) * 10 + CharToDigit( rtc_time[1] ) ;
    newMinute = CharToDigit( rtc_time[3] ) * 10 + CharToDigit( rtc_time[4] ) ;
    newSecond = CharToDigit( rtc_time[6] ) * 10 + CharToDigit( rtc_time[7] ) ;

    /* success input. verification of data is left to RTC internal Error Checking */
    return 0 ;
}

/**
 * \brief Calculate week from year, month,day.
 */
static char _CalcWeek( int year, int month, int day )
{
    char week ;

    if ( month == 1 || month == 2 )
    {
        month += 12 ;
        --year ;
    }

    week = (day+2*month+3*(month+1)/5+year+year/4-year/100+year/400)%7;

    ++week ;

    return week ;
}

/**
 * \brief Get new time, successful value is put in newYear, newMonth, newDay, newWeek.
 */
static int _GetNewDate( void )
{
    char ucKey ;
    int i=0;

    /* clear setting variable */
    newYear = 0xFFFF;
    newMonth = newDay= newWeek = 0xFF;

    /* use time[] as a format template */
    while ( 1 )
    {
        ucKey = UART_GetChar() ;

        /* end input */
        if ( ucKey == 0x0d || ucKey == 0x0a )
        {
            UART_puts( "\n\r" ) ;
            break ;
        }

        /* DEL or BACKSPACE */
        if ( ucKey == 0x7f || ucKey == 0x08 )
        {
            if ( i > 0 )
            {
                /* end of date[], index then one more back */
                if ( !date[i] )
                {
                    --i ;
                }

                UART_puts( pEraseSeq ) ;
                --i ;

                /* delimitor '/' for date is uneditable */
                if ( !IsDigitChar( date[i] ) && i > 0 )
                {
                    UART_puts( pEraseSeq ) ;
                    --i ;
                }
            }
        }

        /* end of time[], no more input except above DEL/BS or enter to end */
        if ( !date[i] )
        {
            continue ;
        }

        if ( !IsDigitChar( ucKey ) )
        {
            continue;
        }

        UART_PutChar( ucKey ) ;
        date[i++] = ucKey ;

        /* ignore non digit position */
        if ( !IsDigitChar( date[i] ) && i < 10 )
        {
            UART_PutChar( date[i] ) ;
            ++i ;
        }
    }

    if ( i == 0 )
    {
        return 0 ;
    }

    if ( i != 0 && date[i] != '\0' && i != 6 )
    {
        return 1 ; /* failure input */
    }

    /* MM-DD-YY */
    newMonth = CharToDigit( date[0] ) * 10 + CharToDigit( date[1] ) ;
    newDay = CharToDigit( date[3] ) * 10 + CharToDigit( date[4] ) ;
    if ( i != 6 )
    {/* not scenario of getting mm/dd/ only for alarm */
        newYear = CharToDigit(date[6]) * 1000 + CharToDigit(date[7]) * 100 +
                  CharToDigit(date[8]) * 10 + CharToDigit(date[9] ) ;
        newWeek = _CalcWeek( newYear, newMonth, newDay ) ;
    }

    /* success input. verification of data is left to RTC internal Error Checking */
    return 0 ;
}


/**
 * \brief Displays the user interface on the terminal.
 */
static void _RefreshDisplay( void )
{

    unsigned char hour, minute, second ;
    unsigned short year ;
    unsigned char month, day, week ;

    if ( bState != STATE_MENU )
    { /* not in menu display mode, in set mode */
    }
    else
    {
        /* Retrieve date and time */
        RTC_GetTime( RTC, &hour, &minute, &second ) ;
        RTC_GetDate( RTC, &year, &month, &day, &week ) ;

        /* display */
        if ( !bMenuShown )
        {
            printf( "\n\rMenu:\n\r" ) ;
            printf( "  t - Set time\n\r" ) ;
            printf( "  d - Set date\n\r" ) ;
            printf( "  i - Set time alarm\n\r" ) ;
            printf( "  m - Set date alarm\n\r" ) ;

            if ( alarmTriggered )
            {
                printf( "  c - Clear alarm notification\n\r" ) ;
            }

            printf( "  q - Quit!\n\r" ) ;

            printf( "\n\r" ) ;

            bMenuShown = 1 ;
        }

        /* update current date and time */
        _PrnToBuf( rtc_time, "%02d:%02d:%02d", hour, minute, second ) ;
        _PrnToBuf( date, "%02d/%02d/%04d", month, day, year ) ;
        _PrnToBuf( calendar, " [Time/Date: %s, %s %s ][Alarm status:%s]", rtc_time, date, pDayNames[week-1], alarmTriggered?"Triggered!":"" ) ;

        printf( "\r%s", calendar ) ;
    }
}

/**
 * \brief Interrupt handler for the RTC. Refreshes the display.
 */
void RTC_IrqHandler( void )
{
    uint32_t dwStatus = RTC->RTC_SR ;

    /* Second increment interrupt */
    if ( (dwStatus & RTC_SR_SEC) == RTC_SR_SEC )
    {
        /* Disable RTC interrupt */
        RTC_DisableIt( RTC, RTC_IDR_SECDIS ) ;

        _RefreshDisplay() ;

        RTC->RTC_SCCR = RTC_SCCR_SECCLR ;

        RTC_EnableIt( RTC, RTC_IER_SECEN ) ;
    }
    /* Time or date alarm */
    else
    {
        if ( (dwStatus & RTC_SR_ALARM) == RTC_SR_ALARM )
        {
            /* Disable RTC interrupt */
            RTC_DisableIt( RTC, RTC_IDR_ALRDIS ) ;

            alarmTriggered = 1 ;
            _RefreshDisplay() ;
            bMenuShown = 0 ; /* shown additional menu item for clear notification */
            RTC->RTC_SCCR = RTC_SCCR_ALRCLR ;

            RTC_EnableIt( RTC, RTC_IER_ALREN ) ;
        }
    }
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for RTC example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
    uint8_t ucKey ;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf( "\n\r\n\r\n\r" ) ;
    printf( "-- RTC Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Default RTC configuration */
    RTC_SetHourMode( RTC, 0 ) ; /* 24-hour mode */
    if ( RTC_SetTimeAlarm( RTC, 0, 0, 0 ) )
    {
        printf( "\n\r Disable time alarm fail!" ) ;
    }

    if ( RTC_SetDateAlarm( RTC, 0, 0 ) )
    {
        printf( "\n\r Disable date alarm fail!" ) ;
    }

    /* Configure RTC interrupts */
    NVIC_DisableIRQ( RTC_IRQn ) ;
    NVIC_ClearPendingIRQ( RTC_IRQn ) ;
    NVIC_SetPriority( RTC_IRQn, 0 ) ;
    NVIC_EnableIRQ( RTC_IRQn ) ;
    RTC_EnableIt( RTC, RTC_IER_SECEN | RTC_IER_ALREN ) ;

    /* Refresh display once */
    _RefreshDisplay() ;

    /* Handle keypresses */
    while ( 1 )
    {
        ucKey = UART_GetChar() ;

        /* set time */
        if ( ucKey == 't' )
        {
            bState = STATE_SET_TIME;

            do
            {
                printf( "\n\r\n\r Set time(hh:mm:ss): " ) ;
            } while( _GetNewTime() ) ;

            /* if valid input, none of variable for time is 0xff */
            if ( newHour != 0xFF )
            {
                if ( RTC_SetTime( RTC, newHour, newMinute, newSecond ) )
                {
                    printf( "\n\r Time not set, invalid input!\n\r" ) ;
                }
            }

            bState = STATE_MENU ;
            bMenuShown = 0 ;
            _RefreshDisplay() ;
        }


        /* set date */
        if ( ucKey == 'd' )
        {
            bState = STATE_SET_DATE ;

            do
            {
                printf( "\n\r\n\r Set date(mm/dd/yyyy): " ) ;
            } while ( _GetNewDate() ) ;

            /* if valid input, none of variable for date is 0xff(ff) */
            if ( newYear !=0xFFFF )
            {
                if ( RTC_SetDate( RTC, newYear, newMonth, newDay, newWeek ) )
                {
                    printf( "\n\r Date not set, invalid input!\n\r" ) ;
                }
            }

            /* only 'mm/dd' inputed */
            if ( newMonth != 0xFF && newYear == 0xFFFF )
            {
                printf( "\n\r Not Set for no year field!\n\r" ) ;
            }

            bState = STATE_MENU ;
            bMenuShown = 0 ;
            _RefreshDisplay() ;
        }


        /* set time alarm */
        if ( ucKey == 'i')
        {
            bState = STATE_SET_TIME_ALARM ;

            do
            {
                printf( "\n\r\n\r Set time alarm(hh:mm:ss): " ) ;
            } while ( _GetNewTime() ) ;

            if ( newHour != 0xFF )
            {
                if ( RTC_SetTimeAlarm( RTC, &newHour, &newMinute, &newSecond ) )
                {
                    printf( "\n\r Time alarm not set, invalid input!\n\r" ) ;
                }
                else
                {
                    printf( "\n\r Time alarm is set at %02d:%02d:%02d!", newHour, newMinute, newSecond ) ;
                }
            }
            bState = STATE_MENU ;
            bMenuShown = 0 ;
            alarmTriggered = 0 ;
            _RefreshDisplay() ;
        }

        /* set date alarm */
        if ( ucKey == 'm' )
        {
            bState = STATE_SET_DATE_ALARM;

            do
            {
                printf( "\n\r\n\r Set date alarm(mm/dd/): " ) ;
            } while ( _GetNewDate() ) ;

            if ( newYear == 0xFFFF && newMonth != 0xFF )
            {
                if ( RTC_SetDateAlarm( RTC, &newMonth, &newDay ) )
                {
                    printf( "\n\r Date alarm not set, invalid input!\n\r" ) ;
                }
                else
                {
                    printf( "\n\r Date alarm is set on %02d/%02d!", newMonth, newDay ) ;
                }

            }
            bState = STATE_MENU ;
            bMenuShown = 0 ;
            alarmTriggered = 0 ;
            _RefreshDisplay() ;
        }

        /* clear trigger flag */
        if ( ucKey == 'c' )
        {
            alarmTriggered = 0 ;
            bMenuShown = 0 ;
            _RefreshDisplay() ;
        }

        /* quit */
        if ( ucKey == 'q' )
        {
            break ;
        }
    }

    return 0 ;
}

