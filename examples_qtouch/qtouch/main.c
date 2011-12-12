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
 *  \page qtouch QTouch example with SAM3S Microcontrollers
 *
 *  \section Purpose
 *  The QTouch example will help new users get familiar with Atmel's
 *  SAM3S family of microcontrollers. This basic application shows the startup
 *  sequence of a chip and how to use its core peripherals.
 *
 *  \section Requirements
 *  This package can be used with SAM3S evaluation kits.
 *
 *  \section Description
 *  The demonstration program makes printf on the RS232 (UART) for see wich
 *  button are pressed, and the position of the slider.
 *
 *  \section Usage
 *  -# Build the program and download it inside the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or to the
 *     <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *     IAR EWARM User Guide</a>,
 *     depending on your chosen solution.
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear (values depend
 *    on the board and chip used):
 *     \code
 *      -- QTouch Project--
 *      -- ATSAM3S-EK
 *      -- Compiled: xxxxxxxxxxxxx --
 *      Configure sys tick to get 25 ms tick period.
 *      Library QTouch for IAR
 *      [here is display the button or slider position]
 *     \endcode
 *
 *  \section References
 *  - qtouch/main.c
 */

/** \file
 *
 *  This file contains all the specific code for the QTouch example.
 *
 */


/*----------------------------------------------------------------------------
  Headers
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>


#include "board.h"
#include "libqtouch.h"

/*----------------------------------------------------------------------------
  prototypes
----------------------------------------------------------------------------*/

/* initialise host app, pins, watchdog, etc */
static void init_system( void );

/* configure timer ISR to fire regularly */
static void init_timer_isr( void );

/*  Assign the parameters values to global configuration parameter structure    */
static void qt_set_parameters( void );

/* Configure the sensors */
static void config_sensors(void);


/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

#define GET_SENSOR_STATE(SENSOR_NUMBER) (qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8)))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

/* QDebug */
extern uint16_t timestamp1_hword;
extern uint16_t timestamp1_lword;
extern uint16_t timestamp2_hword;
extern uint16_t timestamp2_lword;
extern uint16_t timestamp3_hword;
extern uint16_t timestamp3_lword;

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/
/* measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;
/* Output can be observed in the watch window using this pointer */
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;

/*----------------------------------------------------------------------------
  static variables
----------------------------------------------------------------------------*/
/* flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;

/* current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0u;

/**
 *  \brief       trace_library_info
 *
 *  Display information about library used
 */
static void trace_library_info(void)
{
    qt_lib_siginfo_t plib_sig;

    qt_measure_sensors( current_time_ms_touch );
    qt_get_library_sig(&plib_sig);

    /*printf("lib_sig_lword 0x%X\r\n", plib_sig.lib_sig_lword);*/
    if((plib_sig.lib_sig_lword&0x01) == 0)
    {
        printf("Library QTouch ");
    }
    else
    {
        printf("Library QMatrix ");
    }
    if(((plib_sig.lib_sig_lword>>2)&0x01) == 0)
    {
        printf("for GCC\r\n");
    }
    else
    {
        printf("for IAR\r\n");
    }
    printf("Max Channels %d\r\n", (plib_sig.lib_sig_lword>>3)&0x7F);

    if(((plib_sig.lib_sig_lword>>10)&0x01) == 0)
    {
        printf("supports only keys \r\n");
    }
    else
    {
        printf("supports keys and rotors/sliders\r\n");
    }
    printf("Maximum number of Rotors/Sliders %d \r\n", (plib_sig.lib_sig_lword>>11)&0x1F);
    /*printf("lib_sig_hword 0x%X\r\n",   plib_sig.lib_sig_hword);*/
    printf("Version 0x%X\r\n", plib_sig.library_version);
    printf("\r\n");
}
/**
 *  \brief    main
 *
 *  main code entry point
 */
int main( void )
{
    /*status flags to indicate the re-burst for library*/
    uint16_t status_flag = 0u;
    uint16_t burst_flag = 0u;

    uint8_t vld_pressed = 0;
    uint8_t up_pressed = 0;
    uint8_t dwn_pressed = 0;
    uint8_t lft_pressed = 0;
    uint8_t rgt_pressed = 0;

    static uint8_t old_position = 0;

    /* initialise host app, pins, watchdog, etc */
    init_system();

    /* reset touch sensing */
    qt_reset_sensing();

    /* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
    config_sensors();

    /* initialise touch sensing */
    qt_init_sensing();

    /* Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
    qt_set_parameters( );

    /* configure timer ISR to fire regularly */
    init_timer_isr();

    /* Address to pass address of user functions */
    /* This function is called after the library has made capacitive measurements,
    * but before it has processed them. The user can use this hook to apply filter
    * functions to the measured signal values.(Possibly to fix sensor layout faults) */
    qt_filter_callback = 0;

    trace_library_info();

    /* enable interrupts */
    //sei();

    /* loop forever */
    for( ; ; )
    {
        if( time_to_measure_touch )
        {
            /* clear flag: it's time to measure touch */
            time_to_measure_touch = 0u;

            do
            {
                /*  one time measure touch sensors    */
                status_flag = qt_measure_sensors( current_time_ms_touch );

                burst_flag = status_flag & QTLIB_BURST_AGAIN;

                /*Time critical host application code goes here*/

            } while (burst_flag);
        }

        /*  Time Non-critical host application code goes here  */

        if(( GET_SENSOR_STATE(0) != 0 )&&(vld_pressed==0))
        {
            vld_pressed=1;
            printf ("Vld P ");
        }
        else
        {
            if(( GET_SENSOR_STATE(0) == 0)&&(vld_pressed==1))
            {
                  printf ("Vld R ");
                  vld_pressed=0;
            }
        }
        if(( GET_SENSOR_STATE(1) != 0 )&&(up_pressed==0))
        {
            up_pressed=1;
            printf ("Up P ");
        }
        else
        {
            if(( GET_SENSOR_STATE(1) == 0)&&(up_pressed==1))
            {
                printf ("Up R ");
                up_pressed=0;
            }
        }
        if(( GET_SENSOR_STATE(2) != 0 )&&(dwn_pressed==0))
        {
            dwn_pressed=1;
            printf ("Dwn P ");
        }
        else
        {
            if(( GET_SENSOR_STATE(2) == 0)&&(dwn_pressed==1))
            {
                printf ("Dwn R ");
                dwn_pressed=0;
            }
        }
        if(( GET_SENSOR_STATE(3) != 0 )&&(lft_pressed==0))
        {
            lft_pressed=1;
            printf ("lft P ");
        }
        else
        {
            if(( GET_SENSOR_STATE(3) == 0)&&(lft_pressed==1))
            {
                printf ("Lft R ");
                lft_pressed=0;
            }
        }
        if(( GET_SENSOR_STATE(4) != 0 )&&(rgt_pressed==0))
        {
            rgt_pressed=1;
            printf ("Rgt P ");
        }
        else
        {
            if(( GET_SENSOR_STATE(4) == 0)&&(rgt_pressed==1))
            {
                printf ("Rgt R ");
                rgt_pressed=0;
            }
        }
        if( GET_ROTOR_SLIDER_POSITION(0) != old_position )
        {
            old_position = GET_ROTOR_SLIDER_POSITION(0);
            printf ("%d ", old_position);
        }
#if defined (  __GNUC__  )
        fflush(stdout);
#endif
    }
}
/**
 *  \brief  qt_set_parameters
 *
 *  This will fill the default threshold values in the configuration
 *  data structure.But User can change the values of these parameters
 *  initialize configuration data for processing
 */
static void qt_set_parameters( void )
{
    /*  This will be modified by the user to different values   */
    qt_config_data.qt_di              = DEF_QT_DI;
    qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
    qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
    qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
    qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
    qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
    qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}
/**
 *  \brief Configure the sensors
 *
 *      PIOC     0  1  chanel 0
 *      PIOC     2  3  chanel 1
 *      PIOC     4  5  chanel 2
 *      PIOC     6  7  chanel 3
 *      PIOC     8  9  chanel 4
 *      PIOC    10 11  chanel 5
 *      PIOC    12 13  chanel 6
 *      PIOC    14 15  chanel 7
 *      PIOC    16 17  chanel 8
 *      PIOC    18 19  chanel 9
 *      PIOC    20 21  chanel 10
 *      PIOC    22 23  chanel 11   : Valid
 *      PIOC    24 25  chanel 12   : UP
 *      PIOC    26 27  chanel 13   : Down
 *      PIOC    28 29  chanel 14   : Left
 *      PIOC    30 31  chanel 15   : Right
 *      PIOA     0  1  chanel 16   : slider
 *      PIOA     2  3  chanel 17   : slider
 *      PIOA     4  5  chanel 18   : slider
 *      PIOA     6  7  chanel 19
 *      PIOA     8  9  chanel 20
 *      PIOA    10 11  chanel 21
 *      PIOA    12 13  chanel 22
 *      PIOA    14 15  chanel 23
 *      PIOA    16 17  chanel 24
 *      PIOA    18 19  chanel 25
 *      PIOA    20 21  chanel 26
 *      PIOA    22 23  chanel 27
 *      PIOA    24 25  chanel 28
 *      PIOA    26 27  chanel 29
 *      PIOA    28 29  chanel 30
 *      PIOA    30 31  chanel 31
 *
 */
static void config_sensors(void)
{
    PMC_EnablePeripheral(ID_PIOA);
    PMC_EnablePeripheral(ID_PIOC);

    /* Valid: SNS = PC22, SNSK = PC23 */
    qt_enable_key( CHANNEL_11, AKS_GROUP_1, 10u, HYST_6_25 );

    /* Up: SNS = PC24, SNSK = PC25 */
    qt_enable_key( CHANNEL_12, AKS_GROUP_1, 10u, HYST_6_25 );

    /* Down: SNS = PC26, SNSK = PC27 */
    qt_enable_key( CHANNEL_13, AKS_GROUP_1, 10u, HYST_6_25 );

    /* Left: SNS = PC28, SNSK = PC29 */
    qt_enable_key( CHANNEL_14, AKS_GROUP_1, 10u, HYST_6_25 );

    /* Right: SNS = PC30, SNSK = PC31 */
    qt_enable_key( CHANNEL_15, AKS_GROUP_1, 10u, HYST_6_25 );

    /* Slider: PA0, PA1, PA2, PA3, PA4, PA5 */
    qt_enable_slider( CHANNEL_16, CHANNEL_18, AKS_GROUP_1, 16u, HYST_6_25, RES_8_BIT, 0u  );

}
/**
 *  \brief  Handler for Sytem Tick interrupt.
 *
 *  Set systick event flag
 */
void SysTick_Handler(void)
{
    /* set flag: it's time to measure touch */
    time_to_measure_touch = 1u;

    /* update the current time */
    current_time_ms_touch += qt_measurement_period_msec;
}

/**
 *  \brief  configure timer ISR to fire regularly
 *
 *  Configure Timer Counter 0 to generate an interrupt every 25ms.
 */
static void init_timer_isr( void )
{
    printf("Configure sys tick to get %u ms tick period.\r\n", (unsigned int)qt_measurement_period_msec);
    if (SysTick_Config((BOARD_MCK / 1000) * qt_measurement_period_msec)) {
        printf("-F- Systick configuration error\r\n");
    }
}

/**
 *  \brief   initialise host app, pins, watchdog, etc
 *
 *
 */
static void init_system( void )
{
    /* Disable watchdog */
    WDT_Disable(WDT);

    /* Output example information */
    printf("\r\n");
    printf("-- QTouch Project--\r\n");
    printf("-- %s\r\n", BOARD_NAME);
    printf("-- Compiled: %s %s --\r\n", __DATE__, __TIME__);
}
