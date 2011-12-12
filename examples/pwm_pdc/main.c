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
 * \page pwm_pdc PWM with PDC Example
 *
 * \section Purpose
 *
 * This example demonstrates a simple configuration of three PWM channels to
 * generate variable duty cycle signals. The update of the duty cycle values
 * is made automatically by the Peripheral DMA Controller (PDC).
 * This will cause three LEDs on the evaluation kit to glow repeatedly.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * Three PWM channels (channel #0, #1 and #2) are configured to generate
 * a 50Hz PWM signal. The update of the duty cycle values is made
 * automatically by the PDC.
 *
 * When launched, this program displays a menu on the terminal, enabling the user
 * to choose between several options:
 *   - Set update period for syncronous channel
 *   - Set dead time
 *   - Set output override
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please refer to the
 * <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
 * SAM-BA User Guide</a>, the
 * <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
 * GNU-Based Software Development</a> application note or to the
 * <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6132.pdf">
 * IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Depending on the board being used, two LEDs will start glowing repeatedly.
 * -# Select one or more options to set the configuration of PWM channel.
 *
 * \section References
 * - pwm_pdc/main.c
 * - pwmc.c
 * - pwmc.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the pwm_pdc example.
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

/** PWM frequency in Hz. */
#define PWM_FREQUENCY               50

/** Maximum duty cycle value. */
#define MAX_DUTY_CYCLE              50
/** Minimum duty cycle value. */
#define MIN_DUTY_CYCLE              0

/** Duty cycle buffer length for three channels */
#define DUTY_BUFFER_LENGTH          (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE + 1) * 3

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pio pins to configure. */
static const Pin pins[] = {
    PIN_PWM_LED0,
    PIN_PWM_LED1,
    PIN_PWM_LED2
};

/** duty cycle buffer for PDC transfer */
uint16_t dutyBuffer[DUTY_BUFFER_LENGTH];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Display menu.
 */
static void _DisplayMenu( void )
{
    printf("\n\r");
    printf("===============================================================\n\r");
    printf("Menu: press a key to change the configuration.\n\r");
    printf("===============================================================\n\r");
    printf("  u : Set update period for syncronous channel \n\r");
    printf("  d : Set dead time\n\r");
    printf("  o : Set output override\n\r");
    printf("\n\r");
}

/**
 * \brief Get 2 digit numkey.
 *
 * \return numkey value
 */
static uint32_t _GetNumkey2Digit( void )
{
    uint32_t numkey;
    uint8_t key1, key2;

    printf("\n\rEnter 2 digits : ");
#if defined (  __GNUC__  )
        fflush(stdout);
#endif
    key1 = UART_GetChar();
    printf("%c", key1);
    key2 = UART_GetChar();
    printf("%c", key2);
    printf("\n\r");

    numkey = (key1 - '0')*10 + (key2 - '0');

    return numkey;
}

/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM_IrqHandler(void)
{
    uint32_t isr2 = PWM->PWM_ISR2;

    if ((isr2 & PWM_ISR2_ENDTX) == PWM_ISR2_ENDTX) {

        PWMC_WriteBuffer(PWM, dutyBuffer, DUTY_BUFFER_LENGTH);
    }
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for PWM with PDC example.
 *
 * Outputs a PWM on LED1 & LED2 & LED3 to makes it fade in repeatedly.
 * Channel #0, #1, #2 are linked together as synchronous channels, so they have
 * the same source clock, the same period, the same alignment and
 * are started together. The update of the duty cycle values is made
 * automatically by the Peripheral DMA Controller (PDC).
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
    uint32_t i;
    uint8_t key;
    int32_t numkey;

    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- PWM with PDC Example %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    /* PIO configuration */
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    /* Enable PWMC peripheral clock */
    PMC_EnablePeripheral(ID_PWM);

    /* Set clock A to run at PWM_FREQUENCY * MAX_DUTY_CYCLE (clock B is not used) */
    PWMC_ConfigureClocks(PWM_FREQUENCY * MAX_DUTY_CYCLE, 0, BOARD_MCK);

    /* Configure PWMC channel for LED0 (left-aligned, enable dead time generator) */
    PWMC_ConfigureChannelExt( PWM,
                              CHANNEL_PWM_LED0,  /* channel */
                              PWM_CMR_CPRE_CKA,  /* prescaler */
                              0,                 /* alignment */
                              0,                 /* polarity */
                              0,                 /* countEventSelect */
                              PWM_CMR_DTE,       /* DTEnable */
                              0,                 /* DTHInverte */
                              0 );               /* DTLInverte */

    PWMC_SetPeriod(PWM, CHANNEL_PWM_LED0, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(PWM, CHANNEL_PWM_LED0, MIN_DUTY_CYCLE);
    PWMC_SetDeadTime(PWM, CHANNEL_PWM_LED0, 5, 5);

    /* Configure PWMC channel for LED1 */
    PWMC_ConfigureChannelExt( PWM,
                              CHANNEL_PWM_LED1,  /* channel */
                              PWM_CMR_CPRE_CKA,  /* prescaler */
                              0,                 /* alignment */
                              0,                 /* polarity */
                              0,                 /* countEventSelect */
                              0,                 /* DTEnable */
                              0,                 /* DTHInverte */
                              0 );               /* DTLInverte */

    PWMC_SetPeriod(PWM, CHANNEL_PWM_LED1, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(PWM, CHANNEL_PWM_LED1, MIN_DUTY_CYCLE);

    /* Configure PWMC channel for LED2 */
    PWMC_ConfigureChannelExt( PWM,
                              CHANNEL_PWM_LED2,  /* channel */
                              PWM_CMR_CPRE_CKA,  /* prescaler */
                              0,                 /* alignment */
                              0,                 /* polarity */
                              0,                 /* countEventSelect */
                              0,                 /* DTEnable */
                              0,                 /* DTHInverte */
                              0 );               /* DTLInverte */

    PWMC_SetPeriod(PWM, CHANNEL_PWM_LED2, MAX_DUTY_CYCLE);
    PWMC_SetDutyCycle(PWM, CHANNEL_PWM_LED2, MIN_DUTY_CYCLE);

    /* Set synchronous channels, update mode = 2 */
    PWMC_ConfigureSyncChannel(PWM,
                              (1 << CHANNEL_PWM_LED0) |
                              (1 << CHANNEL_PWM_LED1) |
                              (1 << CHANNEL_PWM_LED2),
                              PWM_SCM_UPDM_MODE2, //  (PWMC) Automatic write of data and automatic trigger of the update
                              0,
                              0);

    /* Set Synchronous channel update period value */
    PWMC_SetSyncChannelUpdatePeriod(PWM, PWM_SCUP_UPR(0xF));

    /* Configure interrupt for PDC transfer */
    NVIC_DisableIRQ(PWM_IRQn);
    NVIC_ClearPendingIRQ(PWM_IRQn);
    NVIC_SetPriority(PWM_IRQn, 0);
    NVIC_EnableIRQ(PWM_IRQn);
    PWMC_EnableIt(PWM, 0, PWM_IER2_ENDTX);

    /* Set override value to 1 on PWMH0, others is 0. */
    PWMC_SetOverrideValue(PWM, PWM_OOV_OOVH0);

    /* Fill duty cycle buffer for channel #0, #1 and #2 */
    /* For Channel #0, #1, #2 duty cycle are from MIN_DUTY_CYCLE to MAX_DUTY_CYCLE */
    for (i = 0; i < DUTY_BUFFER_LENGTH/3; i++) {
        dutyBuffer[i*3]   = (i + MIN_DUTY_CYCLE);
        dutyBuffer[i*3+1] = (i + MIN_DUTY_CYCLE);
        dutyBuffer[i*3+2] = (i + MIN_DUTY_CYCLE);
    }

    /* Define the PDC transfer */
    PWMC_WriteBuffer(PWM, dutyBuffer, DUTY_BUFFER_LENGTH);

    /* Enable syncronous channels by enable channel #0 */
    PWMC_EnableChannel(PWM, CHANNEL_PWM_LED0);

    while (1) {
        _DisplayMenu();
        key = UART_GetChar();

        switch (key) {
            case 'u':
                printf("Input update period between %d to %d.\n\r", 0, PWM_SCUP_UPR_Msk);
                numkey = _GetNumkey2Digit();
                if(numkey <= PWM_SCUP_UPR_Msk) {

                    /* Set synchronous channel update period value */
                    PWMC_SetSyncChannelUpdatePeriod(PWM, numkey);
                    printf("Done\n\r");
                } else {

                    printf("Invalid input\n\r");
                }
                break;
            case 'd':
                printf("Input dead time for channel #0 between %d to %d.\n\r",
                    MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
                numkey = _GetNumkey2Digit();
                if(numkey >= MIN_DUTY_CYCLE && numkey <= MAX_DUTY_CYCLE) {

                    /* Set synchronous channel update period value */
                    PWMC_SetDeadTime(PWM, CHANNEL_PWM_LED0, numkey, numkey);
                    /* Update synchronous channel */
                    PWMC_SetSyncChannelUpdateUnlock(PWM);
                    printf("Done\n\r");
                } else {

                    printf("Invalid input\n\r");
                }
                break;
            case 'o':
                printf("0: Disable override output on channel #0\n\r");
                printf("1: Enable override output on channel #0\n\r");
                key = UART_GetChar();

                if (key == '1') {

                    PWMC_EnableOverrideOutput(PWM, PWM_OSSUPD_OSSUPH0 | PWM_OSSUPD_OSSUPL0, 1);
                    printf("Done\n\r");
                } else if (key == '0') {

                    PWMC_DisableOverrideOutput(PWM, PWM_OSSUPD_OSSUPH0 | PWM_OSSUPD_OSSUPL0, 1);
                    printf("Done\n\r");
                }
                break;
            default:
                printf("Invalid input\n\r");
                break;
        }
    }
}

