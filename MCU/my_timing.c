/*
 * my_timing.c
 *
 *  Created on: 14.01.2020
 *      Author: Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdbool.h>

/* Custom Includes */
#include "my_timing.h"


/* Defines */

/* Global variables */


/* Functions */
void t_init(void) {
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
                       TIMER32_FREE_RUN_MODE);
    Timer32_startTimer(TIMER32_0_BASE, false);
}
