/*
 * my_utils.c
 *
 *  Created on: 21.01.2020
 *      Author: Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Custom Includes */
#include "my_utils.h"
#include "my_uart.h"
#include "my_timing.h"

/* Defines */
#define DUTY_CYCLE_MAX_VALUE 511

/* Global variables */
iv_testData_t iv_test;   // automatically initialized to 0, because it is a global variable. Init value not crucial anyway.

/* Functions */

float gaussrand(double mean, double stddev) {
    static double v1, v2, s;
    static int phase = 0;
    double tmp;

    if(phase == 0) {
        do {
            double u1 = (double)rand() / RAND_MAX;
            double u2 = (double)rand() / RAND_MAX;

            v1 = 2 * u1 - 1;
            v2 = 2 * u2 - 1;
            s = v1 * v1 + v2 * v2;
        } while(s >= 1 || s == 0);

        tmp = v1 * sqrt(-2 * log(s) / s);
    } else
        tmp = v2 * sqrt(-2 * log(s) / s);

    phase = 1 - phase;

    return (float)(tmp * stddev + mean);
}

float stddev(float *data, uint16_t num_elements) {
    double sum = 0.0;
    double mean;
    double term0 = 0.0;
    for (uint16_t i = 0; i < num_elements; i++) {
        sum += (double)(data[i]);
    }
    mean = sum / num_elements;
    for (uint16_t i = 0; i < num_elements; i++)
        term0 += pow((double)(data[i]) - mean, 2);
    return (float)(sqrt(term0 / num_elements));
}

float mean(float *data, uint16_t num_elements) {
    double sum = 0.0;
    for (uint16_t i = 0; i < num_elements; i++) {
        sum += (double)(data[i]);
    }
    return (float)(sum / num_elements);
}

/* lp_filtering()
 * Filter input signal with 1st order low-pass filter.
 * Cut-off frequency is 1/(2*pi*tau).
 *   tau_ms:    filter time constant (stationary value approx. after 5 tau)
 *   dt_ms:     sampling interval
 *   input:     raw value (to be filtered)
 *   outputPreiouvs: filtered value of last filtering
 */
float lp_filtering(float input, float outputPrevious, float tau_ms, float dt_ms) {
    float k = dt_ms/tau_ms;
    return k * input + (1-k) * outputPrevious;
}

