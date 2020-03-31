/*
 * my_utils.h
 *
 *  Created on: 21.01.2020
 *      Author: Adrian
 */

#ifndef MY_UTILS_H_
#define MY_UTILS_H_

#include <stdint.h>

/* Defines */
#ifndef DUTYCYCLE_DEFAULT
#define DUTYCYCLE_DEFAULT   461
#endif
#ifndef PV_MAX_VOLTAGE      // must not be higher than 6.5 V, because ADC input channel uses voltage divider gain 0.5 (max voltage: 3.3V)
#define PV_MAX_VOLTAGE 4.8  // max voltage of PV array
#endif

#define X_TEST_NUM 100  // required for typedef iv_testData_t

/* Typedefs */
typedef struct {    // iv_testData_t: struct to hold test measurement data for calculating stddev and mean
    uint16_t idx;
    bool finished;
    float i_data[X_TEST_NUM];
    float i_stddev;
    float i_mean;
    float v_data[X_TEST_NUM];
    float v_stddev;
    float v_mean;
} iv_testData_t;

/* Variable declarations (externally defined variables) */
extern iv_testData_t iv_test;   // automatically initialized to 0, because it is a global variable. Init value not crucial anyway.


/* Functions */
float gaussrand(double mean, double stddev);
float stddev(float *data, uint16_t num_elements);
float mean(float *data, uint16_t num_elements);
float lp_filtering(float input, float outputPrevious, float tau_ms, float dt_ms);

#endif /* MY_UTILS_H_ */
