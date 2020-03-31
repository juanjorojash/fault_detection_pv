/*
 * pv_sim.c
 *
 *  Created on: 17.03.2020
 *      Author: Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdio.h>
#include <math.h>

/* Custom Includes */
#include "pv_sim.h"
#include "my_uart.h"
#include "my_timing.h"
#include "my_utils.h"

/* Defines */
#define DUTY_CYCLE_MAX_VALUE 511

/* Global variables */
static bool sweeping = false;
uint16_t V_bat_sim = V_BAT_SIM_DEFAULT;          // initial value of V_bat_sim (only for simulation purposes needed)
uint16_t dutyCycle = DUTYCYCLE_DEFAULT;

/* Functions */

/* pv_sim()
 * can be used instead of adc_routines() when no PVA is connected to PCB.
 * pv_sim() will simulate the I-V characteristics for the current given
 * duty cycle and returns simulated values to the ADC measurement array.
 */
void pv_sim_measurement() {

    double V_BAT = V_bat_sim/1000.0;
    double dutycycle = (double)dutyCycle;
    double D = dutycycle/(double)DUTY_CYCLE_MAX_VALUE;

    double V_PV = 1/D * V_BAT;

    // 2s3p
//    double Isc = 1512;
    double Isc = 520;
    double Voc = PV_MAX_VOLTAGE;
    double potence = 31;
    double coefficient = (-1 * Isc) / pow(Voc, potence);
    double I_PV_mA = coefficient * pow(V_PV, potence) + Isc;
    I_PV_mA = gaussrand(I_PV_mA, 1.8);  // mean, stddev

    physicalValues[0]  = (float)V_PV;
    physicalValues[1]  = (float)I_PV_mA;
    physicalValues[3]  = (float)V_BAT;
    physicalValues[4]  = (float)V_BAT;
}

void pv_sim_resetDutycycle(void) {
    dutyCycle = DUTYCYCLE_DEFAULT;
}

void pv_sim_stepDutycycle(uint16_t cmdUart_dutyCycleStep, uint16_t cmdCtrl_dutyCycleStep) {
    if (cmdUart_dutyCycleStep) {    // UART command overrules MPPT command for duty cycle set point
        dutyCycle += cmdUart_dutyCycleStep;
    } else {
        dutyCycle += cmdCtrl_dutyCycleStep;
    }
}

void pv_sim_trigger_sweep(void) {
    dutyCycle = (uint16_t)(DUTY_CYCLE_MAX_VALUE*0.99);    // this duty cycle is NOT sent over SPI. Via SPI, only the duty cycle STEP can be sent.
    sweeping = true;
}

void pv_sim_sweep(void) {
    /* Set up variables */
    /* Measurement values */
    float V_PV    = physicalValues[0];
    float I_PV_mA = physicalValues[1];
    float V_BAT2  = physicalValues[4];
    float P_PV_mW = V_PV * I_PV_mA;

    /* Calculated values */
    P_PV_mW = V_PV * I_PV_mA;

    dutyCycle--;
    if (dutyCycle < DUTY_CYCLE_MAX_VALUE * V_BAT2/PV_MAX_VOLTAGE) {   // 5.024 is the maximum voltage of the PVA
        sweeping = false;
    }
}

bool pv_sim_sweeping_active(void) {
    return sweeping;
}

