/*
 * my_adc.c
 *
 *  Created on: 31.01.2020
 *      Author: Adrian
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <string.h>

/* Custom Includes */
#include "my_adc.h"

/* Defines */
//#define ONLY_MOST_IMPORTANT_ADC_CHANNELS
#ifdef LEAN_ADC_MEASUREMENT
#define ADC_INT_N ADC_INT6
#define ADC_MEM_END ADC_MEM6
#else
#define ADC_INT_N ADC_INT10
#define ADC_MEM_END ADC_MEM10
#endif

#define NUM_ANALOG_SIGNALS  11  // 11, if all ADC channels are measured
#define NUM_ANALOG_SIGNALS_LEAN 5
#define NUM_I_SIGNALS       6   // Current (I) signals
#define NUM_V_SIGNALS       5   // Voltage (V) signals
#ifndef NUM_MEAS_AVG
#define NUM_MEAS_AVG    25  // take 25 measurements for averaging, if not specified in header file
#endif
#define R_UPPER_1   12.4    // kOhm
#define R_UPPER_2   23.2    // kOhm
#define R_LOWER     23.2    // kOhm
typedef uint32_t adcSum_t;

#define V_REF_EXT   (5.0f * R_LOWER / (R_UPPER_1 + R_LOWER))     // ~ 3.258 V
#define V_REF_INT   3.33f  // also a reasonable value: 3.326f. 3.3 V (3.295V measured, but in order to correct gain error use 3.33V)


/* Global variables, but not public */
static uint16_t adcRawResults[NUM_ANALOG_SIGNALS];
static adcSum_t adcRawSums[NUM_ANALOG_SIGNALS];
static uint16_t adcAvgs[NUM_ANALOG_SIGNALS];
static float adcValues[NUM_ANALOG_SIGNALS] = {0};
static float zeroCurrentVoltages[NUM_I_SIGNALS];
//static const float k1 = R_LOWER / (R_UPPER_1 + R_LOWER);    // gain of voltage divider 1 (~ 0.652), used for reference voltage and most measurement signals
//static const float k2 = R_LOWER / (R_UPPER_2 + R_LOWER);    // gain of voltage divider 2 (0.5), used for V_BUS (voltage PV bus) and V_PDU_5V
static const float k1_rev = (R_UPPER_1 + R_LOWER) / R_LOWER;    // gain of voltage divider 1 (~ 0.652), used for reference voltage and most measurement signals
static const float k2_rev = (R_UPPER_2 + R_LOWER) / R_LOWER;    // gain of voltage divider 2 (0.5), used for V_BUS (voltage PV bus) and V_PDU_5V
static float voltsPerCount;     // = V_REF_INT / ((1<<14)-1);
static volatile bool adc_newMeas_Available = false;    // flag to indicate that new ADC measurements are available
static volatile int16_t numMeas = 0;           // remaining number of Measurements for average value
//static uint8_t dummy = 0;


/* Functions */
void adc_init(uint8_t voltage_reference, uint32_t clockSource,
              uint32_t clockPredivider, uint32_t clockDivider) {
    uint32_t vref_type;
    switch (voltage_reference) {
    case AVCC:
        voltsPerCount = V_REF_INT / ((1<<14)-1);
        vref_type = ADC_VREFPOS_AVCC_VREFNEG_VSS;
        break;
    case VeREF:
        voltsPerCount = V_REF_EXT / ((1<<14)-1);
        vref_type = ADC_VREFPOS_EXTBUF_VREFNEG_EXTNEG;
        break;
    case VeREF_UNBUFFERED:
        voltsPerCount = V_REF_EXT / ((1<<14)-1);
        vref_type = ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG;
        break;
    default:
        voltsPerCount = V_REF_INT / ((1<<14)-1);
        vref_type = ADC_VREFPOS_AVCC_VREFNEG_VSS;
        break;
    }
    /* Zero-fill buffers */
    memset(adcRawResults, 0, NUM_ANALOG_SIGNALS * sizeof(uint16_t));
    memset(adcRawSums, 0, NUM_ANALOG_SIGNALS * sizeof(adcSum_t));
    memset(adcAvgs, 0, NUM_ANALOG_SIGNALS * sizeof(uint16_t));
    ADC14_enableModule();
    /* Initializing ADC (MCLK/1/1/no internal component mapping) */
    ADC14_initModule(clockSource, clockPredivider, clockDivider, ADC_NOROUTE);
    ADC14_setResolution(ADC_14BIT);     // 14 bit is default

    /* Configuring GPIOs for Analog In */
    /* Setting up GPIO pins as analog inputs (and references) */
    // P5.6: VREF+, P5.7: VREF-
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                               GPIO_PIN6 | GPIO_PIN7,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
    // P5.5(A0)=V_BUS, P5.4(A1)=I_PV, P5.2(A3)=V_PDU_5V, P5.1(A4)=I_LOAD, P5.0(A5)=V_PDU_3V
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                               GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN2 | GPIO_PIN1 | GPIO_PIN0,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
    // P4.6(A7)=V_BAT1, P4.4(A9)=I_PDU_5V, P4.3(A10)=I_BAT1, P4.2(A11)=I_PDU_3.3V, P4.0(A13)=I_BAT2
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN6 | GPIO_PIN4 | GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN0,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
    // P6.1(A14)=V_BAT2
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory: ADC_MEM0 - ADC_MEM10, no repeat (single sample)
     * with AVCC voltage reference, ~ 3.3 V */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM_END, false);
    ADC14_configureConversionMemory(ADC_MEM0,
                                    vref_type,
                                    ADC_INPUT_A0, false);   // V_BUS
    ADC14_configureConversionMemory(ADC_MEM1,
                                    vref_type,
                                    ADC_INPUT_A1, false);   // I_PV
    ADC14_configureConversionMemory(ADC_MEM2,
                                    vref_type,
                                    ADC_INPUT_A4, false);   // I_LOAD
    ADC14_configureConversionMemory(ADC_MEM3,
                                    vref_type,
                                    ADC_INPUT_A7, false);   // V_BAT1
    ADC14_configureConversionMemory(ADC_MEM4,
                                    vref_type,
                                    ADC_INPUT_A14, false);   // V_BAT2
    ADC14_configureConversionMemory(ADC_MEM5,
                                    vref_type,
                                    ADC_INPUT_A10, false);   // I_BAT1
    ADC14_configureConversionMemory(ADC_MEM6,
                                    vref_type,
                                    ADC_INPUT_A13, false);   // I_BAT2
#ifndef LEAN_ADC_MEASUREMENT
    ADC14_configureConversionMemory(ADC_MEM7,
                                    vref_type,
                                    ADC_INPUT_A3, false);   // V_PDU_5V
    ADC14_configureConversionMemory(ADC_MEM8,
                                    vref_type,
                                    ADC_INPUT_A5, false);   // V_PDU_3V3
    ADC14_configureConversionMemory(ADC_MEM9,
                                    vref_type,
                                    ADC_INPUT_A9, false);   // I_PDU_5V
    ADC14_configureConversionMemory(ADC_MEM10,
                                    vref_type,
                                    ADC_INPUT_A11, false);   // I_PDU_3V3
#endif
    /* Enable Interrupts */
    ADC14_enableInterrupt(ADC_INT_N);       // Interrupt for last memory slot enabled (conversion of last ADC channel complete and written to MEM[N])
    Interrupt_enableInterrupt(INT_ADC14);   // Interrupt for Precision ADC enabled
    Interrupt_enableMaster();
    /* Set up sample timer to automatically step through the ADC sequence */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    /* Enable conversion (but not yet started, this is done by triggering the conversion) */
    ADC14_enableConversion();
}

void adc_convertToPhysicalValues(float* rawValues, float* convertedValues) {
    /* Convert to physical values:
     * - Voltages must be converted using the appropriate factor according to the voltage divider used.
     * - ACS723-05AB: Current Sensor: 400 mV/A, zero current voltage = 2.5 V. Multiplication by 2500 is the same as division by 0.0004, but much faster.
     * - LTC3112: uses internal current sensor to determine output current: 1 V/A, zero current voltage = 0 V.
     *  */
    convertedValues[0] =   rawValues[0] * k2_rev;                    // [V]  PV_BUS, (using another voltage divider, so multiply by k)
    convertedValues[1] =  (rawValues[1] - 1.6263f) * k1_rev * 2500;  // [mA] I_PV (current sensor)
    convertedValues[2] =  (rawValues[2] - 1.6295f) * k1_rev * 2500;  // [mA] I_LOAD (current sensor)
    convertedValues[3] =   rawValues[3] * k1_rev;                    // [V]  V_BAT1
    convertedValues[4] =   rawValues[4] * k1_rev;                    // [V]  V_BAT2
    convertedValues[5] =  (rawValues[5] - 1.6243f) * k1_rev * 2500;  // [mA] I_BAT1 (current sensor)
    convertedValues[6] =  (rawValues[6] - 1.6260f) * k1_rev * 2500;  // [mA] I_BAT2 (current sensor)
//    convertedValues[0] =   rawValues[0] * k2_rev;                   // [V]  PV_BUS, (using another voltage divider, so multiply by k)
//    convertedValues[1] =  (rawValues[1] * k1_rev - 2.496) * 2500;   // [mA] I_PV (current sensor)
//    convertedValues[2] =  (rawValues[2] * k1_rev - 2.501f) * 2500;  // [mA] I_LOAD (current sensor)
//    convertedValues[3] =   rawValues[3] * k1_rev;                   // [V]  V_BAT1
//    convertedValues[4] =   rawValues[4] * k1_rev;                   // [V]  V_BAT2
//    convertedValues[5] =  (rawValues[5] * k1_rev - 2.493f) * 2500;  // [mA] I_BAT1 (current sensor)
//    convertedValues[6] =  (rawValues[6] * k1_rev - 2.495f) * 2500;  // [mA] I_BAT2 (current sensor)
#ifndef LEAN_ADC_MEASUREMENT
    convertedValues[7] =   rawValues[7] * k2_rev;                   // [V]  V_PDU_5V (using another voltage divider, so multiply by k)
    convertedValues[8] =   rawValues[8] * k1_rev;                   // [V]  V_PDU_3.3V
    convertedValues[9] =   rawValues[9] * k1_rev * 1000;            // [mA] I_PDU_5V (internal current sensor of IC LTC3112)
    convertedValues[10] = rawValues[10] * k1_rev * 1000;            // [mA] I_PDU3.3V (internal current sensor of IC LTC3112))
#endif
}

void adc_startMeasurement(void) {
    memset(adcRawResults, 0, NUM_ANALOG_SIGNALS * sizeof(uint16_t));
    memset(adcRawSums, 0, NUM_ANALOG_SIGNALS * sizeof(adcSum_t));
    numMeas = NUM_MEAS_AVG;             // remaining number of Measurements
    ADC14->CTL0 |= ADC14_CTL0_SC;   // start conversion, same as ADC14_toggleConversionTrigger(), but faster
}

bool adc_measurementFinished(void) {
    return adc_newMeas_Available;
}

void adc_readMeasurements(float* bufferArrayPointer) {
    adc_newMeas_Available = false;
    for (uint8_t i = 0; i < NUM_ANALOG_SIGNALS; i++) {
        adcAvgs[i] = adcRawSums[i] / NUM_MEAS_AVG;      // shift by 4 = division by 16 (but is not faster, same amount of clock cycles needed)
        adcValues[i] = adcAvgs[i] * voltsPerCount;      // transform to measurement voltage (voltage @ ADC input)
        bufferArrayPointer[i] = adcValues[i];
    }
}

void adc_resetZeroCurrentVoltageValues(void) {
    for (uint8_t i = 0; i < NUM_I_SIGNALS; i++) {
        zeroCurrentVoltages[i] = 0;
    }
}

void adc_getZeroCurrentVoltages(float* rawValues, float* convertedValues, uint16_t period_ms, uint16_t tau_ms) {
    float fraq = (double)period_ms/(double)tau_ms;  // assuming period is 0.01 sec (=10ms)
    for (uint8_t i = 0; i < NUM_I_SIGNALS; i++) {
        zeroCurrentVoltages[i] =  fraq * rawValues[i] + (1-fraq)*zeroCurrentVoltages[i];
        convertedValues[i] = zeroCurrentVoltages[i];
    }
}



/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM10. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
//    ADC14_clearInterruptFlag(status);
    if(status & ADC_INT_N)
    {
        ADC14_getMultiSequenceResult(adcRawResults);
        for (uint8_t i = 0; i < NUM_ANALOG_SIGNALS; i++) {
            adcRawSums[i] += adcRawResults[i];
        }
        if (--numMeas > 0) {
            ADC14->CTL0 |= ADC14_CTL0_SC;   // start conversion, same as ADC14_toggleConversionTrigger(), but faster
        } else {
            adc_newMeas_Available = true;
        }
    }
}

