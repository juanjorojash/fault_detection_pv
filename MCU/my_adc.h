/*
 * my_adc.h
 *
 *  Created on: 31.01.2020
 *      Author: Adrian
 */

#ifndef MY_ADC_H_
#define MY_ADC_H_

#include <stdbool.h>

#define LEAN_ADC_MEASUREMENT

#define NUM_MEAS_AVG        100    // (max. 32767) number of measurements for averaging

#define AVCC                1 //ADC_VREFPOS_AVCC_VREFNEG_VSS
#define VeREF               2 //ADC_VREFPOS_EXTBUF_VREFNEG_EXTNEG
#define VeREF_UNBUFFERED    3 //ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG

void adc_init(uint8_t voltage_reference, uint32_t clockSource,
              uint32_t clockPredivider, uint32_t clockDivider);
void adc_startMeasurement(void);
void adc_readMeasurements(float* bufferArrayPointer);
void adc_convertToPhysicalValues(float* rawValues, float* convertedValues);
bool adc_measurementFinished(void);

void adc_getZeroCurrentVoltages(float* rawValues, float* convertedValues, uint16_t period_ms, uint16_t tau_ms);
void adc_resetZeroCurrentVoltageValues(void);



#endif /* MY_ADC_H_ */
