/*
 * my_framework.h
 *
 *  Created on: 09.03.2020
 *      Author: Adrian
 */

#ifndef MY_FRAMEWORK_H_
#define MY_FRAMEWORK_H_

//#include <stdint.h>
//#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
//#include <limits.h>
//#include "my_timing.h"
#include "my_uart.h"
#include "my_spi.h"
#include "my_utils.h"


/* Defines */
#define FINISHED true
#define NOT_FINISHED false
// SPI Configuration
#define CPOL 0
#define CPHA 0
#define SPI_FREQ 1000000     // clock speed: 1 MHz

/* Typedefs */
typedef struct {    // time_slot_t (currently not used)
    const uint16_t period;
    const uint16_t phase;
    volatile uint16_t phase_counter;
    volatile bool periodFlag;    // recommended to initialize the flag with false
} time_slot_t;
typedef struct {    // uartPrint_t: holds flags for printing several data via UART
    bool cpuLoad;               // flag to enable printing of CPU load
    bool plots;                 // flag to enable printing of I, V, P for plotting through Serial Port Plotter
    bool plots_fast;            // flag to enable fast printing of I, V, dutycycle for plotting through Serial Port Plotter
    bool plots_vipPV;           // flag to enable printing of PV V,I,P for plotting through Serial Port Plotter
    bool plots_vipPVf;          // flag to enable printing of PV V,I,P filtered for plotting through Serial Port Plotter
    bool plots_vipBAT2;         // flag to enable printing of BAT2 V,I for plotting through Serial Port Plotter
    bool plots_vipBAT2f;        // flag to enable printing of BAT2 V,I filtered for plotting through Serial Port Plotter
    bool adc_values;            // flag to enable printing of ADC measurement values (measured voltages)
    bool measurement_values;    // flag to enable printing of physical measurement values (measured voltages converted to physical values)
    bool filtered_inputSignals; // flag to enable printing of filtered physical measurement values
    bool current_meas_noise;    // flag to enable printing of noise (current measurement)
    bool adc_timing;            // flag to enable printing of ADC routine timing
    bool ctrl_timing;           // flag to enable printing of MPPT routine timing
    bool uart_timing;           // flag to enable printing of UART transmit routine timing
    bool dutycycle_cmd;         // flag to enable printing of duty cycle command (sent to SPI and simulate value)
    bool stddev;           // flag to enable printing of standard deviation of I_PV_mA
    bool zeroI_Vmeas;           // flag to enable zero current voltage measurement for current measurement ICs
} uartPrint_t;
typedef struct {    // interval_flag_t: interval flags for uart tx routines and control loop
    bool uart_slow;         // interval flag for sending information via UART to PC
    bool uart_fast;         // interval flag for sending information via UART to PC
    bool uart_superfast;    // interval flag for sending information via UART to PC
    bool control_loop;      // flag to indicate that in this period, the control loop (MPPT, ...) shall be executed
} interval_flag_t;
typedef struct {    // cycles_t: cycles (time measurement) variables (joined in a struct)
    uint32_t adc;
    uint32_t ctrl;
    uint32_t idle;
    uint32_t busy;
    uint32_t uart;
    uint32_t uart_tmp;
} cycles_t;
typedef enum {      // statemachine_t: different states
    INIT, TRIGGER_ADC, READ_INPUTS, SET_DUTYCYCLE, WRITE_DUTYCYCLE, WRITE_UART, WAIT_STATE
} statemachine_t;

/* Global variables */
extern statemachine_t statemachine;
// UART variables
extern char uart_rxString[];    // incoming UART data
extern char uart_txString[];    // outgoing UART data
extern volatile cycles_t cycles;    // cycles measurements (timing measurement values)
// ADC variables
extern float adcResults[];
extern float physicalValues[];
extern float zeroCurrentVoltagesRaw[];
extern float zeroCurrentVoltages[];

/* Functions */
void cmd_change_cpu_clk(void);
void cmd_change_spi_clk(void);
void cmd_change_uart_baudrate(void);
void calc_mean_and_stddev(void);
void system_init(void);
void systick_init(void);
void launchpad_leds_init(void);


#endif /* MY_FRAMEWORK_H_ */
