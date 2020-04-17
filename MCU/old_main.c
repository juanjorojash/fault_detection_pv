/*
 * Program for applying a fault detection algorithm to a PV panel
 * =====================================================================
 * 
 *
 *  Created on: April 2, 2020
 *      Author: Juan J. Rojas
 * 
 * Libraries developed by Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

static char typeString[] = "Embedded fault detection for PV panel\n";

/* Defines that have to be defined before including custom includes */
#define PV_SIM        // uncomment to activate simulated solar cell
#define PV_MAX_VOLTAGE 4.8
#define DUTYCYCLE_DEFAULT   461     // duty cycle default value (as configured in slave MCUs)
// Sampling and Periods
#define SAMPLING_INTERVAL_MS        50      // dt = sampling time of ADC
#define CTRL_LOOP_INTERVAL_MS       200     // default time interval for executing MPPT (may be equal to SAMPLING_TIME or integer multiple of it)
#define TAU_FILTER_CONSTANT_MS      1000    // default filter constant (may be changed through UART command)
#define UART_SLOW_INTERVAL_MS       2000    // 2000 ms period is enough for about 10000 characters to be sent via UART
#define UART_FAST_INTERVAL_MS       400     // 100 ms period is enough for about 500 characters to be sent via UART (if 57600 baud/s)
#define UART_SUPERFAST_INTERVAL_MS  50
#define LONGEST_INTERVAL_MS         2000    // must be a multiple of or equal to the slowest interval (used for resetting the timer)
// System Parameters
#define BAT_SOC_100 4.1     // end-of-charge voltage: 4.2 V
#define BAT_SOC_80  3.8     // ~ 0.8 of assumed linear relation between 2.7 and 4.1 Volt
#define BAT_SOC_0   2.7     // end-of-discharge voltage: 2.5 V (end point voltage)
#define BAT_CURRENT_THRESHOLD   20      // current below this value is quasi zero
#define PV_DEFAULT_SETPOINT_VOLTAGE 4.5 // default voltage to move to when fixed voltage set point is actived

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
/* Custom Includes */
#include "my_uart.h"
#include "my_timing.h"
#include "my_spi.h"
#include "my_adc.h"
#include "my_utils.h"
#include "pv_sim.h"
#include "my_framework.h"

/* Typedefs */
typedef enum {      // bdc_spi_commands_t: commands that shall be sent to slave MCU (BDC) via SPI
    BDC_START, BDC_STOP, BDC_INCREMENT, BDC_DECREMENT, BDC_RESET,
    BDC_REQUEST_VAL_MIN, BDC_REQUEST_VAL_MAX, BDC_REQUEST_VAL_ACT, BDC_REQUEST_STATE,
    RECEIVE_ANSWER_ONLY, NO_COMMAND
    /* Normally, an SPI answer is received at the same time an SPI command is sent.
     * RECEIVE_ANSWER_ONLY is needed to receive an answer without sending a command.
     * In this case, only dummy data is being sent which the slaves do not interpret as a command.
     */
} bdc_spi_commands_t;
typedef struct {    // measurementValues_t: named measurement values - physical values filtered or un-filtered, depending on mode
    float V_PV;         // V
    float I_PV_mA;      // mA
    float I_Load_mA;    // mA
    float V_BAT1;       // V
    float I_BAT1_mA;    // mA
    float V_BAT2;       // V
    float I_BAT2_mA;    // mA
    float V_PDU5V;      // V
    float I_PDU5V;      // mA
    float V_PDU3V3;     // V
    float I_PDU3V3;     // mA
} measurementValues_t;
typedef struct {    // dutycycle_t: struct to hold dutyCycle information
    int8_t dutyCycleStep;       // 1 step = 0.2 %   (this is the relevant value for CTRL)
    int8_t dutyCycleStep_last;
} dutycycle_t;
typedef struct {    // spi_data_t: raw rx data and duty cycles
    uint8_t rx_SS1;
    uint8_t rx_SS2;
    uint16_t rx_dc_SS1;
    uint16_t rx_dc_SS2;
} spi_data_t;

/* Global variables */
// Input variables
static measurementValues_t inputSignals;      // struct variable to hold input signals (filtered or unfiltered measurement values)
static uint16_t tau_ms = TAU_FILTER_CONSTANT_MS;    // filter constant for low-pass filtering inputSignals
// SPI variables
static spi_data_t spi_data;
static const uint8_t spi_magicNumber = 42;
// System variables
static uint16_t ctrlLoop_interval_ms = CTRL_LOOP_INTERVAL_MS;
static uint16_t uartSuperfast_interval_ms = UART_SUPERFAST_INTERVAL_MS;

// Command variables (control)
static dutycycle_t cmdCtrl; // duty cycle set point as determined by Control loop (MPPT)
static dutycycle_t cmdUart; // duty cycle set point as received from UART
static dutycycle_t setSpi;  // duty cycle set point that will finally be sent to slaves through SPI
static float fixedVbus_setPoint = PV_DEFAULT_SETPOINT_VOLTAGE;
static bdc_spi_commands_t cmd_toBDC = NO_COMMAND;   // command that shall be sent to BDC (e.g. switch on/off, change duty cycle, ...)

/* System flags */
static bool spi_newDataRead = false;                // flag to indicate that new data (1 byte) has been read from BDC via SPI
static bool spi_dutyCycle_read = false;             // flag to indicate that a duty cycle (2 bytes) has been read from BDC via SPI
static volatile interval_flag_t interval_flag;      // flags to indicate that an interval is active
static uint8_t bdc_enabled = 0;                     // flag to indicate enabled BDCs (1: SS1, 2: SS2, 3: SS1 and SS2)
static bool excess_power_management_active = false; // flag to indicate that excess power management is active
static bool pv_bus_Vmax_safety_active = false;      // flag to indicate that PVbus Vmax safety is active

/* Flags set via UART command */
static uartPrint_t uartPrint;                       // flags to enable/disable printing of several data through UART
static volatile bool sweep_trigger = false;         // flag to trigger a sweep (sweep is executed instead of MPPT, only for simulation)
static bool mppt_enabled = false;                   // flag to enable MPPT (instead of MPPT)
static bool fixedVbus_enabled = false;              // flag to enable fixed PV bus control (instead of MPPT)
static bool lowpass_active = false;                 // flag to enable low-pass filtering (control loop will use filtered or unfiltered values)


/* Custom functions */
void OE_VoltageLevelShifter(void);
// Handling SPI, UART, ADC and MPPT
void spi_routines(void);
void spi_check_bdcONstate(uint8_t rx_data, uint8_t slave_select);
void uart_handle_spi_response_data(uint8_t rx_data);
void uart_rx_routines(void);
void uart_tx_routines_slow(void);
void uart_tx_routines_fast(void);
void uart_tx_routines_superfast(void);
bool adc_read_routine(void);
void filter_measurement_values(void);
// Control loop
void mppt_PAO(void);
void excess_power_management(void);
void setDutycycle(void);
void pv_bus_Vmax_safety(void);
void movePVbusToVoltageSetPoint(void);

// Handling UART commands
void cmd_change_V_bat_sim(void);
void cmd_change_VbusVoltageSetpoint(void);
void cmd_change_tau(void);
void cmd_change_ctrlLoop_interval(void);


int main(void) {

    statemachine = INIT;
    bool adc_measurement = NOT_FINISHED;

    system_init();                          // halt WDT, enable FPU, master interrupt, system clock @ 24 MHz, init custom stop watch (Timer32)
    OE_VoltageLevelShifter();               // Output Enable on TXS0108E (voltage level shifter 3.3V <-> 5V for SPI between MSP432 master and PIC16 slaves
    uart_init(256000, TX_WITH_INTERRUPTS);  // Initialize UART (communication to PC): 8N1, 256000 baud/s (would still work with 1.5 MHz cpu clock) -> t_byte = 174 us
    spi_init(CPOL, CPHA, SPI_FREQ);         // Initialize SPI (communication to PICs): CPOL=0, CPHA=0, 400 kHz
    adc_init(AVCC, ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_2);  // Initialize Analog to Digital Converter: 14-bit resolution on 11 channels
//    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_128, ADC_PULSE_WIDTH_128);
    launchpad_leds_init();                  // Initialize red LED and RGB LED on MSP432 LaunchPad
    uart_send(typeString);
    cmd_toBDC = BDC_REQUEST_STATE;          // request current BDC state by sending SPI command
    spi_routines();                         // call spi routines in order to send SPI command
    systick_init();                         // Initialize SysTick to trigger every 1 ms (1000 Hz)

    statemachine = WAIT_STATE;

    while (1) {

        /* Scheduling (finite-state machine) */
        switch (statemachine) {
        case TRIGGER_ADC:
            t_stop(cycles.idle);
            t_start(cycles.adc);
            t_start(cycles.busy);
            adc_startMeasurement();
            statemachine = READ_INPUTS;
            break;
        case READ_INPUTS:
#ifdef PV_SIM
            pv_sim_measurement();     // pv_sim() can be used instead of adc_routines() when no PVA is connected to PCB.
            adc_measurement = FINISHED;
#else
            adc_measurement = adc_read_routine();       // INPUT: read analog measurements
#endif
            if (adc_measurement == FINISHED) {
                filter_measurement_values();    // must always be called, since physicalValues[i] get assigned to V_PV, I_PV_mA, ...
                if (uartPrint.stddev) {         // calc_mean_and_stddev() takes about 5 ms to compute for X_TEST_NUM=100 and fcpu = 24MHz
                    calc_mean_and_stddev();     // not used for control, only for debugging.
                }
                statemachine = SET_DUTYCYCLE;
                t_stop(cycles.adc);
            }
            break;
        case SET_DUTYCYCLE:
            t_start(cycles.ctrl);
            if (interval_flag.control_loop) {
                interval_flag.control_loop = false;
#ifdef PV_SIM
                if (sweep_trigger) {
                    sweep_trigger = false;
                    pv_sim_trigger_sweep();
                }
                if (pv_sim_sweeping_active()) {
                    pv_sim_sweep();     // only affects simulated duty cycle; the real duty cycle step (sent over SPI) is NOT changed.
                }
#endif
                /* Execute MPPT, if excess power management is not active.
                 * A sweeping command (only for simulation) also overrules the MPPT.
                 * However, sweeping works only for a simulated PV and for the simulated duty cycle (no real duty cycle is sent then).
                 */
                if (mppt_enabled && !excess_power_management_active && !pv_sim_sweeping_active() && !fixedVbus_enabled) {
                    mppt_PAO();        // PROCESS: do MPPT calculations
                }
                if (fixedVbus_enabled && !mppt_enabled) {
                    movePVbusToVoltageSetPoint();
                }
                excess_power_management();  // protect battery from over-charging
                pv_bus_Vmax_safety();       // take care of not exceeding maximum bus voltage level (harms MCU)
            }
            setDutycycle();     // UART command overrules MPPT command for duty cycle set point
            t_stop(cycles.ctrl);
            statemachine = WRITE_DUTYCYCLE;
            break;
        case WRITE_DUTYCYCLE:
            spi_routines();             // OUTPUT:  write commands via SPI to BDC �Cs
            statemachine = WRITE_UART;
            break;
        case WRITE_UART:
            t_stop(cycles.busy);
            t_start(cycles.uart_tmp);
            uart_tx_routines_superfast();   // OUTPUT: send messages via UART
            uart_tx_routines_fast();        // OUTPUT: send messages via UART
            uart_tx_routines_slow();        // OUTPUT: send messages via UART
            t_start(cycles.idle);
            statemachine = WAIT_STATE;
            break;
        case WAIT_STATE:
            uart_rx_routines();     // INPUT/PROCESS/OUTPUT: handle UART commands
            if (uart_transmissionFinished()) {
                t_stop(cycles.uart_tmp);
                cycles.uart = cycles.uart_tmp;
            }
            break;
        default:
            // there is no default state. Hence default is not allowed, so go back to WAIT_STATE
            statemachine = WAIT_STATE;
            break;
        }
    }
}

void OE_VoltageLevelShifter(void) {
    /* OE high to enable TXS0108E (voltage level shifter) for SPI communication */
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
}


void spi_routines(void) {
    /* Static variables to preserve previous state */
    static bdc_spi_commands_t spi_cmd_last = NO_COMMAND;
    static bool cmd_request_2Bytes = false;
    static bool cmd_request_1stByteReceived = false;
    static uint8_t rx_SS1_1stByte = 0;
    static uint8_t rx_SS2_1stByte = 0;
    /* Ordinary local variables */
    uint8_t tx_data_SS1[2] = {0};
    uint8_t tx_data_SS2[2] = {0};
    bool wait_for_answer = false;
    bdc_spi_commands_t spi_cmd = NO_COMMAND;

    spi_cmd = cmd_toBDC;
    cmd_toBDC = NO_COMMAND;
    if (spi_cmd == BDC_REQUEST_VAL_ACT || spi_cmd == BDC_REQUEST_VAL_MIN || spi_cmd == BDC_REQUEST_VAL_MAX) {
        cmd_request_2Bytes = true;
    }

    if (spi_cmd_last == BDC_START || spi_cmd_last == BDC_STOP ||
            spi_cmd_last == BDC_INCREMENT || spi_cmd_last == BDC_DECREMENT || spi_cmd_last == BDC_REQUEST_STATE ||
            spi_cmd_last == BDC_REQUEST_VAL_MIN || spi_cmd_last == BDC_REQUEST_VAL_MAX || spi_cmd_last == BDC_REQUEST_VAL_ACT) {
        wait_for_answer = true;
    }

    if (cmd_request_1stByteReceived) {
        cmd_request_1stByteReceived = false;
        wait_for_answer = true;
    }

    if (spi_cmd == NO_COMMAND && wait_for_answer) {
        spi_cmd = RECEIVE_ANSWER_ONLY;
    }

    spi_cmd_last = spi_cmd;
    setSpi.dutyCycleStep_last = setSpi.dutyCycleStep;
    setSpi.dutyCycleStep = 0;

    switch (spi_cmd) {
    case BDC_START:
        tx_data_SS1[0] = 4; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 4; tx_data_SS2[1] = 0;
        break;
    case BDC_STOP:
        tx_data_SS1[0] = 5; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 5; tx_data_SS2[1] = 0;
        break;
    case BDC_INCREMENT:
        tx_data_SS1[0] = 6; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 6; tx_data_SS2[1] = 0;
        break;
    case BDC_DECREMENT:
        tx_data_SS1[0] = 7; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 7; tx_data_SS2[1] = 0;
        break;
    case BDC_RESET:
        tx_data_SS1[0] = 8; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 8; tx_data_SS2[1] = 0;
        break;
    case BDC_REQUEST_STATE:
        tx_data_SS1[0] = 9; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 9; tx_data_SS2[1] = 0;
        break;
    case BDC_REQUEST_VAL_MIN:
        tx_data_SS1[0] = 10; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 10; tx_data_SS2[1] = 0;
        break;
    case BDC_REQUEST_VAL_ACT:
        tx_data_SS1[0] = 11; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 11; tx_data_SS2[1] = 0;
        break;
    case BDC_REQUEST_VAL_MAX:
        tx_data_SS1[0] = 12; tx_data_SS1[1] = 0;
        tx_data_SS2[0] = 12; tx_data_SS2[1] = 0;
        break;
    case RECEIVE_ANSWER_ONLY:
        tx_data_SS1[0] = 99; tx_data_SS1[1] = 0;    // dummy value
        tx_data_SS2[0] = 99; tx_data_SS2[1] = 0;    // dummy value
        break;
    default:
        // this case does not exist, nothing to do here. Neither in NO_COMMAND
        break;
    }
    spi_read(SS1); spi_read(SS2);               // dummy reads to clear buffer (necessary!)
    spi_send_arrays(tx_data_SS1, tx_data_SS2);
    if (wait_for_answer) {
        while (!spi_newDataReceived(SS1) || !spi_newDataReceived(SS2));
        spi_data.rx_SS1 = spi_read(SS1);
        spi_data.rx_SS1 -= spi_magicNumber;      // subtract spi_magicNumber to extract value sent by slave
        spi_data.rx_SS2 = spi_read(SS2);
        spi_data.rx_SS2 -= spi_magicNumber;      // subtract spi_magicNumber to extract value sent by slave
        if (cmd_request_2Bytes) {
            rx_SS1_1stByte = spi_data.rx_SS1;
            rx_SS2_1stByte = spi_data.rx_SS2;
            cmd_request_1stByteReceived = true;
            spi_dutyCycle_read = true;
            cmd_request_2Bytes = false;
        } else {
            spi_newDataRead = true;
            /* Check if data sent by slave was only zeros ("214" after subtracting spi_magicNumber).
             * This is not very probable and could indicate an erroneous behavior of the SPI slave.
             * If so, indicate this by setting the received value to zero.
             */
            spi_data.rx_SS1 = spi_data.rx_SS1==(255-spi_magicNumber+1) ? 0 : spi_data.rx_SS1;
            spi_data.rx_SS2 = spi_data.rx_SS2==(255-spi_magicNumber+1) ? 0 : spi_data.rx_SS2;
            if (spi_dutyCycle_read) {
                spi_data.rx_dc_SS1 = (rx_SS1_1stByte << 8) | spi_data.rx_SS1;
                spi_data.rx_dc_SS2 = (rx_SS2_1stByte << 8) | spi_data.rx_SS2;
            } else {
                spi_check_bdcONstate(spi_data.rx_SS1, SS1);
                spi_check_bdcONstate(spi_data.rx_SS2, SS2);
            }
        }
    }
}

void uart_tx_routines_slow(void) {
    if (!interval_flag.uart_slow)
        return;
    interval_flag.uart_slow = false;

    /* UART: Print ADC measurement values */
    if (uartPrint.adc_values) {
        uart_send("Raw ADC values: \n");
        sprintf(uart_txString, "P5.5(A0) : %.3f V (V_PV)\n", adcResults[0]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.4(A1) : %.3f V (I_PV)\n", adcResults[1]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.1(A4) : %.3f V (I_LOAD)\n", adcResults[2]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.6(A7) : %.3f V (V_BAT1)\n", adcResults[3]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P6.1(A14): %.3f V (V_BAT2)\n", adcResults[4]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.3(A10): %.3f V (I_BAT1)\n", adcResults[5]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.0(A13): %.3f V (I_BAT2)\n", adcResults[6]);
        uart_send(uart_txString);
        uart_send("---------------------------------------------\n");
    }
    /* UART: Print physical measurement values */
    if (uartPrint.measurement_values) {
        uart_send("Physical measurement values (not filtered)\n");
        sprintf(uart_txString, "          % 9.3f mW (P_PV)\n", physicalValues[0]*physicalValues[1]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.5(A0) :% 9.3f V  (V_PV)\n", physicalValues[0]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.4(A1) :% 9.3f mA (I_PV)\n", physicalValues[1]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.1(A4) :% 9.3f mA (I_LOAD)\n", physicalValues[2]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.6(A7) :% 9.3f V  (V_BAT1)\n", physicalValues[3]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P6.1(A14):% 9.3f V  (V_BAT2)\n", physicalValues[4]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.3(A10):% 9.3f mA (I_BAT1)\n", physicalValues[5]);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.0(A13):% 9.3f mA (I_BAT2)\n", physicalValues[6]);
        uart_send(uart_txString);
        uart_send("---------------------------------------------\n");
    }
    /* UART: Print filtered physical measurement values */
    if (uartPrint.filtered_inputSignals) {
        if (lowpass_active) {
            uart_send("Input signals (physical measurement values, filtered)\n");
        } else {
            uart_send("Input signals (physical measurement values, not filtered)\n");
        }
        sprintf(uart_txString, "          % 9.3f mW (P_PV)\n", inputSignals.V_PV*inputSignals.I_PV_mA);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.5(A0) :% 9.3f V  (V_PV)\n", inputSignals.V_PV);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.4(A1) :% 9.3f mA (I_PV)\n", inputSignals.I_PV_mA);
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.1(A4) :% 9.3f mA (I_LOAD)\n", inputSignals.I_Load_mA);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.6(A7) :% 9.3f V  (V_BAT1)\n", inputSignals.V_BAT1);
        uart_send(uart_txString);
        sprintf(uart_txString, "P6.1(A14):% 9.3f V  (V_BAT2)\n", inputSignals.V_BAT2);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.3(A10):% 9.3f mA (I_BAT1)\n", inputSignals.I_BAT1_mA);
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.0(A13):% 9.3f mA (I_BAT2)\n", inputSignals.I_BAT2_mA);
        uart_send(uart_txString);
        uart_send("---------------------------------------------\n");
    }
    if (uartPrint.zeroI_Vmeas) {
        sprintf(uart_txString, "P5.4(A1) : %6.4f V (I_PV)\n", zeroCurrentVoltages[0]);      // [V] I_PV (current sensor)
        uart_send(uart_txString);
        sprintf(uart_txString, "P5.1(A4) : %6.4f V (I_LOAD)\n", zeroCurrentVoltages[1]);    // [V] I_LOAD (current sensor)
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.3(A10): %6.4f V (I_BAT1)\n", zeroCurrentVoltages[2]);    // [V] I_BAT1 (current sensor)
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.0(A13): %6.4f V (I_BAT2)\n", zeroCurrentVoltages[3]);    // [V] I_BAT2 (current sensor)
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.4(A9) : %6.4f V (I_PDU5V)\n", zeroCurrentVoltages[4]);   // [V] I_PDU_5V (current sensor)
        uart_send(uart_txString);
        sprintf(uart_txString, "P4.2(A11): %6.4f V (I_PDU3V3)\n", zeroCurrentVoltages[5]);  // [V] I_PDU3.3V (current sensor)
        uart_send(uart_txString);
        uart_send("---------------------------------------------\n");
    }
    if (uartPrint.stddev && iv_test.finished) {
        iv_test.finished = false;
        iv_test.idx = 0;
        sprintf(uart_txString, "I_test stddev:% 9.3f mA\n", iv_test.i_stddev);
        uart_send(uart_txString);
        sprintf(uart_txString, "V_test stddev:% 9.3f mV\n", iv_test.v_stddev);
        uart_send(uart_txString);
        uart_send("--------------------------\n");
    }
}

void uart_tx_routines_fast(void) {
    if (!interval_flag.uart_fast)
        return;
    interval_flag.uart_fast = false;

    if (pv_bus_Vmax_safety_active) {
        uart_send("PVbus Vmax safety active\n");
    }
    /* UART: Print received SPI data and explanation its meaning */
    if (spi_newDataRead) {
        spi_newDataRead = false;
        if (spi_dutyCycle_read) {
            spi_dutyCycle_read = false;
            sprintf(uart_txString, "Slave1 - Duty cycle %d\n", spi_data.rx_dc_SS1);
            uart_send(uart_txString);
            sprintf(uart_txString, "Slave2 - Duty cycle %d\n", spi_data.rx_dc_SS2);
            uart_send(uart_txString);
        } else {
            if (excess_power_management_active) {
                uart_send("EPM active\n");
            }
            uart_send("Slave1 - ");
            uart_handle_spi_response_data(spi_data.rx_SS1);
            uart_send("Slave2 - ");
            uart_handle_spi_response_data(spi_data.rx_SS2);
            uart_send("---------------------------------------------\n");
        }
    }

    /* UART: Print data as $<data to be send>; for displaying them in Serial Port Plotter */
    if (uartPrint.plots) {
        sprintf(uart_txString, "$%.3f %.3f %.3f %d;\n", physicalValues[0]*1000,
                physicalValues[1], physicalValues[0] * physicalValues[1], 1000+200*cmdCtrl.dutyCycleStep_last);     // V_PV, I_PV, P_PV_mW, dutycycle
        uart_send(uart_txString);
    }
    if (uartPrint.plots_vipPV) {
        sprintf(uart_txString, "$%.3f %.3f %.3f;\n", physicalValues[0]*1000, physicalValues[1], physicalValues[0] * physicalValues[1]);     // V_PV, I_PV, P_PV_mW
        uart_send(uart_txString);
    }
    if (uartPrint.plots_vipPVf) {
        sprintf(uart_txString, "$%.3f %.3f %.3f;\n", inputSignals.V_PV*1000, inputSignals.I_PV_mA, inputSignals.V_PV*inputSignals.I_PV_mA);     // filtered: V_PV, I_PV, P_PV_mW
        uart_send(uart_txString);
    }
    if (uartPrint.plots_vipBAT2f) {
        sprintf(uart_txString, "$%.3f %.3f;\n", inputSignals.V_BAT2*1000, inputSignals.I_BAT2_mA);   // filtered V_BAT2, I_BAT2
        uart_send(uart_txString);
    }
    if (uartPrint.plots_vipBAT2) {
        sprintf(uart_txString, "$%.3f %.3f;\n", physicalValues[4]*1000, physicalValues[6]);   // V_BAT2, I_BAT2
        uart_send(uart_txString);
    }
    if (uartPrint.cpuLoad) {
        float cpu_load = (cycles.busy/(CS_getMCLK()/1000.0)) / (float)SAMPLING_INTERVAL_MS;
        sprintf(uart_txString, "CPU load - %.3f %%\n", cpu_load*100);
        uart_send(uart_txString);
        uart_send("---------------------------------------\n");
    }
    if (uartPrint.adc_timing) {
        uart_send("ADC routine - ");
        sprintf(uart_txString, "%d cycles, %.3f us\n", cycles.adc, cycles.adc/(CS_getMCLK()/1000000.0));
        uart_send(uart_txString);
    }
    if (uartPrint.ctrl_timing) {
        uart_send("MPPT routine - ");
        sprintf(uart_txString, "%d cycles, %.3f us\n", cycles.ctrl, cycles.ctrl/(CS_getMCLK()/1000000.0));
        uart_send(uart_txString);
    }
    if (uartPrint.uart_timing) {
        uart_send("UART TX routine - ");
        sprintf(uart_txString, "%d cycles, %.3f us\n", cycles.uart, cycles.uart/(CS_getMCLK()/1000000.0));
        uart_send(uart_txString);
    }
    if (uartPrint.dutycycle_cmd) {
        uart_send("DutyCycle step cmd  -  ");
        if (mppt_enabled)
            uart_send("(MPPT   ACTIVE)  ");
        else
            uart_send("(MPPT INACTIVE)  ");
        if (excess_power_management_active)
            uart_send("(EPM   ACTIVE)\n");
        else
            uart_send("(EPM INACTIVE)\n");
        sprintf(uart_txString, "SPI % d  ||  Ctrl: % d\n", setSpi.dutyCycleStep_last, cmdCtrl.dutyCycleStep_last);
        uart_send(uart_txString);
    }
}

void uart_tx_routines_superfast(void) {
    if (!interval_flag.uart_superfast)
        return;
    interval_flag.uart_superfast = false;

    /* UART: Print data as $<data to be send>; for displaying them in Serial Port Plotter */
    if (uartPrint.current_meas_noise) {
            sprintf(uart_txString, "$%.3f;\n", physicalValues[1]*1000);   // I_PV (�A!)
            uart_send(uart_txString);
        }
    if (uartPrint.plots_fast) {
        sprintf(uart_txString, "$%.3f %.3f %d;\n", physicalValues[0]*1000,
                physicalValues[1], 10+2*cmdCtrl.dutyCycleStep_last);     // V_PV, I_PV, dutycycle
        uart_send(uart_txString);
    }
}

void uart_rx_routines(void) {
    if (uart_stringToRead() == false)
        return;

    uart_read(uart_rxString);
    if (strcmp(uart_rxString, "Type") == 0 || strcmp(uart_rxString, "type") == 0) {
        uart_send(typeString);
    }
    else if (strcmp(uart_rxString, "red") == 0) {
        uart_send("Toggle led RED\n");
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
    }
    else if (strcmp(uart_rxString, "green") == 0) {
        uart_send("Toggle led GREEN\n");
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    }
    else if (strcmp(uart_rxString, "blue") == 0) {
        uart_send("Toggle led BLUE\n");
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }
    else if (strcmp(uart_rxString, "spi on") == 0) {
        uart_send("OE (Output Enable) of SPI voltage level shifter: ON\n");
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
    }
    else if (strcmp(uart_rxString, "spi off") == 0) {
        uart_send("OE (Output Enable) of SPI voltage level shifter: OFF\n");
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    }
    else if (strcmp(uart_rxString, "mppt") == 0 || strcmp(uart_rxString, "ctrl") == 0) {
        uart_send("Active control modules:\n");
        if (mppt_enabled) {
            uart_send("[x] MPPT                   (active)\n");
        } else {
            uart_send("[ ] MPPT                   (not active)\n");
        }
        if (fixedVbus_enabled) {
            uart_send("[x] Fixed PV bus set point (active)\n");
        } else {
            uart_send("[ ] Fixed PV bus set point (not active)\n");
        }
    }
    else if (strcmp(uart_rxString, "mppt on") == 0 || strcmp(uart_rxString, "mppt start") == 0) {
        if (fixedVbus_enabled) {
            fixedVbus_enabled = false;
            uart_send("Fixed PV bus voltage set point disabled.\n");
        }
        uart_send("MPPT enabled.\n");
        mppt_enabled = true;
    }
    else if (strcmp(uart_rxString, "mppt off") == 0 || strcmp(uart_rxString, "mppt stop") == 0) {
        uart_send("MPPT disabled.\n");
        mppt_enabled = false;
    }
    else if (strcmp(uart_rxString, "pvset on") == 0) {
        if (mppt_enabled) {
            mppt_enabled = false;
            uart_send("MPPT disabled.\n");
        }
        fixedVbus_setPoint = PV_DEFAULT_SETPOINT_VOLTAGE;
        fixedVbus_enabled = true;
        sprintf(uart_txString, "Fixed PV bus voltage set to %.3f V\n", fixedVbus_setPoint);
        uart_send(uart_txString);
    }
    else if (strcmp(uart_rxString, "pvset off") == 0) {
        fixedVbus_enabled = false;
        uart_send("Fixed PV bus voltage set point disabled.\n");
    }
    else if ((strncmp(uart_rxString, "pvset ", 6) == 0)) {
        if (mppt_enabled) {
            mppt_enabled = false;
            uart_send("MPPT disabled.\n");
        }
        cmd_change_VbusVoltageSetpoint();
    }
    else if (strcmp(uart_rxString, "pvset") == 0) {
        sprintf(uart_txString, "Fixed PV bus voltage set point is %.3f V\n", fixedVbus_setPoint);
        uart_send(uart_txString);
        if (fixedVbus_enabled) {
            uart_send("Fixed PV bus voltage set point is active.\n");
        } else {
            uart_send("Fixed PV bus voltage set point is not active.\n");
        }
    }
    else if (strcmp(uart_rxString, "lp") == 0) {
        if (lowpass_active) {
            uart_send("Low-pass filter is ON.\n");
        } else {
            uart_send("Low-pass filter is OFF.\n");
        }
        sprintf(uart_txString, "Low-pass filter time constant tau = %d ms\n", tau_ms);
        uart_send(uart_txString);
    }
    else if (strcmp(uart_rxString, "lp on") == 0) {
        uart_send("Low-pass filter enabled.\n");
        lowpass_active = true;
    }
    else if (strcmp(uart_rxString, "lp off") == 0) {
        uart_send("Low-pass filter disabled.\n");
        lowpass_active = false;
    }
    else if ((strncmp(uart_rxString, "lp ", 3) == 0)) {
        cmd_change_tau();
    }
    else if (strcmp(uart_rxString, "mppt int") == 0 || strcmp(uart_rxString, "ctrl int") == 0) {
        sprintf(uart_txString, "Control loop interval: %d ms\n", ctrlLoop_interval_ms);
        uart_send(uart_txString);
    }
    else if (strcmp(uart_rxString, "mppt timing") == 0 || strcmp(uart_rxString, "ctrl timing") == 0) {
        uartPrint.ctrl_timing = !uartPrint.ctrl_timing;
        if (uartPrint.ctrl_timing)
            uart_send("Printing MPPT timing enabled\n");
        else
            uart_send("Printing MPPT timing disabled\n");
    }
    else if ((strncmp(uart_rxString, "ctrl int ", 9) == 0) || (strncmp(uart_rxString, "mppt int ", 9) == 0)) {
        cmd_change_ctrlLoop_interval();
    }
    else if (strcmp(uart_rxString, "bdc on") == 0 || strcmp(uart_rxString, "bdc start") == 0) {
        uart_send("Switching BDC on...\n");
        cmd_toBDC = BDC_START;
    }
    else if (strcmp(uart_rxString, "bdc off") == 0 || strcmp(uart_rxString, "bdc stop") == 0) {
        uart_send("Switching BDC off...\n");
        cmd_toBDC = BDC_STOP;
    }
    else if (strcmp(uart_rxString, "bdc +") == 0) {
        uart_send("Incrementing PWM duty cycle of BDC...\n");
        cmdUart.dutyCycleStep = 1;
    }
    else if (strcmp(uart_rxString, "bdc -") == 0) {
        uart_send("Decrementing PWM duty cycle of BDC...\n");
        cmdUart.dutyCycleStep = -1;
    }
    else if (strcmp(uart_rxString, "bdc min") == 0) {
        uart_send("Requesting minimum PWM duty cycle of BDC...\n");
        cmd_toBDC = BDC_REQUEST_VAL_MIN;
    }
    else if (strcmp(uart_rxString, "bdc max") == 0) {
        uart_send("Requesting maximum PWM duty cycle of BDC...\n");
        cmd_toBDC = BDC_REQUEST_VAL_MAX;
    }
    else if (strcmp(uart_rxString, "bdc act") == 0) {
        uart_send("Requesting actual PWM duty cycle of BDC...\n");
        cmd_toBDC = BDC_REQUEST_VAL_ACT;
    }
    else if (strcmp(uart_rxString, "sweep") == 0) {
        uart_send("Make sweep through solar cell\n");
        sweep_trigger = true;
    }
    else if ((strcmp(uart_rxString, "uart bd") == 0)) {
        sprintf(uart_txString, "UART baudrate %d baud/s\n", uart_getBaudrate());
        uart_send(uart_txString);
    }
    else if ((strncmp(uart_rxString, "uart bd ", 8) == 0)) {
        cmd_change_uart_baudrate();
    }
    else if (strcmp(uart_rxString, "uart timing") == 0) {
        uartPrint.uart_timing = !uartPrint.uart_timing;
        if (uartPrint.uart_timing)
            uart_send("Printing UART transmit timing enabled\n");
        else
            uart_send("Printing UART transmit timing disabled\n");
    }
    else if (strcmp(uart_rxString, "print dc") == 0 || strcmp(uart_rxString, "dc print") == 0) {
        uartPrint.dutycycle_cmd = !uartPrint.dutycycle_cmd;
        if (uartPrint.dutycycle_cmd)
            uart_send("Printing dutycycle command enabled\n");
        else
            uart_send("Printing dutycycle command disabled\n");
    }
    else if (strcmp(uart_rxString, "reset dc") == 0 || strcmp(uart_rxString, "dc reset") == 0) {
        pv_sim_resetDutycycle();
        uart_send("Duty cycle reset\n(only for simulation purposes; has no relevance for real duty cycle which is sent over SPI)\n");
    }
    else if ((strncmp(uart_rxString, "bat sim ", 8) == 0)) {
        cmd_change_V_bat_sim();
    }
    else if (strcmp(uart_rxString, "spi clk") == 0) {
        sprintf(uart_txString, "SPI clock speed: %d kHz\n", spi_getClockSpeed()/1000);
        uart_send(uart_txString);
    }
    else if ((strncmp(uart_rxString, "spi clk ", 8) == 0)) {
        cmd_change_spi_clk();
    }
    else if (strcmp(uart_rxString, "cpu load") == 0) {
        uartPrint.cpuLoad = !uartPrint.cpuLoad;
        if (uartPrint.cpuLoad)
            uart_send("Displaying CPU load enabled\n");
        else
            uart_send("Displaying CPU load disabled\n");
    }
    else if (strcmp(uart_rxString, "plots") == 0) {
        uartPrint.plots = !uartPrint.plots;
        if (uartPrint.plots)
            uart_send("Plotting measurement values enabled\n");
        else
            uart_send("Plotting measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "plots fast") == 0) {
        uartPrint.plots_fast = !uartPrint.plots_fast;
        if (uartPrint.plots_fast)
            uart_send("Plotting measurement values enabled\n");
        else
            uart_send("Plotting measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "plots pv") == 0) {
        uartPrint.plots_vipPV = !uartPrint.plots_vipPV;
        if (uartPrint.plots_vipPV)
            uart_send("Plotting PV measurement values enabled\n");
        else
            uart_send("Plotting PV measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "plots pv lp") == 0) {
        uartPrint.plots_vipPVf = !uartPrint.plots_vipPVf;
        if (uartPrint.plots_vipPVf)
            uart_send("Plotting PV filtered measurement values enabled\n");
        else
            uart_send("Plotting PV filtered measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "plots bat2") == 0) {
        uartPrint.plots_vipBAT2 = !uartPrint.plots_vipBAT2;
        if (uartPrint.plots_vipBAT2)
            uart_send("Plotting BAT2 measurement values enabled\n");
        else
            uart_send("Plotting BAT2 measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "plots bat2 lp") == 0) {
        uartPrint.plots_vipBAT2f = !uartPrint.plots_vipBAT2f;
        if (uartPrint.plots_vipBAT2f)
            uart_send("Plotting BAT2 filtered measurement values enabled\n");
        else
            uart_send("Plotting BAT2 filtered measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "adc zero i") == 0 || strcmp(uart_rxString, "zero i") == 0) {
        uartPrint.zeroI_Vmeas = !uartPrint.zeroI_Vmeas;
        if (uartPrint.zeroI_Vmeas) {
            uart_send("Zero current voltage measurement enabled\n");
        } else {
            adc_resetZeroCurrentVoltageValues();
            uart_send("Zero current voltage measurement disabled\n");
        }
    }
    else if (strcmp(uart_rxString, "adc print") == 0 || strcmp(uart_rxString, "print adc") == 0) {
        uartPrint.adc_values = !uartPrint.adc_values;
        if (uartPrint.adc_values)
            uart_send("Printing ADC measurement values enabled (raw voltages on analog channels)\n");
        else
            uart_send("Printing ADC measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "adc timing") == 0) {
        uartPrint.adc_timing = !uartPrint.adc_timing;
        if (uartPrint.adc_timing)
            uart_send("Printing ADC timing enabled\n");
        else
            uart_send("Printing ADC timing disabled\n");
    }
    else if (strcmp(uart_rxString, "meas print") == 0 || strcmp(uart_rxString, "print meas") == 0) {
        uartPrint.measurement_values = !uartPrint.measurement_values;
        if (uartPrint.measurement_values)
            uart_send("Printing physical measurement values enabled (ADC values converted to physical values)\n");
        else
            uart_send("Printing physical measurement values disabled\n");
    }
    else if (strcmp(uart_rxString, "calc stddev") == 0 || strcmp(uart_rxString, "print stddev") == 0) {
        uartPrint.stddev = !uartPrint.stddev;
        if (uartPrint.stddev)
            uart_send("Printing standard deviation of test I and V enabled\n");
        else
            uart_send("Printing standard deviation of test I and V disabled\n");
    }
    else if (strcmp(uart_rxString, "input print") == 0 || strcmp(uart_rxString, "print input") == 0) {
        uartPrint.filtered_inputSignals = !uartPrint.filtered_inputSignals;
        if (uartPrint.filtered_inputSignals) {
            uart_send("Printing input signals enabled\n");
            if (lowpass_active) {
                uart_send("Low-pass filter is ON, input signals are filtered.\n");
            } else {
                uart_send("Low-pass filter is OFF, input signals are not filtered.\n");
            }
        } else {
            uart_send("Printing input signals disabled\n");
        }
    }
    else if (strcmp(uart_rxString, "noise") == 0) {
        uartPrint.current_meas_noise = !uartPrint.current_meas_noise;
        if (uartPrint.current_meas_noise)
            uart_send("Printing noise (current measurement) enabled\n");
        else
            uart_send("Printing noise (current measurement) disabled\n");
    }
    else if (strcmp(uart_rxString, "cpu clk") == 0) {
        sprintf(uart_txString, "CPU clock speed: %.1f MHz\n", CS_getMCLK()/1000000.0);
        uart_send(uart_txString);
    }
    else if ((strncmp(uart_rxString, "cpu clk ", 8) == 0)) {
        cmd_change_cpu_clk();
    }
    else {   // default case of uart input
        uart_send("Unknown command.\n");
    }
    uart_send("---------------------------------------------\n");
}

bool adc_read_routine(void) {
    if (!adc_measurementFinished())
        return NOT_FINISHED;
    adc_readMeasurements(adcResults);
    adc_convertToPhysicalValues(adcResults, physicalValues);
    if (uartPrint.zeroI_Vmeas) {
        zeroCurrentVoltagesRaw[0] = adcResults[1];  // [V] I_PV (current sensor)
        zeroCurrentVoltagesRaw[1] = adcResults[2];  // [V] I_LOAD (current sensor)
        zeroCurrentVoltagesRaw[2] = adcResults[5];  // [V] I_BAT1(current sensor)
        zeroCurrentVoltagesRaw[3] = adcResults[6];  // [V] I_BAT2 (current sensor)
        zeroCurrentVoltagesRaw[4] = adcResults[9];  // [V] I_PDU_5V (current sensor)
        zeroCurrentVoltagesRaw[5] = adcResults[10];  // [V] I_PDU3.3V (current sensor)
        adc_getZeroCurrentVoltages(zeroCurrentVoltagesRaw, zeroCurrentVoltages, SAMPLING_INTERVAL_MS, 10000);
    }
    return FINISHED;
}

void filter_measurement_values(void) {
    if (lowpass_active) {
        inputSignals.V_PV = lp_filtering(physicalValues[0], inputSignals.V_PV, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.I_PV_mA = lp_filtering(physicalValues[1], inputSignals.I_PV_mA, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.I_Load_mA = lp_filtering(physicalValues[2], inputSignals.I_Load_mA, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.V_BAT1 = lp_filtering(physicalValues[3], inputSignals.V_BAT1, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.V_BAT2 = lp_filtering(physicalValues[4], inputSignals.V_BAT2, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.I_BAT1_mA = lp_filtering(physicalValues[5], inputSignals.I_BAT1_mA, tau_ms, SAMPLING_INTERVAL_MS);
        inputSignals.I_BAT2_mA = lp_filtering(physicalValues[6], inputSignals.I_BAT2_mA, tau_ms, SAMPLING_INTERVAL_MS);
    } else {
        inputSignals.V_PV = physicalValues[0];
        inputSignals.I_PV_mA = physicalValues[1];
        inputSignals.I_Load_mA = physicalValues[2];
        inputSignals.V_BAT1 = physicalValues[3];
        inputSignals.V_BAT2 = physicalValues[4];
        inputSignals.I_BAT1_mA = physicalValues[5];
        inputSignals.I_BAT2_mA = physicalValues[6];
    }
}

void mppt_PAO(void) {
    /* Set up variables */
    static float V_PV_prev    = 0;
    static float P_PV_mW_prev = 0;
    float P_PV_mW;

    float dutycycle_step = 0;

    /* Calculated values */
    P_PV_mW = inputSignals.V_PV * inputSignals.I_PV_mA;

    if (P_PV_mW > P_PV_mW_prev) {   // current PV output (power) is higher as before
        if (inputSignals.V_PV > V_PV_prev)
            dutycycle_step = -1;    // decrement duty cycle to increment PV bus voltage
        else
            dutycycle_step = 1;     // increment duty cycle to decrement PV bus voltage
    } else {                        // previous PV output (power) was higher than now
        if (inputSignals.V_PV > V_PV_prev)
            dutycycle_step = 1;     // increment duty cycle to decrement PV bus voltage
        else
            dutycycle_step = -1;    // decrement duty cycle to increment PV bus voltage
    }

    cmdCtrl.dutyCycleStep = dutycycle_step;

    P_PV_mW_prev = P_PV_mW;
    V_PV_prev = inputSignals.V_PV;
}

void movePVbusToVoltageSetPoint(void) {
    float dutycycle_step = 0;

    if (inputSignals.V_PV < fixedVbus_setPoint) {
        dutycycle_step--;
    } else {
        dutycycle_step++;
    }

    cmdCtrl.dutyCycleStep = dutycycle_step;
}

void excess_power_management(void) {
    float dutycycle_step = 0;

    if (inputSignals.V_BAT2 > BAT_SOC_100) {
        excess_power_management_active = true;
    } else if (inputSignals.V_BAT2 < BAT_SOC_80) {
        excess_power_management_active = false;
    } else if (inputSignals.V_BAT2 < BAT_SOC_0) {
        cmd_toBDC = BDC_STOP;               // switch BDC off
    }

    if (excess_power_management_active) {
        if (inputSignals.I_BAT2_mA > BAT_CURRENT_THRESHOLD && inputSignals.V_PV < PV_MAX_VOLTAGE) {
            dutycycle_step = -1;    // reduce duty cycle = increase PV bus voltage
            cmdCtrl.dutyCycleStep = dutycycle_step;
        }
    }
}

void pv_bus_Vmax_safety(void) {
    float dutycycle_step = 0;

    /* Calculated values */
    if (inputSignals.V_PV >= PV_MAX_VOLTAGE) {
        pv_bus_Vmax_safety_active = true;
        dutycycle_step = 1;         // increase duty cycle = decrease PV bus voltage
        cmdCtrl.dutyCycleStep = dutycycle_step;
    } else {
        pv_bus_Vmax_safety_active = false;
    }
}

void setDutycycle(void) {
#ifndef PV_SIM                      // if PV_SIM is not active, set real dutycycle step to send over SPI
    if (cmdUart.dutyCycleStep) {    // UART command overrules MPPT command for duty cycle set point
        setSpi.dutyCycleStep = cmdUart.dutyCycleStep;
    } else {
        setSpi.dutyCycleStep = cmdCtrl.dutyCycleStep;
    }
    if (setSpi.dutyCycleStep > 0) {
        cmd_toBDC = BDC_INCREMENT;
    } else if (setSpi.dutyCycleStep < 0) {
        cmd_toBDC = BDC_DECREMENT;
    }
#else
    pv_sim_stepDutycycle(cmdUart.dutyCycleStep, cmdCtrl.dutyCycleStep);
#endif
    cmdUart.dutyCycleStep_last = cmdUart.dutyCycleStep;
    cmdUart.dutyCycleStep = 0;
    cmdCtrl.dutyCycleStep_last = cmdCtrl.dutyCycleStep;   // cmdCtrl.dutyCycleStep is changed in mppt_PAO() and excess_power_management()
    cmdCtrl.dutyCycleStep = 0;
}

void cmd_change_VbusVoltageSetpoint(void) {
    uint16_t fixedVbus_setPoint_new_int = atoi(&uart_rxString[6]);
    float fixedVbus_setPoint_new = (float)fixedVbus_setPoint_new_int / 1000.0;  // convert from (mV) to (V)
    if (fixedVbus_setPoint_new < (inputSignals.V_BAT2/0.96) || fixedVbus_setPoint_new > PV_MAX_VOLTAGE) {
        sprintf(uart_txString, "PV bus voltage setpoint unchanged. Current value: %.3f V\n", fixedVbus_setPoint);
        uart_send(uart_txString);
        sprintf(uart_txString, "Put a value between %d and %d mV\n",
                (uint16_t)(inputSignals.V_BAT2*(1000/0.96)), (uint16_t)(PV_MAX_VOLTAGE*1000));
        uart_send(uart_txString);
    } else {
        fixedVbus_setPoint = fixedVbus_setPoint_new;
        fixedVbus_enabled = true;
        sprintf(uart_txString, "PV bus voltage set point changed to %.3f V\n", fixedVbus_setPoint);
        uart_send(uart_txString);
    }
}

void cmd_change_tau(void) {
    uint16_t tau_ms_new = atoi(&uart_rxString[3]);
    if (tau_ms_new < SAMPLING_INTERVAL_MS || tau_ms_new % SAMPLING_INTERVAL_MS) {
        sprintf(uart_txString, "Low-pass filter constant unchanged.\nCurrent value: tau = %d ms\n", tau_ms);
        uart_send(uart_txString);
        sprintf(uart_txString, "Put a value >= ADC sampling interval (%d ms).\n", (uint16_t)(SAMPLING_INTERVAL_MS));
        uart_send(uart_txString);
        uart_send("Further, tau must be an integer multiple of ADC sampling interval\n");
        uart_send("Note: If tau == ADC sampling interval, no filtering is actually happening.\n");
    } else {
        tau_ms = tau_ms_new;
        sprintf(uart_txString, "Low-pass filter constant changed to tau = %d ms\n", tau_ms);
        uart_send(uart_txString);
        uart_send("Activate low-pass by sending 'lp on'\n");
    }
}

void cmd_change_ctrlLoop_interval(void) {
    uint16_t ctrlLoop_interval_ms_new = atoi(&uart_rxString[9]);
    if (ctrlLoop_interval_ms_new < SAMPLING_INTERVAL_MS || ctrlLoop_interval_ms_new % SAMPLING_INTERVAL_MS) {
        sprintf(uart_txString, "Control loop execution interval (CLEI) unchanged.\nCurrent value: %d ms\n", ctrlLoop_interval_ms);
        uart_send(uart_txString);
        sprintf(uart_txString, "Put a value >= ADC sampling interval (%d ms).\n", (uint16_t)(SAMPLING_INTERVAL_MS));
        uart_send(uart_txString);
        uart_send("Further, CLEI must be an integer multiple of ADC sampling interval\n");
    } else {
        ctrlLoop_interval_ms = ctrlLoop_interval_ms_new;
        uartSuperfast_interval_ms = ctrlLoop_interval_ms_new;
        sprintf(uart_txString, "Control loop execution interval changed to %d ms\n", ctrlLoop_interval_ms);
        uart_send(uart_txString);
    }
}

void cmd_change_V_bat_sim(void) {
    uint32_t V_bat_sim_new = atoi(&uart_rxString[8]);
    if (V_bat_sim_new < V_BAT_SIM_MINIMUM || V_bat_sim_new > V_BAT_SIM_MAXIMUM) {
        sprintf(uart_txString, "Simulated battery voltage unchanged. Current value: %d mV\n", V_bat_sim);
        uart_send(uart_txString);
        sprintf(uart_txString, "Put a value between %d and %d mV\n", V_BAT_SIM_MINIMUM, V_BAT_SIM_MAXIMUM);
        uart_send(uart_txString);
    } else {
        V_bat_sim = V_bat_sim_new;
        sprintf(uart_txString, "Simulated battery voltage set to %d mV\n", V_bat_sim);
        uart_send(uart_txString);
    }
}

void uart_handle_spi_response_data(uint8_t rx_data) {
    switch (rx_data) {
    case BDC_ON:
        uart_send("BDC is ON.");
        break;
    case BDC_OFF:
        uart_send("BDC is OFF.");
        break;
    case BDC_SWITCHED_ON:
        uart_send("BDC switched ON.");
        break;
    case BDC_SWITCHED_OFF:
        uart_send("BDC switched OFF.");
        break;
    case BDC_INCREMENTED:
        uart_send("DC +");      // duty cycle incremented
        break;
    case BDC_DECREMENTED:
        uart_send("DC -");      // duty cycle decremented
        break;
    default:
        uart_send("No reaction.");
    }
    uart_send("\n");
}

void spi_check_bdcONstate(uint8_t rx_data, uint8_t slave_select) {
    bool bdc_enabled_previous = bdc_enabled;
    if (rx_data == BDC_ON || rx_data == BDC_SWITCHED_ON) {
        bdc_enabled |= slave_select;
    } else if (rx_data == BDC_OFF || rx_data == BDC_SWITCHED_OFF) {
        bdc_enabled &= ~slave_select;
    }
    if (bdc_enabled != bdc_enabled_previous) {
        if (bdc_enabled) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);   // turn red LED P1.0 on
        } else {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);    // turn red LED P1.0 off
        }
    }
}

void SysTick_Handler(void) {
    static uint16_t ticks_ms = 0;

    if ( !(ticks_ms % UART_SLOW_INTERVAL_MS) ) {
        interval_flag.uart_slow = true;
    }
    if ( !(ticks_ms % UART_FAST_INTERVAL_MS) ) {
        interval_flag.uart_fast = true;
    }
    if ( !(ticks_ms % uartSuperfast_interval_ms) ) {    // interval is coupled to ctrlLoop_interval_ms (which can be changed dynamically)
        interval_flag.uart_superfast = true;
    }
    if ( !(ticks_ms % SAMPLING_INTERVAL_MS) ) {
        statemachine = TRIGGER_ADC;
    }
    if ( !(ticks_ms % ctrlLoop_interval_ms) ) {    // interval may be changed via UART command
        interval_flag.control_loop = true;
    }
    /* ticker reset each LONGEST_INTERVAL_MS (must be equal to or a multiple of the longest period) */
    if (ticks_ms > LONGEST_INTERVAL_MS)
        ticks_ms %= LONGEST_INTERVAL_MS;

    ticks_ms++;
}
