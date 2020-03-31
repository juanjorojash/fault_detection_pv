/*
 * my_framework.c
 *
 *  Created on: 09.03.2020
 *      Author: Adrian
 */


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Custom Includes */
#include "my_framework.h"
#include "my_timing.h"

/* Global variables */
statemachine_t statemachine;

char uart_rxString[100];    // incoming UART data
char uart_txString[100];    // outgoing UART data
volatile cycles_t cycles;    // cycles measurements (timing measurement values)
// ADC variables
// ADC variables
float adcResults[11] = {0};
float physicalValues[11] = {0};
float zeroCurrentVoltagesRaw[6] = {0};
float zeroCurrentVoltages[6] = {0};

/* Functions */
void cmd_change_cpu_clk(void) {
    uint32_t cpu_clk_desired = atoi(&uart_rxString[8]);
    uint32_t cpu_clk_new = 0;
    uint8_t k = 0;
    uint32_t CS_CTL0_DCORSEL_k = 0;
    for (k = 0; k <= 5; k++) {
        if (cpu_clk_desired == 1500 * (1 << k)) {
            cpu_clk_new = cpu_clk_desired;
            CS_CTL0_DCORSEL_k = (k << 16);
            break;
        }
    }
    if (cpu_clk_new == 0) {
        sprintf(uart_txString, "CPU clock speed unchanged. Current value: %.1f MHz\n", (CS_getMCLK()/1000000.0));
        uart_send(uart_txString);
        uart_send("Use 'cpu clk [n]' to change the CPU clock speed to [n] kHz, where [n]=1500 x 2^k and [k]=0...5\n");
    } else {
        /* MSP432 Reference manual p. 425, 427: �C requires core voltage level 1 for 48 MHz.
         * Furthermore, MSP432 needs 1 wait state for accessing the Flash memory when operating
         * @ 48 MHz, because the Flash operates @ 12 MHz (max!).
         * Refer to MSP432 Datasheet(!), p.30 (section 5.8) for core voltage / wait state combinations
         * that fit to the desired CPU operating frequency.
         */
        if (k < 4) {    // f_CPU_new @ < 12 MHz: core voltage level 0, 0 wait states for Flash memory read access
            Interrupt_disableMaster();
            CS_setDCOCenteredFrequency(CS_CTL0_DCORSEL_k);  // set f_CPU BEFORE changing wait states and core voltage (crucial if coming from higher f_CPU)
            if (FlashCtl_getWaitState(FLASH_BANK0) != 0)
                FlashCtl_setWaitState(FLASH_BANK0, 0);  // set 0 wait states for flash bank 0 (default is 0 wait states)
            if (FlashCtl_getWaitState(FLASH_BANK1) != 0)
                FlashCtl_setWaitState(FLASH_BANK1, 0);  // set 0 wait states for flash bank 1 (default is 0 wait states)
            if (PCM_getCoreVoltageLevel() != PCM_VCORE0)
                PCM_setCoreVoltageLevel(PCM_VCORE0);    // set core voltage level 0          (default is VCORE0)
            Interrupt_enableMaster();
        } else if (k == 4) {  // f_CPU_new @ 24 MHz: core voltage level 1, 0 wait states for Flash memory read access
            Interrupt_disableMaster();
            if (CS_getMCLK() > 24000000)  // if coming from HIGHER f_CPU: set f_CPU BEFORE changing wait states and core voltage
                CS_setDCOCenteredFrequency(CS_CTL0_DCORSEL_k);
            if (PCM_getCoreVoltageLevel() != PCM_VCORE1)
                PCM_setCoreVoltageLevel(PCM_VCORE1);    // set core voltage level 1          (default is VCORE0)
            if (FlashCtl_getWaitState(FLASH_BANK0) != 0)
                FlashCtl_setWaitState(FLASH_BANK0, 0);  // set 0 wait states for flash bank 0 (default is 0 wait states)
            if (FlashCtl_getWaitState(FLASH_BANK1) != 0)
                FlashCtl_setWaitState(FLASH_BANK1, 0);  // set 0 wait states for flash bank 1 (default is 0 wait states)
            if (CS_getMCLK() < 24000000)  // if coming from LOWER f_CPU: set f_CPU AFTER changing wait states and core voltage
                CS_setDCOCenteredFrequency(CS_CTL0_DCORSEL_k);
            Interrupt_enableMaster();
        } else {  // f_CPU_new @ 48 MHz: core voltage level 1, 1 wait state for Flash memory read access
            Interrupt_disableMaster();
            if (FlashCtl_getWaitState(FLASH_BANK0) != 1)
                FlashCtl_setWaitState(FLASH_BANK0, 1);  // set 1 wait states for flash bank 0 (default is 0 wait states)
            if (FlashCtl_getWaitState(FLASH_BANK1) != 1)
                FlashCtl_setWaitState(FLASH_BANK1, 1);  // set 1 wait states for flash bank 1 (default is 0 wait states)
            if (PCM_getCoreVoltageLevel() != PCM_VCORE1)
                PCM_setCoreVoltageLevel(PCM_VCORE1);    // set core voltage level 1          (default is VCORE0)
            CS_setDCOCenteredFrequency(CS_CTL0_DCORSEL_k);  // set f_CPU after changing wait states and core voltage (crucial because coming from lower f_CPU)
            Interrupt_enableMaster();
        }
        SysTick_setPeriod(CS_getMCLK()/1000);       // SysTick Frequency: 1000 Hz -> 1 ms
        uart_changeBaudrate(uart_getBaudrate());    // peripheral clock change results in re-calculation of baudrate parameters
        spi_init(CPOL, CPHA, spi_getClockSpeed());  // peripheral clock change results in re-initializing of SPI communication
        sprintf(uart_txString, "CPU clock speed set to %.1f MHz\n", (CS_getMCLK()/1000000.0));
        uart_send(uart_txString);
        sprintf(uart_txString, "Core Voltage Level: %d\n", PCM_getCoreVoltageLevel());
        uart_send(uart_txString);
        sprintf(uart_txString, "Wait states Flash bank 0: %d\nWait states Flash bank 1: %d\n", FlashCtl_getWaitState(FLASH_BANK0), FlashCtl_getWaitState(FLASH_BANK1));
        uart_send(uart_txString);
    }
}

void cmd_change_spi_clk(void) {
    uint32_t spiClk_new = atoi(&uart_rxString[8]);
    if (spiClk_new < 10 || spiClk_new > 4000) {
        sprintf(uart_txString, "SPI clock speed unchanged. Current value: %d kHz\n", spi_getClockSpeed()/1000);
        uart_send(uart_txString);
        uart_send("Use 'spi clk [n]' to change the SPI clock speed to [n] Hz, where 10 < [n] < 4000.\n");
    } else {
        spi_changeClockSpeed(spiClk_new*1000);
        sprintf(uart_txString, "SPI clock speed set to %d kHz\n", spi_getClockSpeed()/1000);
        uart_send(uart_txString);
    }
}

void cmd_change_uart_baudrate(void) {
    uint32_t baudrate_new = atoi(&uart_rxString[8]);
    if (baudrate_new < 1200 || baudrate_new > 256000) {
        sprintf(uart_txString, "Baudrate unchanged. Current value: %d baud/s\n", uart_getBaudrate());
        uart_send(uart_txString);
        uart_send("Use 'bd [n]' to change the baudrate to [n] baud/s, where 1200 <= [n] <= 256000.\n");
    } else {
        uart_changeBaudrate(baudrate_new);
        sprintf(uart_txString, "Baudrate set to %d baud/s\n", uart_getBaudrate());
        uart_send(uart_txString);
    }
}

void calc_mean_and_stddev(void) {
    if (iv_test.idx < X_TEST_NUM) {
        iv_test.i_data[iv_test.idx] = physicalValues[1];
        iv_test.v_data[iv_test.idx] = physicalValues[0]*1000;   // in mV
        iv_test.idx++;
    } else {
        if (!iv_test.finished) {
            iv_test.i_stddev = stddev(iv_test.i_data, X_TEST_NUM);
            iv_test.i_mean = mean(iv_test.i_data, X_TEST_NUM);
            iv_test.v_stddev = stddev(iv_test.v_data, X_TEST_NUM);
            iv_test.v_mean = mean(iv_test.v_data, X_TEST_NUM);
            iv_test.finished = true;
        }
    }
}

void system_init(void) {
    /* Halting WDT  */
    WDT_A_holdTimer();
    /* FPU should be enabled by default, however make sure it always is enabled */
    FPU_enableModule();
    // f_CPU @ 24 MHz requires core voltage level 1 (OR 1 additional Flash wait state for reading Flash memory) for MSP432 to operate within specification
    Interrupt_disableMaster();
    PCM_setCoreVoltageLevel(PCM_VCORE1);    // set core voltage level 1 (default is VCORE0)
    Interrupt_enableMaster();
    /* Setting DCO to 24 MHz and SMCLK (Peripheral Clock used for Timers, UART, SPI) to 12 MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    /* Initializing Timing (stop watch for time measurement)*/
    t_init();
}

void launchpad_leds_init(void) {
    /* Red LED on MSP432 LaunchPad shut off (LOW) and set as output*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    /* RGB LED on MSP432 LaunchPad shut off (LOW) and set as output */
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
}

void systick_init(void) {
    SysTick_enableModule();
    SysTick_setPeriod(CS_getMCLK()/1000);  // SysTick Frequency: 1000 Hz -> 1 ms ticks
    SysTick_enableInterrupt();
}

