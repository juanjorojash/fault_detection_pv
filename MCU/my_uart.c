/*
 * my_uart.c
 *
 *  Created on: 14.01.2020
 *      Author: Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Custom Includes */
#include "my_uart.h"

/* Defines */
#define NUM_LIST_ENTRIES_UCBRSX 36
#define INCREMENT_IDX_TX_RING_BUF(i) i++; if(i >= UART_TX_BUF_SIZE) i = 0

/* Global variables, but not public */
// From MSP432 Reference Manual:
static unsigned int UCBRSx_list[NUM_LIST_ENTRIES_UCBRSX] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x11, 0x21, 0x22, 0x44, 0x25, 0x49, 0x4A, 0x52, 0x92, 0x53, 0x55, 0xAA, 0x6B, 0xAD, 0xB5, 0xB6, 0xD6, 0xB7, 0xBB, 0xDD, 0xED, 0xEE, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE
};
static float frac_list[NUM_LIST_ENTRIES_UCBRSX] = {
    0.0000, 0.0529, 0.0715, 0.0835, 0.1001, 0.1252, 0.1430, 0.1670, 0.2147, 0.2224, 0.2503, 0.3000, 0.3335, 0.3575, 0.3753, 0.4003, 0.4286, 0.4378, 0.5002, 0.5715, 0.6003, 0.6254, 0.6432, 0.6667, 0.7001, 0.7147, 0.7503, 0.7861, 0.8004, 0.8333, 0.8464, 0.8572, 0.8751, 0.9004, 0.9170, 0.9288
};

static volatile char tx_buffer[UART_TX_BUF_SIZE];
static volatile uint16_t txSendIndex = 0;
static volatile bool txActive = false;          // indicates if there's an ongoing transmission
static volatile bool txFinished = false;
static volatile uint16_t dataToSend = 0;
static uint16_t txBufIndex = 0;
static volatile char rx_buffer[UART_RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;
static volatile bool unreadString = false;
static bool tx_interrupts = false;
static uint32_t baudrate_set = 0;


/* Function Declarations */
void (*uart_send_Func)(char *string);
void uart_send_withoutInterrupt(char *string);
void uart_send_withInterrupt(char *string);


/* Functions */
void uart_init(uint32_t baudrate, bool tx_using_interrupts) {
    // Variables needed for register settings calculation for desired baudrate
    uint32_t f_BRCLK = CS_getSMCLK();
    float N = 0;           // division factor for baudrate
    float N_frac = 0;      // fractional part of N
    uint8_t OS16 = 0;       // oversampling flag
    uint16_t UCBRx = 0;     // baudrate register (clock prescaler)
    uint8_t UCBRFx = 0;     // first stage modulator
    uint8_t UCBRSx = 0;     // second stage modulator
    uint8_t i = 0;          // for loop index

    // check if SMCLK (input clock for UART) is sufficiently high
    if (baudrate > f_BRCLK/2) {
        baudrate = 9600;        // if SMCLK too low, set UART to standard baudrate 9600
    }
    baudrate_set = baudrate;

    // calculate register settings for desired baudrate, based on SMCLK clock speed
    N = (float)f_BRCLK / (float)baudrate;

    if (N > 19) {
        OS16 = 1;
        UCBRx = (int)(N/16);
        UCBRFx = (int)(((N/16) - (int)(N/16)) * 16);
    } else {
        OS16 = 0;
        UCBRx = (int)N;
    }
    N_frac = N - (int)N;

    for (i = 0; i < NUM_LIST_ENTRIES_UCBRSX; i++) {
        if (N_frac >= frac_list[i]) {
            UCBRSx = UCBRSx_list[i];
        }
    }

    eUSCI_UART_ConfigV1 uartConfig =
    {
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source = 750 kHz, so the following 3 Options
     UCBRx,                                   // clock prescaler
     UCBRFx,                                  // firstModReg
     UCBRSx,                                  // secondModReg
     EUSCI_A_UART_NO_PARITY,                  // No Parity
     EUSCI_A_UART_LSB_FIRST,                  // LSB First
     EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
     EUSCI_A_UART_MODE,                       // UART mode
     OS16,                                    // Oversampling
     EUSCI_A_UART_8_BIT_LEN                   // 8 bit data length
    };
    /* Selecting P1.2 and P1.3 in UART mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Initialize UART Module:  e.g. 9600 8N1 => 1,042ms/Byte => 960 Bytes/sec, (incl. start bit + stop bit) */
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    /* Enable UART module */
    UART_enableModule(EUSCI_A0_BASE);
    /* Enable interrupts */
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
    if (tx_using_interrupts) {
        tx_interrupts = true;
        uart_send_Func = uart_send_withInterrupt;
    } else {
        tx_interrupts = false;
        uart_send_Func = uart_send_withoutInterrupt;
    }
}

uint32_t uart_getBaudrate() {
    return baudrate_set;
}

void uart_changeBaudrate(uint32_t baudrate) {
    Interrupt_disableInterrupt(INT_EUSCIA0);
    uint16_t UCA0IE_tmp = UCA0IE;           // save status of eUSCI_A0 Interrupt Enable Register
    UCA0IE = 0;                             // disable RX and TX interrupts if enabled
    UART_disableModule(EUSCI_A0_BASE);
    baudrate_set = baudrate;
    uart_init(baudrate, tx_interrupts);
    UCA0IE = UCA0IE_tmp;                    // restore status of eUSCI_A0 Interrupt Enable Register
}

/* uart_send() puts the passed string into a ring buffer. So, practically all passed string will
 * be sent. If the ring buffer is being filled faster than data can be transmitted via UART, older
 * strings will be overwritten and not be sent.
 * This can practically happen if
 *    a) the ring buffer is too small
 *    b) if the baudrate used is too low
 * Countermeasures are increasing the TX ring buffer size or the baudrate.
 * Disabling TX interrupts "uart_init(xxxxx, TX_WITHOUT_INTERRUPTS)" will not use the ring buffer,
 * but definitely send all strings being passed to uart_send(). However, the program will be much slower
 * since the CPU is blocked for long times during sending UART data (if using a low baudrate).
 * Time for sending one byte:
 *   - baud 9600: 1,04 ms
 *   - baud 38400: 260 µs  (works with 1,5 MHz CPU clock)
 *   - baud 115200: 87 µs  (needs higher CPU clock, e.g. 12 MHz)
 *   - baud 256000: 39 µs
 */
void uart_send(char *string) {
    uart_send_Func(string);
}

void uart_send_withoutInterrupt(char *string) {
    while (*string) {
        UART_transmitData(EUSCI_A0_BASE, *string++);
    }
}

void uart_tx(void)                  // called by TX-ISR() uart_send_withInterrupt()
{
    if (dataToSend) {
        dataToSend--;
        UCA0TXBUF = tx_buffer[txSendIndex];
        INCREMENT_IDX_TX_RING_BUF(txSendIndex);
    } else {
        txActive = false;
        txFinished = true;
        UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    }
}

bool uart_transmissionActive(void) {
    return txActive;
}

bool uart_transmissionFinished(void) {
    if (txFinished && !txActive) {
        txFinished = false;
        return true;
    }
    return false;
}

void uart_send_withInterrupt(char *string)
{
    /* Disable "TX complete" interrupt while putting new string into TX ring buffer, because
     * "dataToSend" is affected in while loop below AND in TX ISR! So, it might happen that
     * "dataToSend" grows too slowly in while below while various data are already being sent.
     * This is why TX interrupt must be disabled which results in stop sending data while putting
     * the new string into TX ring buffer.
     */
    uint16_t UCA0IE_temp = UCA0IE;               // save status of eUSCI_A0 Interrupt Enable Register
    UCA0IE &= ~EUSCI_A_UART_TRANSMIT_INTERRUPT;  // disable "TX complete" interrupt
    while (*string) {
        tx_buffer[txBufIndex] = *string;
        INCREMENT_IDX_TX_RING_BUF(txBufIndex);
        dataToSend++;
        string++;
    }
    UCA0IE = UCA0IE_temp;               // restore status of eUSCI_A0 Interrupt Enable Register
    if (!txActive) {                    // No interrupt is generated if there's no ongoing transmission
        txActive = true;                // Transmission must therefore be started manually
        uart_tx();
        UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    }
}

void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {
        rx_buffer[rx_idx] = UART_receiveData(EUSCI_A0_BASE);
        if (rx_buffer[rx_idx] == '\0') {
            rx_idx = 0;
            unreadString = true;
        } else {
            rx_idx++;
        }
    }
    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) {
        uart_tx();
    }
}

bool uart_stringToRead(void) {
    return unreadString;
}

/* uart_read() only reads the newest received string.
 * No ring buffer is used for RX. If a string has not been read and another string gets received,
 * the old string will be overwritten.
 * That's why "uart_stringToRead()" should be called frequently with small time intervalls to check,
 * if there's an unreadString pending.
 */
void uart_read(char* Buffer) {
    uint8_t rxReadIndex = 0;
    while (rx_buffer[rxReadIndex] != '\0') {
        *Buffer++ = rx_buffer[rxReadIndex++];
    }
    *Buffer = '\0';            // append '\0' to create a standard C string
    unreadString = false;
}

