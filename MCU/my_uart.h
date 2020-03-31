/*
 * my_uart.h
 *
 *  Created on: 14.01.2020
 *      Author: Adrian
 */

#ifndef MY_UART_H_
#define MY_UART_H_

#include <stdint.h>
#include <stdbool.h>

#define UART_TX_BUF_SIZE 1000   // Should be big (typ. ~1000) if using UART TX interrupts. Will carry all strings that were issued to send but aren't yet transmitted. If no TX interrupts are used the same size as for the tx_string in main program is sufficient (typ. ~100)
#define UART_RX_BUF_SIZE 100    // No need to be as big as TX, because incoming strings (e.g. sent by PC) are normally smaller than 100 characters
#define TX_WITH_INTERRUPTS true
#define TX_WITHOUT_INTERRUPTS false

/*
 * uart_init():
 * Receive routine is always using interrupts to fill the RX buffer.
 * Transmit routine only, when UART is initialized to do so.
 */
void uart_init(uint32_t baudrate, bool tx_using_interrupts);
void uart_changeBaudrate(uint32_t baudrate);
uint32_t uart_getBaudrate();
void uart_send(char *string);
bool uart_stringToRead(void);
void uart_read(char* Buffer);
bool uart_transmissionActive(void);
bool uart_transmissionFinished(void);



#endif /* MY_UART_H_ */
