/*
 * my_spi.h
 *
 *  Created on: 14.01.2020
 *      Author: Adrian Wenzel
 */

#ifndef MY_SPI_H_
#define MY_SPI_H_

#include <stdint.h>
#include <stdbool.h>


#define SPI_TX_BUF_SIZE 20      // Number of bytes in SPI buffer. Currently, there are maximum 5 bytes send in a row.

// !SS1 on P2.4 (temporary on P2.6)
#define SS1_GPIO_PORT_Px    GPIO_PORT_P2
#define SS1_PxOUT           P2OUT           // output stage (register); must be correspond to GPIO_PORT above
#define SS1_GPIO_PINx       GPIO_PIN4
// !SS2 an P2.6
#define SS2_GPIO_PORT_Px    GPIO_PORT_P2
#define SS2_PxOUT           P2OUT           // output stage (register); must be correspond to GPIO_PORT above
#define SS2_GPIO_PINx       GPIO_PIN6
// Slave selects
#define SS1 1
#define SS2 2
// SPI answers
#define BDC_ON           1  // BDC is in ON state
#define BDC_OFF          2  // BDC is in OFF state
#define BDC_SWITCHED_ON  4  // BDC has been switched on
#define BDC_SWITCHED_OFF 5  // BDC has been switched off
#define BDC_INCREMENTED  6  // BDC duty cycle has been incremented
#define BDC_DECREMENTED  7  // BDC duty cycle has been decremented

void spi_init(uint8_t cpol, uint8_t cpha, uint32_t f_Hz_SPIclockSpeed);
uint8_t spi_send(uint8_t tx_data_SS1, uint8_t tx_data_SS2);
void spi_send_arrays(uint8_t *tx_data_SS1, uint8_t *tx_data_SS2);
void spi_read_old(uint8_t *rx1, uint8_t *rx2);
uint8_t spi_read(uint8_t SSx);
bool spi_newDataReceived(uint8_t SSx);
void spi_changeClockSpeed(uint32_t f_Hz_clockSpeed);
uint32_t spi_getClockSpeed(void);


#endif /* MY_SPI_H_ */
