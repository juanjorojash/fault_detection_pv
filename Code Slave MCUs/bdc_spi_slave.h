/* 
 * File:   bdc_spi_slave.h
 * Author: Adrian
 *
 * Created on 18. Februar 2020, 08:06
 */

#ifndef BDC_SPI_SLAVE_H
#define	BDC_SPI_SLAVE_H

/* Includes */
#include <stdint.h>
#include <stdbool.h>


/* Macros and defines */
#define CPOL 0  // 0 or 1
#define CPHA 0  // 0 or 1
#define SS_ACTIVE true
#define SS_INCATIVE false
#define spi_read() SSPBUF
#define spi_write(data) SSPBUF = (uint8_t)(data)


/* Function Declarations */
void init(void);
void spi_slave_init(uint8_t cpol, uint8_t cpha, bool SS_active);

#endif	/* BDC_SPI_SLAVE_H */

