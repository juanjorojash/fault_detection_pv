/*
 * my_spi.c
 *
 *  Created on: 21.01.2020
 *      Author: Adrian Wenzel
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Custom Includes */
#include "my_spi.h"


/* Defines */
#define SS1_IDLE    SS1_PxOUT |= SS1_GPIO_PINx
#define SS2_IDLE    SS2_PxOUT |= SS2_GPIO_PINx
#define SS1_ACTIVE  SS1_PxOUT &= ~SS1_GPIO_PINx;
#define SS2_ACTIVE  SS2_PxOUT &= ~SS2_GPIO_PINx;

#define CPOL_0 EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW
#define CPOL_1 EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
#define CPHA_0 EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
#define CPHA_1 EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT

#define INCREMENT_IDX_TX_RING_BUF(i) i++; if(i >= SPI_TX_BUF_SIZE) i = 0

typedef enum {
    SPI_SLAVE1, SPI_SLAVE2, NONE
} last_SPI_transmission_t;


/* Global variables, only visible in this source file */
static volatile uint8_t rxDummy = 0;
static volatile uint8_t RXData_SS1 = 0;
static volatile uint8_t RXData_SS2 = 0;
static uint8_t TXData_SS2 = 0;
static volatile bool spi_newdata_received = false;
static volatile uint8_t spi_newData_received = 0;
//static volatile bool transmitActive = false;
static volatile uint8_t transmitActive = false;
static eUSCI_SPI_MasterConfig spiMasterConfig;
//static uint8_t spi_dataToSend = 0;
static volatile uint8_t spi_SS1_dataToSend = 0;
static volatile uint8_t spi_SS2_dataToSend = 0;
static uint8_t spi_tx_buffer1[SPI_TX_BUF_SIZE];
static uint8_t spi_tx_buffer2[SPI_TX_BUF_SIZE];
static volatile uint8_t spi_txBufIndex1 = 0;
static volatile uint8_t spi_txBufIndex2 = 0;
static volatile uint8_t spi_txSendIndex1 = 0;
static volatile uint8_t spi_txSendIndex2 = 0;
static last_SPI_transmission_t last_SPI_transmission = NONE;



/* Function Declarations */
void spi_tx(void);
void spi_tx1(uint8_t tx_data);
void spi_tx2(uint8_t tx_data);
void spi_send_arrays(uint8_t *tx_data_SS1, uint8_t *tx_data_SS2);

/* Functions, public and private ones */
void spi_init(uint8_t cpol, uint8_t cpha, uint32_t f_Hz_SPIclockSpeed) {
    /* Selecting P1.5, P1.6 and P1.7 in SPI mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring !SS pins for SPI */
    GPIO_setOutputHighOnPin(SS1_GPIO_PORT_Px, SS1_GPIO_PINx);   // Idle state
    GPIO_setAsOutputPin(SS1_GPIO_PORT_Px, SS1_GPIO_PINx);
    GPIO_setOutputHighOnPin(SS2_GPIO_PORT_Px, SS2_GPIO_PINx);   // Idle state
    GPIO_setAsOutputPin(SS2_GPIO_PORT_Px, SS2_GPIO_PINx);

    spiMasterConfig.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
    spiMasterConfig.clockSourceFrequency = CS_getSMCLK();
    spiMasterConfig.desiredSpiClock = f_Hz_SPIclockSpeed;
    spiMasterConfig.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    spiMasterConfig.spiMode = EUSCI_B_SPI_3PIN;    /* 3-Wire SPI Mode. When more than 1 slave: UCxSTE can not
                                                      be used for !SS. Instead GPIOs must be used for this purpose. */
    if (cpol)
        spiMasterConfig.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;  // CPOL = 1: Clock idle state is a HIGH level
    else
        spiMasterConfig.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;   // CPOL = 0: Clock idle state is a LOW level

    if (cpha)
        spiMasterConfig.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;   // CPHA = 1: read data on 2nd edge of CLK line (active -> idle transition)
    else
        spiMasterConfig.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;   // CPHA = 0: read data on 1st edge of CLK line (idle -> active transition)

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_B0_BASE);
    /* Enabling interrupts */
    SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_SPI_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIB0);
}

uint32_t spi_getClockSpeed(void) {
    return spiMasterConfig.desiredSpiClock;
}

void spi_changeClockSpeed(uint32_t f_Hz_clockSpeed) {
    Interrupt_disableInterrupt(INT_EUSCIB0);
    uint16_t UCB0IE_tmp = UCB0IE;           // save status of eUSCI_B0 Interrupt Enable Register
    UCB0IE = 0;                             // disable RX and TX interrupts if enabled
    SPI_changeMasterClock(EUSCI_B0_BASE, CS_getSMCLK(), f_Hz_clockSpeed); // Initializes the SPI Master clock. At the end of this function call, SPI module is left enabled.
    spiMasterConfig.desiredSpiClock = f_Hz_clockSpeed;
    UCB0IE = UCB0IE_tmp;
    Interrupt_enableInterrupt(INT_EUSCIB0);
}

/* Returns true, if there is unread data,
 *         false, if not.
 */
bool spi_newDataReceived_old(void) {
    return spi_newdata_received;
}


bool spi_newDataReceived(uint8_t SSx) {
    return (spi_newData_received & SSx);
}


uint8_t spi_read_old_old(void) {
    spi_newdata_received = false;
    return RXData_SS1;
}

void spi_read_old(uint8_t *rx1, uint8_t *rx2) {
    spi_newdata_received = false;
    *rx1 = RXData_SS1;
    *rx2 = RXData_SS2;
}

uint8_t spi_read(uint8_t SSx) {
    spi_newData_received &= ~SSx;
    if (SSx == 1)
        return RXData_SS1;
    else if (SSx == 2)
        return RXData_SS2;
    else
        return 0;
}

void spi_send_arrays(uint8_t *tx_data_SS1, uint8_t *tx_data_SS2) {
    while (*tx_data_SS1) {
        spi_tx_buffer1[spi_txBufIndex1] = *tx_data_SS1;
        INCREMENT_IDX_TX_RING_BUF(spi_txBufIndex1);
//        spi_dataToSend++;
        spi_SS1_dataToSend++;
        tx_data_SS1++;
    }
    while (*tx_data_SS2) {
        spi_tx_buffer2[spi_txBufIndex2] = *tx_data_SS2;
        INCREMENT_IDX_TX_RING_BUF(spi_txBufIndex2);
//        spi_dataToSend++;
        spi_SS2_dataToSend++;
        tx_data_SS2++;
    }

    if (!transmitActive) {
        spi_tx();   // must be called to start transmission. If transmission is already ongoing, it will be called by ISR()
    }

}

void spi_tx1(uint8_t tx_data) {
    SS1_ACTIVE;
    while ( !(UCB0IFG & EUSCI_SPI_TRANSMIT_INTERRUPT) );    // wait until SPI transmit flag is set (= check that previous transmission is complete)
    UCB0TXBUF = tx_data;       // put tx data into transmit buffer to initiate transmission
    last_SPI_transmission = SPI_SLAVE1;
}

void spi_tx2(uint8_t tx_data) {
    SS2_ACTIVE;
    while ( !(UCB0IFG & EUSCI_SPI_TRANSMIT_INTERRUPT) );    // wait until SPI transmit flag is set (= check that previous transmission is complete)
    UCB0TXBUF = tx_data;       // put tx data into transmit buffer to initiate transmission
    last_SPI_transmission = SPI_SLAVE2;
}

void spi_tx(void)                  // called by TX-ISR() uart_send_withInterrupt()
{
    if (spi_SS1_dataToSend && spi_SS2_dataToSend) {
        if (last_SPI_transmission == SPI_SLAVE2) {
            spi_SS1_dataToSend--;
            transmitActive = 1;
            spi_tx1(spi_tx_buffer1[spi_txSendIndex1]);
            INCREMENT_IDX_TX_RING_BUF(spi_txSendIndex1);
        } else if (last_SPI_transmission == SPI_SLAVE1) {
            spi_SS2_dataToSend--;
            transmitActive = 2;
            spi_tx2(spi_tx_buffer2[spi_txSendIndex2]);
            INCREMENT_IDX_TX_RING_BUF(spi_txSendIndex2);
        } else {
            spi_SS1_dataToSend--;
            transmitActive = 1;
            spi_tx1(spi_tx_buffer1[spi_txSendIndex1]);
            INCREMENT_IDX_TX_RING_BUF(spi_txSendIndex1);
        }
    } else if (spi_SS1_dataToSend) {
        spi_SS1_dataToSend--;
        transmitActive = 1;
        spi_tx1(spi_tx_buffer1[spi_txSendIndex1]);
        INCREMENT_IDX_TX_RING_BUF(spi_txSendIndex1);
    } else if (spi_SS2_dataToSend) {
        spi_SS2_dataToSend--;
        transmitActive = 2;
        spi_tx2(spi_tx_buffer2[spi_txSendIndex2]);
        INCREMENT_IDX_TX_RING_BUF(spi_txSendIndex2);
    } else {
        transmitActive = 0;
    }

}


uint8_t spi_send(uint8_t tx_data_SS1, uint8_t tx_data_SS2) {
    uint8_t UCB0TXBUF_tmp;
    TXData_SS2 = tx_data_SS2;
    SS1_ACTIVE;
    while ( !(UCB0IFG & EUSCI_SPI_TRANSMIT_INTERRUPT) );    // wait until SPI transmit flag is set (= check that previous transmission is complete)
    UCB0TXBUF = tx_data_SS1;       // put tx data into transmit buffer to initiate transmission
    transmitActive = true;
    UCB0TXBUF_tmp = UCB0TXBUF;
    return UCB0TXBUF_tmp;
}

uint8_t spi_send_SS1(uint8_t tx_data) {
    uint8_t UCB0TXBUF_tmp;
    while (transmitActive);     // wait until "transmitActive" has been cleared, i.e. no further data is being send at the moment
    SS1_ACTIVE;
    while ( !(UCB0IFG & EUSCI_SPI_TRANSMIT_INTERRUPT) );
    UCB0TXBUF = tx_data;       // Transmit data
    transmitActive = true;
    UCB0TXBUF_tmp = UCB0TXBUF;
    return UCB0TXBUF_tmp;
}

uint8_t spi_send_SS2(uint8_t tx_data) {
    uint8_t UCB0TXBUF_tmp;
    while (transmitActive);     // wait until "transmitActive" has been cleared, i.e. no further data is being send at the moment
    SS2_ACTIVE;
    while ( !(UCB0IFG & EUSCI_SPI_TRANSMIT_INTERRUPT) );
    UCB0TXBUF = tx_data;       // Transmit data
    UCB0TXBUF_tmp = UCB0TXBUF;
    return UCB0TXBUF_tmp;
}


//******************************************************************************
//
//This is the EUSCI_B0 interrupt vector service routine.
//
//******************************************************************************
void EUSCIB0_IRQHandler_old(void)
{
    uint32_t status = SPI_getEnabledInterruptStatus(EUSCI_B0_BASE);
    if (status & EUSCI_SPI_RECEIVE_INTERRUPT)
    {
        if (transmitActive) {
            SS1_IDLE;
            RXData_SS1 = SPI_receiveData(EUSCI_B0_BASE);
            transmitActive = 0;
            spi_send_SS2(TXData_SS2);
        } else {
            SS2_IDLE;
            RXData_SS2 = SPI_receiveData(EUSCI_B0_BASE);
            spi_newdata_received = true;
        }
    }
}

//******************************************************************************
//
// EUSCI_B0 interrupt vector service routine (SPI routine)
//
//******************************************************************************
void EUSCIB0_IRQHandler(void)
{
    uint32_t status = SPI_getEnabledInterruptStatus(EUSCI_B0_BASE);
    if (status & EUSCI_SPI_RECEIVE_INTERRUPT)
    {
        SS1_IDLE;
        SS2_IDLE;
        if (transmitActive == 1) {

            RXData_SS1 = SPI_receiveData(EUSCI_B0_BASE);   // read SPI receive register in order to clear interrupt flag
            spi_newData_received |= 1;
            spi_tx();
        } else if (transmitActive == 2) {

            RXData_SS2 = SPI_receiveData(EUSCI_B0_BASE);   // read SPI receive register in order to clear interrupt flag
            spi_newData_received |= 2;
            spi_tx();
        } else {
            rxDummy = SPI_receiveData(EUSCI_B0_BASE);   // read SPI receive register in order to clear interrupt flag
        }
    }
}


