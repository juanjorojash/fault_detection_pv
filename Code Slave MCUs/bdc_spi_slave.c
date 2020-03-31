/* 
 * File:   bdc_spi.c
 * Author: Adrian Wenzel
 *
 * Created on 4th December 2019
 */

// PIC16F1786 Configuration Word Register Settings
// Register CONFIG1
//#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
//#pragma config WDTE = OFF       // Watchdog Timer Disabled (WDT disabled)
//#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
//#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)//If this is enabled, the Timer0 module will not work properly.
//#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
//#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
//#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset enabled)
////#pragma config CLKOUTEN = OFF   // CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin (Default setting))
//#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
//#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
//
//// Register CONFIG2
//#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
//#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
//#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
//#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
//#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
//#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
//#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)//IF THIS IN ON MCLR is always enabled

// Includes
#include "bdc_spi_slave.h"
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

// Macros and defines
#ifndef _XTAL_FREQ
    #define _XTAL_FREQ 32000000
#endif
//#define CPOL 0  // 0 or 1
//#define CPHA 0  // 0 or 1
//#define spi_read() SSPBUF
//#define spi_write(data) SSPBUF = (uint8_t)data


/* Function Declarations */
void delay_ms(uint16_t ms_max_65535);
//void init(void);
//void spi_slave_init(uint8_t cpol, uint8_t cpha, bool SS_active);

//uint8_t rx_spi_global;
//uint8_t rx_spi_global_prev;
//uint8_t tx_spi_global;

int main_old(void) {
    init();
    spi_slave_init(CPOL, CPHA, true);   // cpol, cpha, SS_active
    uint8_t rx_spi = 0;
    uint8_t tx_spi = 0;
    SSP1IF = 0;     // Clear interrupt flag prior to enable interrupts
    PEIE = 1;       // Peripheral Interrupts Enable (for SPI)
    SSP1IE = 1;     // MSSP (SPI) Interrupt Enable
    GIE = 1;        // Global Interrupts Enable
    

    while (1) {
//        if (BF) {
//            rx_spi = spi_read();    // rx_spi = SSPBUF;
//            tx_spi = rx_spi + 13;
//            spi_write(tx_spi);       // SSPBUF = tx_spi;
//        }
    }
}

//void __interrupt() ISR() {
//    if (SSP1IF) {
//        SSP1IF = 0;     // Clear interrupt flag
//        rx_spi_global_prev = rx_spi_global;
//        rx_spi_global = spi_read(); // rx_spi = SSPBUF;
//        tx_spi_global = rx_spi_global + 42;
//        spi_write(tx_spi_global); // SSPBUF = tx_spi;
//    }
//}


void init(void) {
    OSCCONbits.SCS = 0x02; // Set clock source to internal oscillator
    OSCCONbits.IRCF = 0x0F; // Set clock to 16 MHz
    LATB = 0x00; // Port B: Output Latch Low
    TRISB &= ~0x10; // Set RB4 as output
    for (uint8_t i = 0; i < 5; i++) {
        RB4 = 1; 
        delay_ms(200);
        RB4 = 0;
        delay_ms(200);
    }
}


void delay_ms(uint16_t ms_max_65535) {
    uint16_t i;
    for (i = 0; i < ms_max_65535; i++) {
        __delay_us(1000);
    }
}

void spi_slave_init(uint8_t cpol, uint8_t cpha, bool SS_active) {
    // SSPSTAT - register configuration
    SSPSTATbits.SMP = 0;        // SMP bit must be cleared when in slave mode
    if (cpha)                   // CPHA = !CKE
        SSPSTATbits.CKE = 0;    // CPHA = 1: read data on 2nd edge of CLK line (active -> idle transition)
    else
        SSPSTATbits.CKE = 1;    // CPHA = 0: read data on 1st edge of CLK line (idle -> active transition)
    
    // SSPCON1 - register configuration
    if (cpol)                   // CPOL = CKP
        SSP1CON1bits.CKP = 1;   // Clock idle state is a HIGH level
    else
        SSP1CON1bits.CKP = 0;   // Clock idle state is a LOW level
    if (SS_active) {              // slave select input active
        SSP1CON1bits.SSPM = 0x04; // (= 0b0100) SPI slave mode, !SS pin control enabled
        ANSA5 = 0;                // = Digital I/O: Pin is assigned to port or digital special function
        TRISA5 = 1;               // set pin A5 as input: !SS
    } else {                      // don't use slave select input 
        SSP1CON1bits.SSPM = 0x05; // (= 0b0101) SPI slave mode, !SS pin control disabled
    }
    // set further input and output pins accordingly
    TRISC3 = 1; // input:  SCK
    TRISC4 = 1; // input:  SDI (MOSI)
    TRISC5 = 0; // output: SDO (MISO)
    
    /* Before enabling SPI module in slave mode, clock line (CLK) must match the
       proper idle state. Clock line can be observed by reading SCK pin. Idle
       state is determined by CKP bit of SSPCON1 register. */
    if (cpol) {         // Clock idle state is a HIGH level
        while (!RC3);   // wait for SCK to go to Idle state (=LOW on PIN RC3)
    } else {            // Clock idle state is a LOW level
        while (RC3);    // wait for SCK to go to Idle state (=HIGH on PIN RC3)
    }
    SSPCON1bits.SSPEN = 1; // enable SPI module
}
