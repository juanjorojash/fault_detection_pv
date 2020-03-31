/**
 * @file main.c
 * @author Juan J. Rojas
 * @date 10 Nov 2018
 * @brief main loop for the BDC prototype converter.
 * @par Institution:
 * LaSEINE / CeNT. Kyushu Institute of Technology.
 * @par Mail (after leaving Kyutech):
 * juan.rojas@tec.ac.cr
 * @par Git repository:
 * https://bitbucket.org/juanjorojash/bdc_prototype/src/master
 */

#include "hardware.h"
#include "bdc_spi_slave.h"

uint8_t check_var = 0;
volatile uint8_t char_cnt2 = 0;
volatile bool spi_recepFlag = false;
volatile uint8_t spi_readVal = 0;
volatile uint8_t spi_action = 0;
const uint8_t magicNumber = 42;

/**@brief This is the main function of the program.
 */
void main(void) {
    initialize(); /// * Call the #initialize() function
    spi_slave_init(CPOL, CPHA, SS_ACTIVE); // cpol, cpha, SS_active
    __delay_ms(10);
    interrupt_enable();
    log_on = 0;
    while (1) {
        if (spi_recepFlag) {
            spi_recepFlag = false;
            switch (spi_action) {
                case 4:
                    START_CONVERTER(); /// -# Start the converter by calling the #START_CONVERTER() macro.
                    RESET_TIME();
                    break;
                case 5:
                    STOP_CONVERTER(); /// -# Stop the converter by calling the #STOP_CONVERTER() macro.
                    RESET_TIME();
                    break;
                case 6:
                    dc++;
                    if (dc > DC_MAX) dc = DC_MAX;
                    break;
                case 7:
                    dc--;
                    if (dc < DC_MIN) dc = DC_MIN;
                    break;
                default:
                    // nothing to do here
                    break;
            }
            check_var = spi_action;
        }
        if (recep_flag) {
            recep_flag = 0;
            switch (recep[0]) {
                case 0x01: ///
                    if (char_count == 3)
                        char_count++;
                    else
                        char_count = 0;
                    break;
                case 0x03:
                    if (recep[1] == 0x01 || char_count == 2)
                        char_count++;
                    else
                        char_count = 0;
                    break;
                case 0x04:
                case 0x05:
                case 0x06:
                case 0x07:
                case 0x08:
                    if (char_count == 1) {
                        action = recep[0];
                        char_count++;
                    } else char_count = 0;
                    break;
                default:
                    char_count = 0;
            }
            if (char_count == 4) {
                switch (action) {
                    case 0x04:
                        START_CONVERTER(); /// -# Start the converter by calling the #START_CONVERTER() macro.
                        RESET_TIME();
                        check_var = 4;
                        break;
                    case 0x05:
                        STOP_CONVERTER(); /// -# Stop the converter by calling the #STOP_CONVERTER() macro.
                        RESET_TIME();
                        check_var = 5;
                        break;
                    case 0x06:
                        dc++;
                        if (dc > DC_MAX) dc = DC_MAX;
                        check_var = 6;
                        break;
                    case 0x07:
                        dc--;
                        if (dc < DC_MIN) dc = DC_MIN;
                        check_var = 7;
                        break;
                    case 0x08:
                        break;
                }
                action = 0;
                char_count = 0;
            }
        }

        if (SECF) {
            SECF = 0;
            if (check_var) {
                UART_send_u16(check_var + 48);
                check_var = 0;
            }
            //            log_control_hex();
            //if ((vbatav < vbatmax) && (vbusr > ivbusr)) vbusr -= 2;
            if (vbatav < vbatmin)STOP_CONVERTER();
        }
    }
}

/**@brief This is the interruption service function. It will stop the process if an @b ESC or a @b "n" is pressed. 
 */
void __interrupt() ISR(void) {
    uint8_t spi_tx = 0;
    static uint8_t request_tmp = 0;
    static uint16_t dc_tmp = 0;
    if (SSP1IF) { // If MSSP Interupt Flag bit (SPI reception flag) is set then:
        SSP1IF = 0; // Clear interrupt flag
        spi_readVal = spi_read();
        /* echo back: received byte + magic number (will be send on next SPI transmission which is triggered by master µC)
         * Adding magicNumber to ensure that new value is sent and not the byte that has been received previously and that
         * still resides in the SPI RX/TX register (SSPSR). This might happen, if...
         * 1) Slave MCU (PIC16) is too slow, i.e. writes a new value too late to SSPBUF (buffer fro SSPSR)
         * 2) Master MCU (MSP432) is too fast, i.e. triggers already next byte exchange, when slave hasn't yet had
         *    a chance to write a new byte to SSPBUF. */
        spi_tx = spi_readVal + magicNumber;
        switch (request_tmp) {
            case 10:
                spi_tx = (uint8_t)(DC_MIN&0xFF) + magicNumber;
                break;
            case 11:
                spi_tx = (uint8_t)(dc_tmp&0xFF) + magicNumber;
                break;
            case 12:
                spi_tx = (uint8_t)(DC_MAX&0xFF) + magicNumber;
                break;
        }
        request_tmp = 0;
        if (spi_readVal > 3 && spi_readVal < 8) {   // only action, if SPI RX value is 4, 5, 6 or 7
            spi_recepFlag = true;
            spi_action = spi_readVal;
        } else if (spi_readVal == 9) {
            if (conv) {     // BDC is ON
                spi_tx = 1 + magicNumber;
            } else {        // BDC is OFF
                spi_tx = 2 + magicNumber;
            }
        } else if (spi_readVal == 10) {
            request_tmp = 10;
            spi_tx = (uint8_t)((uint16_t)DC_MIN >> 8) + magicNumber;
        } else if (spi_readVal == 11) {
            request_tmp = 11;
            dc_tmp = dc;
            spi_tx = (uint8_t)(dc_tmp >> 8) + magicNumber;
        } else if (spi_readVal == 12) {
            request_tmp = 12;
            spi_tx = (uint8_t)((uint16_t)DC_MAX >> 8) + magicNumber;
        }
        spi_write(spi_tx);
    }

    if (TMR1IF) {
        TMR1H = 0xE1; //TMR1 Fosc/4= 8Mhz (Tosc= 0.125us)
        TMR1L = 0x83; //TMR1 counts: 7805 x 0.125us = 975.625us
        TMR1IF = 0; //Clear timer1 interrupt flag
        vbus = read_ADC(VS_BUS); /// * Then, the ADC channels are read by calling the #read_ADC() function
        vbat = read_ADC(VS_BAT); /// * Then, the ADC channels are read by calling the #read_ADC() function
        ibat = (int16_t) (read_ADC(IS_BAT)); /// * Then, the ADC channels are read by calling the #read_ADC() function
        //HERE 2154 is a hack to get 0 current
        ibat = 2048 - ibat;
        if (conv) {
            //            control_loop(); /// -# The #control_loop() function is called*/
            //            //if ((vbat >= vbatmax) && (vbusr < voc)) vbusr +=1; ///NEEDS CORRECTION
            set_DC(&dc);
        }
        calculate_avg(); /// * Then, averages for the 250 values available each second are calculated by calling the #calculate_avg() function
        timing(); /// * Timing control is executed by calling the #timing() function         
    }

    if (RCIF)/// If the UART reception flag is set then:
    {
        if (RC1STAbits.OERR) /// * Check for errors and clear them
        {
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;
        }
        recep[1] = recep[0];
        recep[0] = RC1REG; /// * Empty the reception buffer and assign its contents to the variable @p recep   
        recep_flag = 1;
    }
}