/**
 * @file hardware.h
 * @author Juan J. Rojas
 * @date 10 Nov 2018
 * @brief Definitions for the BDC prototype controller
 * @par Institution:
 * LaSEINE / CeNT. Kyushu Institute of Technology.
 * @par Mail (after leaving Kyutech):
 * juan.rojas@tec.ac.cr
 * @par Git repository:
 * https://bitbucket.org/juanjorojash/bdc_prototype/src/master
 */

#ifndef HARDWARE_H
    #define HARDWARE_H
    // PIC16F1786 Configuration Bit Settings

    // 'C' source line config statements
    #pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
    #pragma config WDTE = OFF       // Watchdog Timer Disabled (WDT disabled)
    #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
    #pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)//If this is enabled, the Timer0 module will not work properly.
    #pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
    #pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
    #pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset enabled)
    #pragma config CLKOUTEN = ON    // Clock Out Negative Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
    #pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
    #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

    // CONFIG2
    #pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
    #pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
    #pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
    #pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
    #pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
    #pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
    #pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)//IF THIS IN ON MCLR is always enabled

    #include <xc.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <math.h>
    #include <stdint.h> // To include uint8_t and uint16_t
    #include <string.h>
    #include <stdbool.h> // Include bool type

#define 	_XTAL_FREQ 				32000000
#define		ERR_MAX					1000
#define		ERR_MIN					-1000
#define 	DC_MIN                  178 // DC = 0.35 min value 4.2V/0.35=12V max (LTC3112 works up to 15 V)
#define 	DC_MAX                  485	// DC = 0.95 max value
#define     DC_DEFAULT              461 // DC = 0.9  default value (reset BDC reset)
#define     KP                      15 ///< Proportional constant divider 
#define     KI                      35 ///< Integral constant divider 
#define     VREF                    4800                  
#define     sVREF                   (uint16_t) ( ( ( VREF * 4096.0 ) / 5935 ) + 0.5 )
#define     CREF                    2000                  
#define     sCREF                   (uint16_t) ( ( ( CREF * 4096.0 ) / (5000 * 2.5 * 5 ) ) + 0.5 )
#define     VOC                     5400
#define     sVOC                    (uint16_t) ( ( ( VOC * 4096.0 ) / 5935 ) + 0.5 )
#define     VBATMIN                 2500
#define     sVBATMIN                (uint16_t) ( ( ( VBATMIN * 4096.0 ) / 5000 ) + 0.5 )
#define     VBATMAX                 4150
#define     sVBATMAX                (uint16_t) ( ( ( VBATMAX * 4096.0 ) / 5000 ) + 0.5 )
#define     COUNTER                 128
#define		VS_BUS                  0b00010 //AN2 (RA2) 
#define		VS_BAT                  0b00001 //AN1 (RA1) 
#define		IS_BAT                  0b00000 //AN0 (RA0) 

bool                                recep_flag = 0;
bool                                SECF = 0;
uint16_t                            count = COUNTER + 1; ///< Counter that should be cleared every second. Initialized as #COUNTER 
uint8_t                             char_count = 0;
char                                action = 0;
char                                recep[12] = {0x00};
int24_t                             intacum = 0;   ///< Integral acumulator of PI compensator
uint16_t                            dc = DC_DEFAULT;  ///< Duty cycle     
bool                                log_on = 0; ///< Variable to indicate if the log is activated 
int16_t                             second = -1; ///< Seconds counter, resetted after 59 seconds.
uint16_t                            minute = 0; ///< Minutes counter, only manually reset
bool                                conv = 0; ///< Turn controller ON(1) or OFF(0). Initialized as 0
uint16_t                            vbus = 0;
uint16_t                            vbat = 0;
int16_t                             ibat = 0;
uint24_t                            vbusac = 0;
uint24_t                            vbatac = 0;
int24_t                             ibatac = 0;
uint16_t                            vbusav = 0;
uint16_t                            vbatav = 0;
int16_t                             ibatav = 0;
//uint16_t                            capap = 0;
//uint16_t                            cvref = 4800;
//uint16_t                            ocref = 2000;
uint16_t                            vbusr = sVREF;    
const uint16_t                      ivbusr = sVREF; 
//const uint16_t                      iref = sCREF;
const uint16_t                      voc = sVOC;
const uint16_t                      vbatmin = sVBATMIN;
const uint16_t                      vbatmax = sVBATMAX;
        
void initialize(void);
void pid(uint16_t feedback, uint16_t setpoint, int24_t* acum, uint16_t* duty_cycle);
void set_DC(uint16_t* duty_cycle);
uint16_t read_ADC(uint16_t channel);
void log_control_hex(void);
void display_value_u(uint16_t value);
void display_value_s(int16_t value);
void control_loop(void);
void calculate_avg(void);
void interrupt_enable(void);
void UART_send_char(char bt);
char UART_get_char(void); 
void UART_send_string(const char* st_pt);
void UART_send_u16(uint16_t number); 
void timing(void);
void PAO(uint16_t pv_voltage, uint16_t pv_current, uint32_t* previous_power, char* previous_direction);


#define     LINEBREAK               {UART_send_char(0xA); UART_send_char(0xD);}
#define     HEADER                  {UART_send_char(0x01); UART_send_char(0x02);}
#define     FOOTER                  {UART_send_char(0x02); UART_send_char(0x01);}
#define     RESET_TIME()            { minute = 0; second = -1; count = COUNTER + 1;} ///< Reset timers.
//DC-DC CONVERTER RELATED DEFINITION
#define		STOP_CONVERTER()		{ vbusr = ivbusr; dc = DC_DEFAULT; set_DC(&dc); RA4 = 0; log_on = 1; intacum = 0; conv = 0; TRISC0 = 1;}
#define  	START_CONVERTER()		{ vbusr = ivbusr; dc = DC_DEFAULT; set_DC(&dc); RA4 = 1; log_on = 1; intacum = 0; conv = 1; TRISC0 = 0;}

#endif /* HARDWARE_H*/