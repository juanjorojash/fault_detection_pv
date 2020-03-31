## UART commands

Send the following digits as HEX or DEC over UART to BDC controller (PIC16) in order to control the BDC.
All digits in a row, one digit per byte, no spaces or anything else in between.

### BDC START
* Switches BDC on, start PWM with default duty cycle  
1 3 4 3 1

### BDC STOP
* Switches BDC off  
1 3 5 3 1

### Increment PWM_DC
* Increments PWM duty cycle of BDC, thus decrements bus voltage  
1 3 6 3 1

### Decrement PWM_DC
* Decrements PWM duty cycle of BDC, thus increments bus voltage  
1 3 7 3 1

### Reset PWM_DC
* _Supposed to reset BDC, but currently not implemented_  
1 3 9 3 1