# Master MCU - C code (MPPT)

### What does this code do?
* ADC measurements
* Maximum Power Point Tracking (MPPT)
* Master control
* Communication to PC via UART

### What does this code _not_ do?
* Control of the Bi-directional DC/DC converter (BDC control)

## Most important files
* `main.c` - main program
* `my_adc.c` - ADC routines: measurement and conversion to physical values
* `my_spi.c` - SPI routines: processes transmitting and receiving data via SPI
* `my_uart.c` - UART routines: processes transmitting and receiving data via UART
* `my_timing.c` - Timing routines: stopwatch for time measurements inside the program

## How do I put the program into the MCU?
* Remove **5V jumper** (**J101** jumper bench) on [MSP432 LaunchPad](http://www.ti.com/tool/MSP-EXP432P401R) (Important! Otherwise, two 5V sources will feed the 5V power line on the PCB)
* Connect [MSP432 LaunchPad](http://www.ti.com/tool/MSP-EXP432P401R) to PC via USB
* Compile with [Code Composer Studio](http://www.ti.com/tool/CCSTUDIO) (Build Project) and Run-->Debug

## Detailed description of Master MCU code
The master MCU (**MSP432**) conducts all I,V measurements in the EPS.
It then executes the **MPPT** based on the outcome of those measurements.

The output of the MPPT is the **duty cycle set point** that will be sent to the µC slaves via SPI.
Applying this duty cycle demand onto the PWM of the BDC aims to move the PVA towards its current optimal operation point.
Consequently, the PVA shall be kept in its optimal operation point which is dependent on solar inclination and solar cell temperature.

Furthermore, the master µC also handles the excess energy management:
When the batteries are fully charged, the duty cycle must be adjusted in such a way that the PVA bus is moved to a voltage level above the PVA's optimal operating point,
where no output current is being generated in the PVA.


## Which voltages and currents does the MCU measure?
* **V_Bus** - voltage of PVA bus
* **I_Bus** - current from PVA (at PV bus voltage level)
* **I_Load** - current through loads (at PV bus voltage level)
* **V_Bat1** - voltage of battery 1
* **I_Bat1** - current into battery 1 (at battery voltage level)
* **V_Bat2** - voltage of battery 2
* **I_Bat2** - current into battery 2 (at battery voltage level)
* **I_PDU_5V** - current to PDU (at 5V level)
* **I_PDU_3.3V** - current to PDU (at 3.3V level)

Where:
* **I_Load** = **I_PDU_5V'** + **I_PDU_3.3V'**
* **I_Bus** = **I_Load** + **I_Bat1'** + **I_Bat2'**
* **V_Bus** = **V_Load** = **V_in(PDU_5V)** = **V_in(PDU_3.3V)**
* **I_Bat1'** and **I_Bat2'** are the current into the battery **before** the BDC (at PV bus voltage)
* **I_Bat1** and **I_Bat2** are the current into the battery **after** the BDC (at battery voltage)
* **I_PDU_5V'** and **I_PDU_3.3V'** are the currents **before** the PDUs (at PV bus voltage)
* **I_PDU_5V** and **I_PDU_3.3V** are the currents **after** the PDUs (voltage converted to 5V and 3.3V)


## Known issues
* Current tests have shown that the **SPI frequency** is somehow band-pass limited: The frequency must be above 70 kHz and below 3 MHz for a proper functioning SPI communication between MSP432 and PIC16. Though, this phenomen might be due to the fact that for testing the SPI communication, a voltage level shifter consisting of _74LS245N_ and _CD4050BE_ has been used which might be a limiting factor. However, using **400 kHz SPI clock speed** is a reasonable and sufficient fast frequency.


## List of abbreviations
* PDU - power distribution unit
* MPPT - Maximum Power Point Tracking
* BDC  - Bi-directional DC/DC converter
* MCU  - Microcontroller Unit
* PVA  - Photovoltaic Array
* EPS  - Electrical Power System
* FRDET - Fully regulated bus with direct energy transfer
* BCDET - Battery-clamped direct energy transfer
* µC   - Microcontroller
* PWM - Pulse Width Modulation
* PBC - Printed Circuit Board
* I - current
* V - voltage

 
## Who can I talk to?
* [Adrian Wenzel](mailto:a.wenzel@tum.de) 
* [Juan J. Rojas](mailto:juan.rojas@itcr.ac.cr)

 