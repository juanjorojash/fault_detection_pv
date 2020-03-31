# Embedded fault detection in PV modules usign electrical variables

## Acronyms ##
* **MCU:** microcontroller unit
* **MPU:** microprocessor unit 
* **EFS:** embedded fault detection system
* **CFS:** centralized faul detection system
* **PV:** photovoltaic


## Key features
* MCU: **MSP432** in a [MSP432P401R](http://www.ti.com/tool/MSP-EXP432P401R) for embedded detection algorithm implementation and communication with CPU. 
    * Programming language: **C**
* MPU: **Cortex-A72 (ARM v8-A)** in a [Raspberry Pi 4](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) for central data collection and higher level detection algorithm
    * Programming language: **Python**
* Hardware schematics: **EAGLE**



## What is the purpose of this project?
This project is about fault detection usign a combination of an EFS that can be connected close to the PV module and a CFS that will receive the data from several EFS to implement a higher level fault detection.  

## What does the MCU do?
The MCU conducts all I,V measurements in the PV module and use those measurements to detect fault by usign one or several algorithms. After finishing the analysis the MCU sends the data to the MPU. 

## What does the MPU do?
The MPU receive the data from several MCUs and performs a higher level algorithm for fault detection and fault location. 


## What about the hardware?
The **EFS prototype board** has been adapted to carry the [MSP432P401R LaunchPad](http://www.ti.com/tool/MSP-EXP432P401R), a development kit from Texas Instruments that holds the MSP432.
The advantage of using the LaunchPad instead of a naked MSP432 is being able to make use of already existing circuitry around the MPS432,
such as an appropriate power supply in addition to an on-board debugger and a backchannel UART.

## Known issues
* Not yet

## What does this repository contain?
<!-- * **C code Master MCU**: MSP432 - I,V measurements in EPS + MPPT + sending desired duty cycle to µC slaves via SPI
* **C code Slave MCUs**: PIC16F - reception of desired duty cycle via SPI from µC master + PWM generation, setting duty cycle
* **Hardware schematics**: BDC prototype board v2.3, adapted for conducting MPPT -->

## Who can I talk to?
* [Juan J. Rojas](mailto:juan.rojas@tec.ac.cr)
* [Luis D. Murillo](mailto:lmurillo@tec.ac.cr) 

