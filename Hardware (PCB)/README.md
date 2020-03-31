# Hardware (PCB)

## Description
The **BDC prototype board** (v2.3) is based on a previous work of [Juan J. Rojas](mailto:juan.rojas@itcr.ac.cr).
It has been adapted to carry the [MSP432P401R LaunchPad](http://www.ti.com/tool/MSP-EXP432P401R), a development kit from Texas Instruments that holds the MSP432.
The advantage of using the LaunchPad instead of a naked MSP432 is being able to make use of already existing circuitry around the MPS432,
such as an appropriate power supply in addition to an on-board debugger and a backchannel UART.


## Known issues
### 5 V bus on PCB
The PCB 5 V bus can be powered by three different voltage sources:
* **5 V extern** - **[recommended]** using the 7.5 VDC power jack as input, the 5 V bus is powered through a 7805 linear regulator. Select through jumper `5V_EXT`.  
  Recommended, since the 7.5 VDC extern are **always used** in order to power the reference voltages [REF195](https://www.analog.com/en/products/ref195.html#product-overview)
  for the current sensors [ACS723](https://www.allegromicro.com/en/Products/Sense/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS723).
* **5 V intern** - using the 5 V power distribution unit on the PCB to power the 5 V bus. Select through jumper `5V_INT`
* **5 V from LaunchPad** - using the USB 5V from the LaunchPad to power the 5 V bus. Select through jumper `J101 5V` on LaunchPad.

Only one of the three above-mentioned jumpers can be plugged in at the same time!

## Workarounds
### 5 V bus on PCB
* In the _current state of the PCB design_, **always remove the J101 5V jumper** (J101 jumper bench on MSP432 LaunchPad)!  
  Otherwise, two different power supplies will feed the PCB 5V bus: 5V from USB **and** 5V from either _PCB intern_ (from PDU5V) or _PCB extern_ (7.5V in -> linear voltage regulator to 5V)  
  However, *USB must be connected* in order to feed the MSP432 with 3.3V (voltage regulator from 5V to 3.3V is on LaunchPad and cannot be powered by 5V bus)
* The pins for ADC reference voltage should have **two decoupling capacitors, 5 µF and 50 nF**. However, the current design of the PCB does not have those capacitors. They might be added by another small PCB to plug ontop of the LaunchPad.


### Power MSP432 through PCB-internal 3.3V power source
MSP432 _can_ be fed through PCB internal 3.3V from **PDU3.3V**. Not recommended. Do this only, when 3.3V supply from LaunchPad is not desired to power the MSP432!
* Remove **J101 3V jumper** (J101 jumper bench on MSP432 LaunchPad)
* **Activate PDU3.3V** through jumper on PCB and **connect 3.3V pin on MSP432 LaunchPad** through a cable to PDU3.3V_out
* Removal of **J101 3V jumper** is important! Otherwise, two different power supplies would feed the same 3.3V bus
 
## Who can I talk to?
* [Adrian Wenzel](mailto:a.wenzel@tum.de) 
* [Juan J. Rojas](mailto:juan.rojas@itcr.ac.cr)

