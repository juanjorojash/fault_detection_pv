# Master MCU - UART commands


## How to set up communication?
* Micro USB cable from **PC to MSP432 LaunchPad** (communication uses back-channel UART of LaunchPad)
* UART settings: 256.000 baud/s, 8n1
* send commands in Ascii, e.g. using [HTerm](http://www.der-hammer.info/pages/terminal.html), Send on enter: _Null_


## UART commands
* `type` or `Type` - returns a standard string that describes the device and its purpose (_"MSP432P401R LaunchPad on EPS prototype: FRDET with MPPT"_)
* `spi [on/off]` - **[default: ON]** enable (`on`) or disable (`off`) SPI communication from master MCU (MSP432) to slave MCUs (PIC16)
* `bdc [on/off]` - **[default: OFF]** switch **both** BDCs `on` or `off` _(master MCU sends SPI command to slaves)_
* `bdc [-/+]` - decrease (`-`) or increase (`+`) duty cycle of BDCs by one step _(master MCU sends SPI command to slaves)_
* `bdc [min/max/act]` - get minimum (`min`), maximum (`max`) or actual (`act`) duty cycle of BDCs _(master MCU sends SPI command to slaves)_
* `ctrl` or `mppt` - shows which control modules are active and inactive
* `ctrl timing` or `mppt timing` - **[default: OFF]** enables/disables printing MPPT routine timing (cock cycles and duration) via UART
* `ctrl int [x]` or `mppt int [x]` - changes the control loop execution interval (CLEI) to `[x]` ms. Send `ctrl int` or `mppt int` to get the current value.
* `mppt [on/off]` - **[default: OFF]** enable/disable maximum power point tracking
* `pvset [on/off]` - **[default: OFF]** enable/disable fixed voltage set point for V_bus. Default, if enabled: 4.5 V
* `pvset [x]` - **[default: OFF]** set the voltage set point for V_bus to `[x]` mV and enable it. Send `pvset` to get current state and setpoint value.
* `lp [on/off]` - **[default: OFF]** enable/disable low-pass filter that filters ADC measurement values before using them in the control loop. Send `lp` to get the current state.
* `lp [x]` - set filter constant _tau_ to `[x]` ms. Send `lp` to get the current value. Cut-off frequency is **f_c = 1/(2\*pi\*tau)**.
* `dc print` or `print dc` - **[default: OFF]** enables/disables printing duty cycle cmd (SPI and simulated) via UART
* `dc reset` or `reset dc` - reset the duty cycle to DEFAULT value; only for simulation purposes, affects only _pv_sim()_
* `bat sim [x]` - set the simulated battery's voltage to [x] mV; only relevant for the simulation case _pv_sim()_
* `sweep` - run a sweep through solar cell voltage to determine I-V characteristics;only for simulation purposes, affects only _pv_sim()_
* `uart bd` - returns current UART baudrate of master MCU
* `uart bd [x]` - sets UART baudrate to `[x]` baud/s
* `spi clk` - returns current SPI clock frequency (communication between master and slave MCUs)
* `spi clk [x]` - sets SPI clock frequency to `[x]` kHz
* `cpu clk` - returns current CPU freqency (master clock) of master MCU
* `cpu clk [x]` - set CPU clock to `[x]` kHz. Allowed values are: 1500, 3000, 6000, 12000, 24000 and 48000.
* `adc timing` - **[default: OFF]** enables/disables printing ADC routine timing via UART
* `print adc` or `adc print` - **[default: OFF]** enables/disables printing ADC measurement values via UART every 2 seconds
* `print meas` or `meas print` - **[default: OFF]** enables/disables printing physical measurement values (unfiltered) via UART every 2 seconds
* `print input` or `input print` - **[default: OFF]** enables/disables printing input signals (filtered or unfiltered input to control loop) via UART every 2 seconds
* `plots` - **[default: OFF]** enables/disables sending ADC measurement values in `$[data];` format via UART, so that they can be displayed with Serial Port Plotter. Useful for monitoring e.g. PVA voltage and current over time.
* `plots [pv/pv lp/bat2/bat2 lp]` - **[default: OFF]** enables/disables sending ADC measurement values in `$[data];` format via UART; `pv` - V,I,P of PV bus,
  `pv lp` - low-pass filtered V,I,P of PV bus, `bat2` - V,I of BAT2, `bat2 lp` - low-pass filtered V,I of BAT2
* `noise` - **[default: OFF]** enables/disables sending ADC measurement values (I_PV in µA) in `$[data];` format via UART;
* `print stddev` or `stddev print` - **[default: OFF]** enables/disables printing standard deviation of a ADC channel for current measurement
* `adc zero i` or `zero i` - **[default: OFF]** enables/disables zero current voltage measurement. Takes about 5 min to get reasonable value (using software low-pass filtering).
            This command is used to measure the voltage that is output by the [ACS723-05AB](https://www.allegromicro.com/en/Products/Sense/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS723)
            current sensors for **zero current**.
            When using this command, ensure that no load is connected to the PCB and no current is flowing anywhere.
            The outcome of this measurement can be used to adjust the 'calibration' in software through hard-coding the offset.
            Be aware that _I_PDU5V_ and _I_PDU3.3V_ are measured through [LTC3112](https://www.analog.com/media/en/technical-documentation/data-sheets/3112fd.pdf),
            so the zero current measurement does not apply here.
* `cpu load` - **[default: OFF]** enables/disables displaying CPU load. Handle with caution.  
   Actually, this command does not print the **cpu load** but the **period load**, i.e. which portion of the SAMPLING_INTERVAL is occupied by processing (by ADC, ...).  
   ADC the highest portion on the cpu load, as it takes about 1.2 ms to sample 11 channels 25 times + averaging (@ f_cpu = 24 MHz).  
If enabled, a estimation of the CPU load is periodically sent via UART. However, in order to sent this estimate properly via UART, the CPU load should be below 10 %, as interrupt routines of UART is not included in CPU load. Use your brain when using this command. Is the displayed CPU load estimate reasonable?

 
## So what are those UART commands good for?
* Set volatile configurations (the MCU awakes to default configuration after CPU reset)
* Query parameters from the MCU
* Activate and deactivate modules on the prototype PCB


## Who can I talk to?
* [Adrian Wenzel](mailto:a.wenzel@tum.de) 
* [Juan J. Rojas](mailto:juan.rojas@itcr.ac.cr)

 