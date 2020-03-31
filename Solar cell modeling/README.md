# Solar cell model (Matlab/Simulink)

* This repository contains a MATLAB/Simulink model of a solar cell.  
* Doing the simulation will plot the I-V characteristics, herefore the script `plot_IV_curve.m` is necessary.  
* Most blocks used in the Simulink model are standard Simulink blocks, with exception of:
* Electrical component blocks are from Library *Simscape* -> *Power Systems* -> *Specialized Technology* -> *Fundamental Blocks*
* **OBSOLETE:** `SimscapeFoundation_singleDiodeModel_solarCell.slx` - **This model does not work when put in series to form an array!**
* **Instead, use** `PowerSystems_singleDiodeModel_solarCell.slx` to build up a solar array, as done in the example `PowerSystems_singleDiodeModel_solarArray.slx`.
    * Further, the _Power Systems_ elements are also used for the simulation of the BDC, so these models can be combined.
    * `SimscapeFoundation_singleDiodeModel_solarCell.slx` in opposition uses *Simscape* -> *Foundation Library*, which is not compatible to the BDC model.