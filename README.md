# ProtoCentral MAX30001 ECG and Bio-Impedance Breakout Board
[![Compile Examples](https://github.com/Protocentral/protocentral_max30001_arduino_library/workflows/Compile%20Examples/badge.svg)](https://github.com/Protocentral/protocentral_max30001_arduino_library/actions?workflow=Compile+Examples)

![ProtoCentral MAX30001 Single-channel ECG breakout](assets/max30001_brk.jpg)

If you dont already have one, you can buy [ProtoCentral MAX30001 breakout here.](https://protocentral.com/product/protocentral-max30001/)


MAX30001 is a single-lead ECG monitoring IC which has built-in R-R detection and several other features that make it perfect for a wearable single-lead ECG application.  

Several new features on this chip make it ideal for wearable applications. First is the low power consumption - just 85 uW of power and can work from 1.1 V onwards ! Also of interest is the fact that it can work with only two chest electrodes without the need for a third right-leg drive (DRL) electrode.

The best feature of this chip though is the built-in R-R detection algorithm which can measure the time between successive peaks of the QRS complex of the ECG. This means that heart-computation comes right out of the box without any microcontroller-side code requirement. Heart-rate computation just got a lot easier !!

# Features

* MAX30001 IC on-board
* Single-lead ECG monitoring
* R-R peak detection for heart rate computation
* High DC Offset range
* Heart Rate computation using Pan-Tompkins algorithm
* On-board level translator for 5V-tolerant operation
* On-board low-noise 1.8V and 3.3V voltage regulator

# Repository Contents

* This repository contains only the Arduino library software
* Hardware information (schematics, datasheets etc.) is at https://github.com/Protocentral/protocentral_max30001

# Wiring the board to your Arduino

If you have bought the breakout the connection with the Arduino board is as follows:

|MAX30001 pin label| Arduino Connection   |Pin Function      |
|----------------- |:--------------------:|-----------------:|
| MISO             | D12                  |  Slave out|             
| MOSI             | D11                  |  Slave in           |
| SCK              | D13                  |  Serial clock     |
| CS0              | D7                   |  Slave select|
| FCLK             | NC                   |  External clock(32KHz)     |
| INT1             | D2                   |  Interrupt        |
| INT2             | NC                   |  Interrupt       |
| 3V3              | Supply               |  Board which supports 3.3V and 1.8V    |
| VCC              | Supply 5V            | 5V            |
| GND              | Gnd  


# Running the Arduino Sketch

If you have correctly installed the libraries, the example sketeches should now be available from within Arduino.

Open up your Arduino IDE and run the Arudino sketch (.ino) file in the archive that you downloaded. Your Arduino should now be programmed to read the ecg data and sending over the USB-UART.  

# Using the ProtoCentral OpenView GUI

The GUI for visualizing the ECG and Respiration as well as parameters like Heart rate and Respiration rate is written in Processing, based on Java and is cross-compilable across platforms.

![Wearing the Electrode](assets/gif-max30001-openview.gif)

Java 8 is required on all platforms for running the processing-based GUI application. You can download Java for your platform from the [Official Java website](https://www.java.com/en/download/).

You can download and install [ProtoCentral OpenView from here](https://github.com/Protocentral/protocentral_openview).

Once you have opened OpenView, make sure to select "MAX30001 breakout" under the "Select Board" dropdown. 

# Connecting the ECG Electrodes

A 2-electrode cable along with a standard stereo jack is provided along with the shield to connect the electrodes to the board. The electrode input connector is highlighted in the below picture.

The other side of the electrode connector would connect to snap-on electrodes attached to the body. For testing purposes, you can use an ECG simulator to provide inputs to the board.

*Warning:
When connecting the electodes to the body, it is safer to disconnect the mains power source to the Arduino. For example, if  you are using the Arduino along with a laptop, disconnecting the battery charger from the laptop would be a safe option.*

# Placing the Electrodes on the body

![Wearing the Electrode](assets/body.png)


License Information
===================

![License](license_mark.svg)

This product is open source! Both, our hardware and software are open source and licensed under the following licenses:

Hardware
---------

**All hardware is released under the [CERN-OHL-P v2](https://ohwr.org/cern_ohl_p_v2.txt)** license.

Copyright CERN 2020.

This source describes Open Hardware and is licensed under the CERN-OHL-P v2.

You may redistribute and modify this documentation and make products
using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).
This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED
WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY
AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2
for applicable conditions

Software
--------

**All software is released under the MIT License(http://opensource.org/licenses/MIT).**

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Documentation
-------------
**All documentation is released under [Creative Commons Share-alike 4.0 International](http://creativecommons.org/licenses/by-sa/4.0/).**
![CC-BY-SA-4.0](https://i.creativecommons.org/l/by-sa/4.0/88x31.png)

You are free to:

* Share — copy and redistribute the material in any medium or format
* Adapt — remix, transform, and build upon the material for any purpose, even commercially.
The licensor cannot revoke these freedoms as long as you follow the license terms.

Under the following terms:

* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.

Please check [*LICENSE.md*](LICENSE.md) for detailed license descriptions.