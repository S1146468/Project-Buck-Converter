![alt text](https://github.com/S1146468/Project-Buck-Converter/blob/master/Product_Front_view.jpg "")

# Switch Node Ringing Phenomenon

This project is about the switch node ringing phenomenon. We engineered an simulationbox that simulates the ringing phenomenon after the button is pressed. 
To use this project you will need the following equipment:
* The simulation box
* Oscilloscope
* Lab-bench power supply
* 5V USB power supply (only if you don't have a double channel lab-bench power supply)
* The nesserary cables for the test:
    * Oscilloscope probes
    * Power supply cables
    * USB-A to USB-micro cable in case you don't have a double channel lab-bench power supply


## User guide

![alt text](https://github.com/S1146468/Project-Buck-Converter/blob/master/Product_Top_view.png "")

### Pinout description
| Pinout            |               |
| ----------------- | ------------- | 
| GND	            | Power supply ground reference for the oscilloscope    |
| USB-Power	        | If the switch is in this position the 5V will be generated through the Nucleo 5V supply     |
| External 5V	    | If the switch is in this position the Nucleo board will only receive external power through the J2 header     |
| J1                | External 10V input    |
| J2                | External 5V input (make sure to set the switch to external 5V)     |
| J3                | External GND from PSU     |
| J4                | External trigger for the oscilloscope     |
| J5                | Clock out / input to mosfet driver for Q4     |
| J6                | Output / Measure here with the oscilloscope     |
| J7                | Trigger signal to mosfet driver to trigger the output oscillations     |     
| J8                | Q4 gate signal     |
| J9                | Q8 gate signal     |
| Potmeter          | Use this to change the amplitude of the oscillations     |
| Button            | This push button is used to trigger the oscillations     |


### Power supply
The output of the power supply should be set to around 10V DC. If the results you get are not what you expected, you can vary the output voltage of the power supply by +-2V.

If you have a power supply with a second channel, you can use that to power the Nucleo board. Make sure to set the switch to External 5V if you do, if you don't, you risk destroying the Nucleo board.
In case if you don't have a second channel on the power supply, you can also use the micro-usb port on the Nucleo to power it. Make sure to set the switch to USB-Power before you connect it. If you don't you risk destroying the Nucleo board.
