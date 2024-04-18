![alt text](https://github.com/S1146468/Project-Buck-Converter/blob/master/Product_Front_view.jpg "")

# Switch Node Ringing Phenomenon

This project is about the switch node ringing phenomenom. We engineered an simulation box that simulates the ringing phenomenom after the button is pressed. 
To use this project you will need the following equipment:
* The simulation box
* Oscilloscope
* Lab-bench power supply
* 5V USB power supply (only if you don't have a double channel Lab-bench power supply)
* The nesserary cables for the test:
    * Oscilloscope probes
    * Power supply cables
    * USB-A to USB-micro calbe in case you don't have a double channel Lab-bench power supply


## User guide

![alt text](https://github.com/S1146468/Project-Buck-Converter/blob/master/Product_Top_view.png "")

The output of the powersupply should be set to around 10V DC. If the results you get are not what you expected, you can vary the output voltage of the power supply by +-2V.

If you have a power supply with a second channel, you can use that to power the Nucleo board. Make sure to set the switch to External 5V if you do, if you dont, you risk destroying the Nucleo board.
In case if you don't have a second channel on the power supply, you can also use the micro-usb port on the Nucleo to power it. Make sure to set the switch to USB-Power before you connect it. If you don't you risk destroying the Nucleo board.
