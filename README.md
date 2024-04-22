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

Pinout:
* GND	            Power supply ground reference
* USB-Power	        If the switch is in this position the 5V will be generated through the Nucleo 5V supply
* External 5V	    if the switch is in this posistion the Nucleo board will only recieve external power 
                    through the J2 header
* J1              
* J2              
* J3              
* J4              
* J5              
* J6              
* J7              
* J8              
* J9              
* Potmeter        
* Button          



The output of the powersupply should be set to around 10V DC. If the results you get are not what you expected, you can vary the output voltage of the power supply by +-2V.

If you have a power supply with a second channel, you can use that to power the Nucleo board. Make sure to set the switch to External 5V if you do, if you dont, you risk destroying the Nucleo board.
In case if you don't have a second channel on the power supply, you can also use the micro-usb port on the Nucleo to power it. Make sure to set the switch to USB-Power before you connect it. If you don't you risk destroying the Nucleo board.




Tag	Functie
GND	Dit is de nullijn van het gehele circuit, deze headers zijn geplaatst voor de ground van scopes en multimeters.
USB-Power	Als de switch in de bovenste stand staat is het circuit gevoed door de external power supply en de microcontroller door de USB.
External 5V	Als de switch in de onderste positie staat is de microcontroller gevoed door de externe 5v die van de power supply komt.
J1	External 10V input, hierop wordt de labvoeding aangesloten.
J2	External 5V input, hierop wordt de labvoeding aangesloten.
J3	External GND, hierop wordt de GND van de labvoeding aangesloten.
J4	Deze pin is om de scope een trigger moment te geven, deze pin is aangesloten op de microcontroller en de 
J5	Ingangssignaal van de MOSFET-driver die de buck converter stuurt.
J6	Uitgangssignaal, op deze pin kan het uitgangssignaal van het circuit worden gemeten.
J7	Ingangssignaal van de MOSFET-driver die het circuit belast.
J8	Uitgangssignaal van de MOSFET-driver die de buck converter stuurt.
J9	Uitgangssignaal van de MOSFET-driver die het circuit belast.
Potmeter	Hiermee kan de duty cycle die naar de MOSFET gaat worden aangepast, helemaal naar links is 10% duty cycle en helemaal naar rechts is 100% duty cycle.
Button	Hiermee kan de trigger die de microcontroller uitstuurt worden ingesteld.
