# Vu_Meter_Mega2560
## Arduino Mega2560 ws2812 reactive visualizer 


This project provides mutliple set of visualizations for your viewing enjoyment.  you will need a series of RGB based LEDs.  I used a series of WS2812 neopixels (60), however you may use others if you like and ensure you update the LED_TYPE to accomodate the the setup.

To get this up and running:
* Use a Mega2560,  the memory requirments exceed anything the UNO can provide.
* NeoPixels
* SSD 1306 display 128x32 i2C
* Electret microphone
* Power plug
* 100 µF capacitor
* 470 Ω resitor
* momentary pushbutton
* 10 kΩ potentiometer


A few other general hardware items,
* Libraries:
* adafruit nepixels
* adafruit_ssd1306
* fix_fft

Download trhe repot and compile the module place on your Arduino and let er rip.
Below you will find, images of the schematics, wiring diagram and the Shield image in the repo also providing the PCB files,  

I use *`Osh Park`* . https://oshpark.com/ . They are reasonable and they are cool, and not too mention rather than the standard green or the relatively new red pcb,  these guys do things in Purple and looks great.

- ### NOTE:  The compressed gerber files for upload are located in file # TODO:  **`Vu_Meter_Mega2560_shield.zip`**

## Schematic Diagram
![Schematic Diagram](https://github.com/AGHG46087/Vu_Meter_Mega2560/blob/master/vu_schematic.jpg "Schematic Diagram")


## Wiring Diagram
![Wiring Diagram](https://github.com/AGHG46087/Vu_Meter_Mega2560/blob/master/vu_wiring.jpg "Wiring Diagram")

## Shield Diagram
![Shield Diagram](https://github.com/AGHG46087/Vu_Meter_Mega2560/blob/master/vu_shield.jpg "Shield Diagram")


 
