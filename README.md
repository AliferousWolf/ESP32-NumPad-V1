Included are all the files for the PCB and parts of the numpad v1 (Arduino IDE and KiCad)
------------------------
This is technically a prototype. The code is from various libraries and not very optomized, but it is a place to start.
To flash the ESP32 with the code, I used ArduinoIDE with the ESP32 boards added. When flashing, don't forget to download the libraries used within the code.

Parts used (part number):
------------------------
ESP 32 (NodeMCU-32S)
2x Encoder (PEC11L-4115F-S0020)
22x SMD diode (1N4148WS)
TFT display (Adafruit 4383)
mini sd card for tft display (technically optional)
24x mx switches depending on configuration
2x stabilizors depending on configuration


Optional for batery/bluetooth (part number)
------------------------
Switch (EG2208)
2x Capacitor 1uF (VJ1206Y105JXQTW1BC)
3.3v regulator (TC1264-3.3VDB)
10k resistor
6.8k resistor
3.3k resistor
battery
