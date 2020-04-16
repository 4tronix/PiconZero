Picon Zero
=======
Picon Zero is a Raspberry Pi addon in the form factor of a Pi Zero. It contains an ATMega328 with the Arduino bootloader. The ATMega328 is connected to the Raspberry Pi via I2C and handles all the interface to the I/O ports and motor drivers.
Specifically, the ATMega328 allows:
 - Dual H-Bridge motor driver (DRV8833)
 - Standard digital outputs (On / Off)
 - PWM outputs (0 to 100%)
 - Servo outputs (-90 to +90 degrees)
 - WS2812B compatible smart pixels (up to 100 pixels)
 - Standard digital inputs (On / Off)
 - Analog inputs (0 to 255)

Picon Zero Released Files
======================
All files are Licensed under Creative Commons BY-SA
See creativecommons.org/licenses/by-sa/3.0/ for details
 - piconzero.py (this is the main Python library)
 - various python example files
 - PiconZero10.ino (this is the ATMega code)

Programming the ATMega
======================
You can reprogram the ATMega with your own code by using a USB to Serial converter that has the following pinout:
 - DTR
 - RXD
 - TXD
 - VCC
 - N/A (usually CTS, but not used)
 - GND
There are many sources of this converter including the 4tronix shop: https://shop.4tronix.co.uk/products/usb340g
