# pca9685test9 Steve Lomax 2025. Not for commercial use

# Arduino 32 Point Control system
## PART 9  
this code considers

* sending servo position values to 2 PCA9685 modules.
* receiving push button operations from two PCF8575 modules
* sending Neopixel data to 32 LEDs for mimic display
* Sending calibrationand mimic status to a 20 x 4 LCD display 
* on the fly point setting adjustment
* point movement speed setting
* point pairing amd grouping


The display uses I2C check listing for library link

Always ensure 5 vots supply is established before sending a control voltage to the data pin!

Thw manual 4 button code remains


