# I2C LCD
First attempts at communicating over I2C using the STM32-F446RE board. Basic function will be to print data to the screen (source currently unknown). This project is simply for me to test out I2C communication to understand the innerworkings.

# Hardware
- STM32-F446RE
- 16x02 LCD
- Jumper Wires

# Features
- I2C Communication - Implemented
- Printing characters and strings to LCD screen via I2C - Implemented
- Menu driven screen using potentiometer values and button presses - NEXT
- UART Debugged - Future
- Terminal Commands to control output - Future

# Pins
- 5V to VDD
- GND to GND
- PB8(D15) to SCL
- PB9(D14) to SDA

![LCD_Screen](/home/jmac/Downloads/LCD_Screen.png)
