# lightThermo


Simple Temperature and Humidity sensor with 1.3" OLED display.  
Based on Raspberry PICO board (with RP2040 microprocessor).  

## Hardware configuration

There are three main devices:
- the Raspberry PICO board
- the 1.3" OLED display with SH1106 driver and I2C interface  
  The display resolution is 128 x 64 pixel
- the temperature and humidity sensor DHT20 with I2C interface

## Connections

The display and the sensor are connected to the PICO board via  
the I2C interface:
- the display connections are:
    - *VDD* to PIN 36 of PICO (3V3)
    - *GND* to PIN 38 of PICO (GND)
    - *SCK* to PIN 12 of PICO (GP9 = I2C0 SCL) 
    - *SDA* to PIN 11 of PICO (GP8 = I2C0 SDA)
- the DHT20 sensor connections are:
    - *+* to PIN 36 of PICO (3V3)
    - *-* to PIN 38 of PICO (GND)
    - *SCL* to PIN 20 of PICO (GP15 = SCL)
    - *SDA* to PIN 19 of PICO (GP14 = SDA)
Obviously, all connections can be changed.
 
