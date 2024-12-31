# lightThermo


Simple Temperature and Humidity sensor with 1.3" OLED display.
Based on Raspberry PICO board (with RP2040 microprocessor).  
I know, there are a lot of these devices build on Raspberry PICO
therefore I've added a serial log on UART port (text or CSV format)
in order to use it as a data logger too and collection of statistic
that can be showed on the display using a push button.  

## Hardware configuration

There are three main devices:
- the Raspberry PICO board
- the 1.3" OLED display with SH1106 driver and I2C interface  
  The display resolution is 128 x 64 pixel
- the temperature and humidity sensor DHT20 with I2C interface

and a push button in order to change the display view between
three different views:
- real time measure of temperature and humidity
- last 24 hours min, max and average value of temperature
- last 24 hours min, max and average value of humidity

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
    - *SCL* to PIN 25 of PICO (GP19 = I2C1 SCL)
    - *SDA* to PIN 24 of PICO (GP18 = I2C1 SDA)  
    
Obviously, all connections can be changed and only one I2C channel
can be used.

The UART used to send the measures is the default (it can be changed in
lightThermo.h file) so it's enough to connect the PIN 1 and 2 to an
UART-to-USB converter to see the ouput on a terminal console on the PC.

Be careful to use a 3.3V converter!


I use AZ-Delivery FT232RL with putty on Window 11 PC and it works.

## How it shows the temperature and humidity

In the image below an example of how the real-time measurement is
displayed on the OLED:
![lightThermo in action](/resorces/lt_realtime.jpg "the realtime view")

Here how the statistics are displayed (press to the button to change view mode)
![lightThermo in action](/resorces/show_temp_stats.jpg "the statistic view")

Here the output of the UART interface (in this case in CSV format
*Temperature;Humidity*)
![lightThermo in action](/resorces/serial_output.jpg "the CSV format out log")

## Battery powered prototype

In the image below my prototype with 3 AAA alcanine battery power supply:
![lightThermo prototype](/resorces/lt_batt_prototype.jpg "a battery powered prototype")


## Next development

I'm working on porting this project to Pico W to have a wireless
version of the sensor. I also want to add a little eeprom where 
store some configuration parameters (sush as °C or °F) that can
be changed via web interface.
