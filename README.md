# Wifi-Telemetry-with-Pixhawk-Arducopter
This Repo contains the instructions to use Wi-Fi Telemetry with Pixhawk or Arducopter on board

The ESP8266 is a low-cost Wi-Fi microchip, with built-in TCP/IP networking software, 
and microcontroller capability, produced by Espressif Systems in Shanghai, China.

You can buy any one of the modules from the below list
1. https://www.sparkfun.com/products/17146

We have used this module - 

![WhatsApp Image 2022-05-24 at 11 23 48 AM](https://user-images.githubusercontent.com/49754981/169959909-042154c5-aa43-4dce-856d-37afd680eef5.jpeg)

2. https://www.gearbest.com/boards-shields/pp_009604906563.html
3. https://www.banggood.in/Wireless-Wifi-to-Uart-Telemetry-Module-With-Antenna-for-Mini-APM-Flight-Controller-p-1065339.html?cur_warehouse=CN
4. https://www.banggood.in/MAVLink-Wifi-Bridge-2_4G-Wireless-Wifi-Telemetry-Module-with-Antenna-for-Pixhawk-APM-Flight-Controller-p-1428590.html?cur_warehouse=CN

A NodeMCU is a low-cost open source IoT platform. It initially included firmware which runs on the ESP8266 Wi-Fi SoC from Espressif Systems,
and hardware which was based on the ESP-12 module. 

A NodeMCU can be used for the same purpose and also can be used as an on board computer.

You can buy this module from robu.in - ESP8266 NodeMCU CP2102 Board [Make Sure the module has a CP2102 / FTDI Chip for USB Flashing]

You can flash the ESP8266 with "firmware_esp_arducopter" or "firmware_esp_px4" according to what firmware you are working on.

Flashing Instructions : 
1. Connect the ESP8266 in flash mode

![image](https://user-images.githubusercontent.com/49754981/169957873-f612c8f3-366b-49e6-a77d-11417366c106.png)

2. Install the NodeMCU Flasher (Windows System Only) from the following link - 

https://github.com/nodemcu/nodemcu-flasher/raw/master/Win64/Release/ESP8266Flasher.exe

3. Connect the module to CP2102 USB 2.0 to TTL UART Serial convertor Module. Wire it up in the following manner - 

Our Module looks like this -


![WhatsApp Image 2022-05-24 at 11 23 48 AM (1)](https://user-images.githubusercontent.com/49754981/169959952-cfdc4ce9-9a6e-4afb-afee-04782899fd5c.jpeg)

ESP8266           Serial Converter Module

RX                TXD
VCC               5V
GPIO 0            GND
RESET             -
CH_PD             5V
GPIO 2            -
TX                RXD
GND               GND

Conenct the Serial Converter Module to your PC and open the NodeMCU Flasher Software and follow these steps - 

1. Note down the COM Port (Available in the Device Manager)
2. Select the COM Port on the Software
3. Go to Advanced and Set the Baud Rate to 115200, Flash Size 4MByte, Flash Speed 26.7MHz, and SPI Mode DIO
4. Go to Config and select the binary file to flash
5. Come back to Operation and Flash

After Flashing, remove connections from GPIO 0 and keep the remaining connections. 

For PX4 - If you want to customize the firmware - 

Build From Sources
The firmware repository contains instructions and all the tools needed for building and flashing the 
ESP8266 firmware.

https://github.com/dogmaphobic/mavesp8266


Pixhawk/Arducopter Setup - 


You can flash your flight controller with either Ardupilot and PX4. Make sure to set up MAVLink2 on both firmwares by
following these steps : 

For Ardupilot,
Use one of the Telemetry Ports on the hardware

1. Set the following parameters for that particular Telemetry Port - 

  SERIAL2_PROTOCOL = 2 (the default) to enable MAVLink 2 on the serial port.
  SERIAL2_BAUD = 921 so the flight controller can communicate with the RPi at 921600 baud.
  LOG_BACKEND_TYPE = 3 if you are using APSync to stream the dataflash log files to the RPi

2. Make sure you have flashed "firmware_esp_arducopter" into the ESP8266 before connecting. 
3. Connections - 

ESP8266          Autopilot

VCC              5V
TX               RX (Of Telemetry chosen)
RX               TX (Of Telemetry chosen)
GND              GND
CH_PD            5V

For PX4
Use one of the Telemetry Ports on the hardware

1. Set the following parameters for that particular Telemetry Port (We Used TELEM2) - 
   ![image](https://user-images.githubusercontent.com/49754981/169961090-20526be3-12c3-413c-89cb-74cbd8c2dbd0.png)
    
    Set SER_TELEM2_BAUD to 921600
    
2. Make sure you have flashed "firmware_esp_px4" into the ESP8266 before connecting. 
3. Connections - 
    
ESP8266          Autopilot

VCC              5V
TX               RX (Of Telemetry chosen)
RX               TX (Of Telemetry chosen)
GND              GND
CH_PD            5V
   
Now we are set to use Wifi Telemetry on Board!

The ESP8266 emits a hotspot for a range of around 300m. The name of the network will most probably be "PixRacer" with password "pixracer"
Connect to this network (acts as access point) on the ground station and open QGroudControl (Have not tested for Mission Planner)

You should be able to see the following - 

You should be able to download parameters and ready to go!



