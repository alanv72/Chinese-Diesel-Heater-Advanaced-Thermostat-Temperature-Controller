Added some error handling and frontend alerts. Disabled control when coms are down.

Added rename fuciontality. Set default name to heater-therm.

Some fixes to saving and loading historical data from spiffs. Updated layout of some buttons on the webinterface.

![image](https://github.com/user-attachments/assets/28f88f78-811c-4963-ba36-8c46f3d24f07)


Updated Frost mode.

![image](https://github.com/user-attachments/assets/19fe8017-6a75-46a7-86a0-744ed1ae7b08)

added some Totals based on 24hr chart data. Frontend only.

![image](https://github.com/user-attachments/assets/8da05b24-8115-452d-8f36-fff700f42d15)


Fixed accumulators load on boot. Changed backend to all use UTC (no offset) with epochtime and updated frontend to apply offset based on client TZ for Date/Time. The chart data is displayed with realtive times thru serialize functions so no updates needed. Other minor fixes to html and js. Amps are calculated based on measure current from each fan, heater controller, glowplug, and heater fan. 

![image](https://github.com/user-attachments/assets/d5e68c90-9f49-4518-9fbf-b4ddad9fa451)
![image](https://github.com/user-attachments/assets/f348b02c-d7fc-4913-bda0-565018380bb6)
![image](https://github.com/user-attachments/assets/4951d689-65f7-4a4a-b5e5-3b1e316f39c8)

added history charts with saves to json files on spiff and reload on boot. Fans are now controlled with PWM over IRLZ44 mosfet instead of simple relay. Still use the 3v to 5v shifter to ensure mosfet is completely on. shifted the software serial pin so that it doesn't interfere with Bluetooth in the future. Added outdoor temp to chart. add file management with delete function. TODO: add mqtt to HA for reporting and controls. add BLE fuctionality to interfade directly without wifi.

![image](https://github.com/user-attachments/assets/70a87fe1-5200-47be-acca-2c5edf089551)
![image](https://github.com/user-attachments/assets/583fb211-5f2f-4d22-98c7-40c3ac62dc2c)
![image](https://github.com/user-attachments/assets/432b674f-6094-4a57-b9da-d1afd8ae339a)
![image](https://github.com/user-attachments/assets/414c9f24-d4ce-4b0e-ae7c-99cb6a9d24e0)

Upload js and ico files along with the index_html to SPIFF with /fallback

Added voltage history graph.

Updated error message handling on the frontend. Note: the index_html needs to be loaded to SPIFFS. You can do that thru the /fallback file upload at the bottom.
![image](https://github.com/user-attachments/assets/0b64eb28-4ce3-459e-9299-7ece548c891a)


Corrected reading Error codes and added logic to detect other shutdowns outside of ESP32 control. Changed the Wall control logic to be delta between cabin air and inside the wall that encloses the ducting.

Lot's of new undocumented features ;)

added a ext temp sensor (DS18B20) to run some duct fans that are triggered by 5v logic relays. Fixed calcs for fuel and testing a few avg methods. Add fuel gauge and a temp history chart. Use the file upload to saving frontend index_html changes. Fallback html incase I break frontend with JS updates. Added a url to reset stats that are persistent in prefs. I have pins soldered to up/down buttons to mimic button push since I can't get the controller to accept temp changes thru the data protocol.
![image](https://github.com/user-attachments/assets/4ebed93b-0f3e-45f1-ab5c-d9bbae2841d9)
![image](https://github.com/user-attachments/assets/4b475c2a-d423-4451-91b8-054ff41593bb)

New updates and added functonality to mimic afterburner. Will document when I have time.
![image](https://github.com/user-attachments/assets/2f727591-7cb6-40a4-b427-13b11fb2900d)


This is a fork/port for ESP32 devkit 1. Currently MVP is working. Will be adding things like OTA, web interface, and bluetooth for monitoring. Will be adding status collection and fuel gauge from 0-90ohm 20gal diesel tank.

You will need a 3.3v to 5v level shifter!
i.e. https://www.amazon.com/dp/B08C9PFVGP
![image](https://github.com/user-attachments/assets/54cb48d1-d689-4f43-a3b4-d8f57d3fb5c1)

Including sniffer from the great https://gitlab.com/mrjones.id.au/bluetoothheater

Original intent from Jess
--------------------------
# Chinese Diesel Heater - Advanced Thermostat Temperature Controller
A simple Arduino inline extension to Chinese Diesel Heaters to help maintain a stable temperature by gracefully shutting down the heater if it gets too hot, or turning it on if it gets too cold.

Keeping the existing controller/display, it does this by turning off the heater if the observed tempertaure goes beyond that set on the thermostat control. It will also automatically restart the heater if the temperature dips too cold. A temperature range is set to avoid excess power cyling.

Originally written by @Jess-- and recently refactored, improved and tested.

Use at your own risk...
## The Problem

Ideally the standard heater behavour will maintain its own temperature, by reducing fuel and burn speed. Once the heater reaches the thermostat temperature it starts to slow down. However, the heater does not fully turn off. So if the heater is used in a small space, the minimum heat input can often be greater than that being lost. Then it gets hotter and hotter!
## Hardware

Keep the standard controller but splice into the 5V, GND and Blue control/data wire near the exising control/display unit.

The 5V and GND connect to the Arduino VCC and GND pins. The blue data wire connects to the D2 pin. (check input voltage to the controller is definitely 5V - incase it is a different model)

Wiring:

D2 -> Bus blue

5V -> + 5V red

GND -> GND black

D8 -> GND optional inhibit switch

Confirmed working with the Arduino Nano 5V 16Mhz, but should work with most 5v ATMega chips.

I have only tested it with the model I have but I suspect it will work with a range of heaters. The world famous "Afterburner" diesel heater controller by Ray Jones suggest this compatability list https://gitlab.com/mrjones.id.au/bluetoothheater/-/wikis/home#compatibility But thats only a guess, so use at your own risk!

You can add an optional switch on to D8 Pin - shorting this pin to GND turns disables the new functionaility.
## Behaviour 

Once the heater has been powered on into standby mode the addon will activate after 30 seconds. During this time, set the thermostat to the desired temp. The module monitors temperature (desired temperature and actual temperature) data from the controller to the heater.

If it is already warm enough, nothing will happen.

If the temperature is or falls 2 degrees below the desired temperature the module will issue a start command to the heater. This start command triggers the same startup as you pressing the 'On' button. So all normal startup procedures are followed by the heater.

If the actual temperature rises 2 degrees above the desired temperature and the heater has reached its normal running state the module will send a stop command to the heater. The command sent triggers the normal shutdown on the heater as if you had pressed the off button on the controller so all of the normal cooldown procedures are followed.

The reason for 2 degrees above being chosen as the cutoff is to allow the heater to try and maintain temperature on its own by going to it's low setting (which it does at 1 degree above) and only shut it off if the amount of heat being produced on low is still raising the temperature.

It will continue to turn on/off as required.

If you turn the heater off using the controller or remote. It will disable the thermostat behaviour and will not resume until the 'on' button is pressed again, or the unit is completely powered off/on again.


## Change Log

Changes since forked:
- Fixed Negative Temperature Issue
- Real-world tested for 2 years without any issues (other than the negative temperature issue - now fixed!)
- Refactored
- Added 30 seconds delay before sending on/off commands.
- Added extra temperature buffer to avoid cycling the heater excessively
- Fixed potential array out of bounds issue
- Commented out debug information for extra stability
- Check Temperatures are reading correctly before starting the heater
- Added LED heartbeat (normal speed for no changes, slow for low temp - turned on, fast for high temp - turned off)
- Changed behaviour to default thermostat mode on at start.
