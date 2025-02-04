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
