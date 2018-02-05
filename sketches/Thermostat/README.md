# Low Power Thermostat
Here you will find the schematics and the code of a low power Thermostat, based on a [Moteino](https://lowpowerlab.com/guide/moteino/) board. It has a build-in radio to transmit information to a WiFi-enabled base station and, therefore, push data the cloud.

The thermostat itself works just as a switch to the heater control line, so no AC supply is needed.

Key features:
- Low power (stand by current of 24uA). No AC supply needed. Can run for months with 3xAAA.
- OLED screen as a HID.
- 3 modes of operation:
	- Turn the heater ON for a period of time.
	- After a period of time, turn the heater ON for another period of time.
	- Temperature set point with ~~a configurable~~ 1ºC hysteresis. 
- IoT enabled through another Arduino based device connected to the Internet.

TODO:
- Connect the Thermostat directly to the Internet, keeping the low power profile.
- Control the temperature set point from the cell phone.
- Add an RTC in order to program the Thermostat based on week days, day time, etc.
