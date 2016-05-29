/************************************************************************************
* 	
* 	Name     : Thermostat.h
* 	Author   : Joaquim Barrera
* 	Date     : 2016
* 	Version  : 0.1
*	Platform : Arduino Nano 328 5V (CH340 driver)
* 	Notes    : 
* 
***********************************************************************************/

#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#define TurnHeatON()	{isHeatOn=true; digitalWrite(RELE_PIN, isHeatOn);}
#define TurnHeatOFF()	{isHeatOn=false; digitalWrite(RELE_PIN, isHeatOn);}

// Custom Arduino libaries
#include "dht.h"			// https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTlib
#include "Sleep_n0m1.h"		// https://github.com/n0m1/Sleep_n0m1
#include "RTClib.h"			// https://github.com/adafruit/RTClib
// #include "IRremote.h"		// https://github.com/z3t0/Arduino-IRremote

// Pinout
// #define PIN0  0		// Rx
// #define PIN1  1		// Tx
#define PIN_BTN_MORE  2 		// INT0
#define DHT22_PIN 3 	// INT1
#define SD_CS	  4
#define DHT22_VCC 5
#define PIN_BTN_LESS  6 		// 
#define RTC_VCC	  7
#define RELE_PIN  8
#define INFO_LED  9
// #define PIN10 10

#define SD_VCC	A0

#define	CTRL_B0		0x70
#define	CTRL_B1		0x07

// Initial Thermostat settings
#define	DEFAULT_OBJECTIVE	200


// App defines
#define BTN_NONE	0
#define BTN_MORE	1
#define BTN_LESS	2
#define TIMEOUT_TO_HEAT		(unsigned int)5000 // milliseconds
#define TIMEOUT_TO_SLEEP	(unsigned int)10000 // milliseconds

#define SLEEP_TIME_MS		(unsigned long)10000

struct ThermoSettings {
	unsigned int temp_objective;

	void toSerial()
	{
		char buff[32];

		Serial.println(F("Thermostat Settings:"));
		sprintf(buff, "\tObjective: %d\n", temp_objective);
		Serial.println(buff);
	}

} _thermoSettings;

unsigned int _lastButtonPressed = BTN_NONE;
bool _wasButtonPressed = false;

unsigned long _timerToSleep = 0;
unsigned long _timerPeriod100  = 0;
unsigned long _timerPeriod1000 = 0;

static bool _isHumanInteraction = false;

unsigned int _tempActual = 0;

bool t=false;

bool isSDInit = false;
bool isRTCInit = false;

bool isTempHumOK = false;
bool isHeatOn;

#endif //THERMOSTAT_H
