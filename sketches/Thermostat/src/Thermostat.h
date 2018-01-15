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

#include "Arduino.h"

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<PIN_COUNT; p++) {        \
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        } while(0)  // This while is to allow ';' at the end of the macro


// I/O Pins
enum {
    D0,
    D1,
    D2_RESERVED_RADIO,
    BUTTON_IN,
    RELAY_TRIGGER,
    D5,
    DHT_PIN,
    OLED_VCC,
    FLASH_SS,
    INFO_LED,
    D10_RESERVED_RADIO,
    D11_RESERVED_RADIO,
    D12_RESERVED_RADIO,
    D13_RESERVED_RADIO,
    PIN_COUNT
};	/* Moteino */

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#define USE_DEBUG

#if defined(USE_DEBUG)
    #define DEBUG(str)   Serial.print(str)
    #define DEBUGLN(str) Serial.println(str)
#else
    #define DEBUG(str)
    #define DEBUGLN(str)
#endif

#endif //THERMOSTAT_H
