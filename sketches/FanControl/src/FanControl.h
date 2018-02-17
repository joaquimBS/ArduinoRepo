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

#ifndef FANCONTROL_H
#define FANCONTROL_H

#include "Arduino.h"

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<DIGITAL_PIN_COUNT; p++) {\
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        }while(0)  // This while is to allow ';' at the end of the macro

// I/O Digital Pins
enum {
    D0 = 0,
    INFO_LED,
    TEMP_LM35,
    D3,
    D4,
    D5,
    DIGITAL_PIN_COUNT
};

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

#define SERIAL_BR 115200
#define USE_DEBUG

#if defined(USE_DEBUG)
#define DEBUG(str)   DigiUSB.print(str)
#define DEBUGLN(str) DigiUSB.println(str)
#else
#define DEBUG(str)
#define DEBUGLN(str)
#endif

#endif //FANCONTROL_H
