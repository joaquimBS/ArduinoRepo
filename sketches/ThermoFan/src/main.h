/************************************************************************************
 * 	
 * 	Name     : main.h
 * 	Author   : Joaquim Barrera
 * 	Date     : 2018
 * 	Version  : 1
 * 	Notes    : 
 * 
 ***********************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"

#define SET_DIGITAL_PINS_AS_INPUTS()        \
        uint8_t p;                          \
        for(p=0; p<DIGITAL_PIN_COUNT; p++) {\
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        }while(0)  // This while is to allow ';' at the end of the macro

// I/O Digital Pins
enum {
    DHT22_PIN = 0,
    INFO_LED,
    RELAY_TRIGGER,
    D3,
    D4,
    D5,
    DIGITAL_PIN_COUNT
};

#define A0 
#define A1 
#define A2 
#define A3 
#define SDA A4 
#define SCL A5 
#define A6 
#define A7 

#define NULL_PTR (void*)0

#define LED_ON  digitalWrite(INFO_LED, HIGH)
#define LED_OFF digitalWrite(INFO_LED, LOW)

#define RELAY_ON  digitalWrite(RELAY_TRIGGER, LOW)
#define RELAY_OFF digitalWrite(RELAY_TRIGGER, HIGH)

//#define USE_DEBUG
#define SERIAL_BR 115200

#if defined(USE_DEBUG)
#define DEBUG(str)   Serial.print(F(str))
#define DEBUGVAL(str, val) \
        Serial.print(__func__); \
        Serial.print(F("[")); Serial.print(__LINE__); Serial.print(F("] ")); \
        Serial.print(F(str)); Serial.println(val); 
#define DEBUGLN(str) \
        Serial.print(__func__); \
        Serial.print(F("[")); Serial.print(__LINE__); Serial.print(F("] ")); \
        Serial.println(F(str))
#else
#define DEBUG(str)
#define DEBUGLN(str)
#define DEBUGVAL(str, val)
#endif

#endif //MAIN_H
