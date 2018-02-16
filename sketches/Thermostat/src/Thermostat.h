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
        for(p=0; p<DIGITAL_PIN_COUNT; p++) {\
            pinMode(p, INPUT);              \
            digitalWrite(p, LOW);           \
        }while(0)  // This while is to allow ';' at the end of the macro

// I/O Digital Pins
enum {
    D0 = (uint8_t)0, /* RX */
    D1, /* TX */
    D2_RESERVED_RADIO,
    BUTTON_CTRL, /* Future RELAY_FEEDBACK */
    RELAY_PLUS,
    RELAY_MINUS,
    DHT_PIN,
    OLED_VCC,
    FLASH_SS,
    INFO_LED, /* BUILT-IN LED */
    D10_RESERVED_RADIO,
    D11_RESERVED_RADIO, /* MOSI */
    D12_RESERVED_RADIO, /* MISO */
    D13_RESERVED_RADIO, /* SCK */
    BUTTON_DOWN,
    BUTTON_UP,
    RELAY_FEEDBACK,
    RTC_VCC,
    D18,
    D19,
    DIGITAL_PIN_COUNT
};

#define A0 
#define A1 
#define A2 
#define A3 
#define SDA A4 
#define SCL A5 
#define A6 
#define VBAT_IN A7 

#define NULL_PTR (void*)0

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

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

#endif //THERMOSTAT_H
