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
    D0, /* RX */
    D1, /* TX */
    D2_RESERVED_RADIO,
    BUTTON_CTRL, /* Future RELAY_FEEDBACK */
    RELAY_PLUS,
    RELAY_MINUS,
    DHT_PIN,
    OLED_VCC,
    FLASH_SS,
    INFO_LED,
    D10_RESERVED_RADIO,
    D11_RESERVED_RADIO, /* MOSI */
    D12_RESERVED_RADIO, /* MISO */
    D13_RESERVED_RADIO, /* SCK */
    BUTTON_UP,
    BUTTON_DOWN,
    D16, /* Future BUTTON_CTRL */
    RTC_VCC,
    D18,
    D19,
    DIGITAL_PIN_COUNT
};

#define LED_ON      digitalWrite(INFO_LED, HIGH)
#define LED_OFF     digitalWrite(INFO_LED, LOW)

// #define USE_DEBUG

#if defined(USE_DEBUG)
#define DEBUG(str)   Serial.print(str)
#define DEBUGLN(str) Serial.println(str)
#else
#define DEBUG(str)
#define DEBUGLN(str)
#endif

#endif //THERMOSTAT_H
